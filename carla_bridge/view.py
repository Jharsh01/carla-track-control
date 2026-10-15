"""What you see: a camera behind the ego car, a HUD, a video file and a live stream.

On a cloud machine without a screen the CARLA server renders off screen, so
the bridge attaches its own camera to the ego car and
- writes the frames to an MP4 (results/carla/<controller>_<dynamics>.mp4), and
- optionally serves them as an MJPEG stream on http://localhost:PORT/, to
  watch live from a laptop through an SSH tunnel (README: "Watching live").

The HUD shows the controller, the dynamics, the time, speed, lateral offset,
sideslip and the NMPC's solve time, with a map of the track.
"""
import io
import math
import queue
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
from PIL import Image, ImageDraw, ImageFont

import carla


class ChaseCamera:
    """An RGB camera behind and above the ego car, read synchronously each tick."""

    def __init__(self, world, ego, width=1280, height=720, fov=90.0):
        bp = world.get_blueprint_library().find("sensor.camera.rgb")
        bp.set_attribute("image_size_x", str(width))
        bp.set_attribute("image_size_y", str(height))
        bp.set_attribute("fov", str(fov))
        mount = carla.Transform(carla.Location(x=-7.0, z=3.0), carla.Rotation(pitch=-12.0))
        self.sensor = world.spawn_actor(bp, mount, attach_to=ego,
                                        attachment_type=carla.AttachmentType.SpringArmGhost)
        self.frames = queue.Queue()
        self.sensor.listen(self.frames.put)

    def read(self, frame, timeout=10.0):
        """The image of world frame `frame` as an RGB array (H, W, 3)."""
        while True:
            image = self.frames.get(timeout=timeout)
            if image.frame >= frame:
                break
        data = np.frombuffer(image.raw_data, dtype=np.uint8).reshape(image.height, image.width, 4)
        return data[:, :, 2::-1].copy()  # BGRA to RGB

    def destroy(self):
        self.sensor.stop()
        self.sensor.destroy()


class Hud:
    def __init__(self, circuit, size=(1280, 720), map_size=260):
        self.circuit = circuit
        self.map_size = map_size
        try:
            self.font = ImageFont.truetype("DejaVuSansMono.ttf", 20)
            self.small = ImageFont.truetype("DejaVuSansMono.ttf", 15)
        except OSError:
            self.font = self.small = ImageFont.load_default()
        # the centreline, scaled into a square in the top-right corner
        track = circuit.track
        pts = [track.pose(s) for s in np.arange(0.0, track.length, 2.0)]
        xs, ys = [p.x for p in pts], [p.y for p in pts]
        self.x0, self.y0 = min(xs), max(ys)
        self.scale = (map_size - 20) / max(max(xs) - min(xs), max(ys) - min(ys))
        self.offset = (size[0] - map_size - 10, 10)
        self.line = [self._map(p.x, p.y) for p in pts]

    def _map(self, x, y):
        return (self.offset[0] + 10 + (x - self.x0) * self.scale, self.offset[1] + 10 + (self.y0 - y) * self.scale)

    def draw(self, rgb, lines, ego_xy, traffic_xy):
        image = Image.fromarray(rgb)
        d = ImageDraw.Draw(image, "RGBA")
        # text panel
        height = 12 + 26 * len(lines)
        d.rectangle([10, 10, 470, 10 + height], fill=(0, 0, 0, 150))
        for i, text in enumerate(lines):
            d.text((22, 18 + 26 * i), text, font=self.font, fill=(255, 255, 255, 255))
        # map
        ox, oy = self.offset
        d.rectangle([ox, oy, ox + self.map_size, oy + self.map_size], fill=(0, 0, 0, 150))
        d.line(self.line + [self.line[0]], fill=(200, 200, 200, 255), width=3)
        for x, y in traffic_xy:
            px, py = self._map(x, y)
            d.ellipse([px - 4, py - 4, px + 4, py + 4], fill=(80, 160, 255, 255))
        px, py = self._map(*ego_xy)
        d.ellipse([px - 6, py - 6, px + 6, py + 6], fill=(255, 60, 60, 255))
        return np.asarray(image)


class Video:
    """MP4 writer (ffmpeg from the imageio-ffmpeg package)."""

    def __init__(self, path, size, fps):
        import imageio_ffmpeg

        self.writer = imageio_ffmpeg.write_frames(path, size, fps=fps, codec="libx264", quality=7,
                                                  macro_block_size=8, ffmpeg_log_level="error")
        self.writer.send(None)

    def add(self, rgb):
        self.writer.send(np.ascontiguousarray(rgb))

    def close(self):
        self.writer.close()


class Stream:
    """The latest frame as an MJPEG stream on http://0.0.0.0:port/ (a page) and /stream (the stream)."""

    PAGE = (b"<!doctype html><title>carla-track-control</title>"
            b"<body style='margin:0;background:#111'><img src='/stream' style='width:100%'></body>")

    def __init__(self, port):
        self.jpeg = None
        self.ready = threading.Condition()
        stream = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def do_GET(self):
                if self.path != "/stream":
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html")
                    self.end_headers()
                    self.wfile.write(Stream.PAGE)
                    return
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                try:
                    while True:
                        with stream.ready:
                            stream.ready.wait(timeout=5.0)
                            jpeg = stream.jpeg
                        if jpeg is None:
                            continue
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(jpeg))
                        self.wfile.write(jpeg + b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    pass

        self.server = ThreadingHTTPServer(("0.0.0.0", port), Handler)
        self.server.daemon_threads = True
        threading.Thread(target=self.server.serve_forever, daemon=True).start()

    def add(self, rgb):
        buffer = io.BytesIO()
        Image.fromarray(rgb).save(buffer, format="JPEG", quality=80)
        with self.ready:
            self.jpeg = buffer.getvalue()
            self.ready.notify_all()

    def close(self):
        self.server.shutdown()
        self.server.server_close()


def hud_lines(controller, dynamics, t, x, e_y, progress, lap_length, solve_time):
    speed = 3.6 * x[3]
    sideslip = math.degrees(math.atan2(x[4], x[3])) if x[3] > 0.5 else 0.0
    lines = [
        "%s on %s dynamics" % ({"nmpc": "NMPC", "pure_pursuit": "Pure pursuit"}.get(controller, controller),
                               {"carla": "CARLA", "chrono": "Project Chrono"}.get(dynamics, dynamics)),
        "t %6.1f s   lap %5.1f %%" % (t, 100.0 * progress / lap_length),
        "speed %5.1f km/h" % speed,
        "lateral offset %+5.2f m" % e_y,
        "sideslip %+5.1f deg" % sideslip,
    ]
    if solve_time is not None:
        lines.append("NMPC solve %4.0f ms" % (1e3 * solve_time))
    return lines
