"""What you see: a camera behind the ego car, a HUD, a video file and a live stream.

On a cloud machine without a screen the CARLA server renders off screen, so
the bridge attaches its own camera to the ego car and
- writes the frames to an MP4 (results/carla/<controller>_<dynamics>.mp4), and
- optionally serves them as an MJPEG stream on http://localhost:PORT/, to
  watch live from a laptop through an SSH tunnel (README: "Watching live").

The HUD shows the controller, the dynamics, the time, speed, lateral offset,
sideslip and the NMPC's solve time, with a map of the track. A panel on the
right (off with --no-rates) plots how fast the controller and the plant run in
wall-clock time, and with the threaded bicycle plant the CARLA world thread too.
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
        # BGRA to RGB, in PIL's C code: a third of the time numpy's reversed-stride copy takes
        rgb = Image.frombuffer("RGB", (image.width, image.height), bytes(image.raw_data), "raw", "BGRX", 0, 1)
        return np.asarray(rgb)

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
        # the map's box and track, drawn once (they are most of the HUD's drawing time) and pasted every frame
        ox, oy = self.offset
        self.map_layer = Image.new("RGBA", (map_size + 1, map_size + 1), (0, 0, 0, 150))
        ImageDraw.Draw(self.map_layer).line([(x - ox, y - oy) for x, y in self.line + [self.line[0]]],
                                            fill=(200, 200, 200, 255), width=3)

    def _map(self, x, y):
        return (self.offset[0] + 10 + (x - self.x0) * self.scale, self.offset[1] + 10 + (self.y0 - y) * self.scale)

    def draw(self, rgb, lines, ego_xy, traffic_xy, banner=None):
        """`banner`: (text, detail, (r, g, b)) shown in that colour under the text panel, e.g. which controller
        is in command."""
        image = Image.fromarray(rgb)
        d = ImageDraw.Draw(image, "RGBA")
        # text panel
        height = 12 + 26 * len(lines)
        d.rectangle([10, 10, 470, 10 + height], fill=(0, 0, 0, 150))
        for i, text in enumerate(lines):
            d.text((22, 18 + 26 * i), text, font=self.font, fill=(255, 255, 255, 255))
        if banner:
            text, detail, color = banner
            top = 18 + height
            d.rectangle([10, top, 470, top + (58 if detail else 36)], fill=tuple(c // 3 for c in color) + (220,),
                        outline=tuple(color) + (255,), width=3)
            d.text((22, top + 7), text, font=self.font, fill=tuple(color) + (255,))
            if detail:
                d.text((22, top + 34), detail, font=self.small, fill=(235, 235, 235, 255))
        # map
        ox, oy = self.offset
        image.paste(self.map_layer, (ox, oy), self.map_layer)
        for x, y in traffic_xy:
            px, py = self._map(x, y)
            d.ellipse([px - 4, py - 4, px + 4, py + 4], fill=(80, 160, 255, 255))
        px, py = self._map(*ego_xy)
        d.ellipse([px - 6, py - 6, px + 6, py + 6], fill=(255, 60, 60, 255))
        return np.asarray(image)


COMMAND_COLORS = {"nmpc": (90, 220, 120), "pure_pursuit": (255, 170, 60), "none": (200, 200, 200)}


def command_banner(status, deadline):
    """The HUD banner saying whose input the plant is getting: trackctl.FallbackControl.status()."""
    source = status["source"]
    ticks = max(1, status["ticks"])
    share = "NMPC %.0f%%" % (100.0 * status["nmpc_ticks"] / ticks)
    if source == "nmpc":
        return ("COMMAND: NMPC", "plan %3.0f ms old, solve %3.0f ms  %s" % (
            1e3 * status["plan_age"], 1e3 * status["last_solve"], share), COMMAND_COLORS[source])
    if source == "pure_pursuit":
        return ("COMMAND: PURE PURSUIT (fallback)", "%s  %s" % (status["reason"], share), COMMAND_COLORS[source])
    return ("COMMAND: waiting", "deadline %.0f ms" % (1e3 * deadline), COMMAND_COLORS["none"])


class RatePanel:
    """A panel to the right of the camera view plotting the rate, in wall-clock Hz, of each series: its mean
    over the last second against simulated time, its average over the run, and its real-time rate (the
    dashed line).

    In lockstep (one loop: controller, then plant and CARLA tick) `add` takes the wall seconds each one took
    in a step, and the rate is how many steps a second each could manage. With threads, `add_counts` takes
    each thread's event count and the wall time since the start, and the rate is the frequency it ran at."""

    WIDTH = 424  # 1280 + 424: a multiple of 8, so the video is not resized
    WINDOW = 30.0  # seconds of simulated time shown
    LOCKSTEP = (("controller", "Controller", (255, 170, 60)), ("plant", "Plant step (incl. CARLA tick)", (80, 200, 255)))
    THREADS = (("controller", "Controller thread", (255, 170, 60)), ("plant", "Plant thread", (80, 200, 255)),
               ("world", "CARLA world thread", (150, 220, 120)))
    FALLBACK = (("command", "Command thread", (235, 235, 235)),
                ("nmpc", "NMPC solves", (90, 220, 120)), ("pure_pursuit", "Pure pursuit thread", (255, 170, 60)),
                ("plant", "Plant thread", (80, 200, 255)), ("world", "CARLA world thread", (190, 150, 255)))

    def __init__(self, dt, height=720, series=LOCKSTEP, real_time=None, refresh=0.2):
        """`real_time`: each series' real-time rate [Hz] (default 1 / dt). The panel is redrawn every `refresh`
        seconds of samples and reused in between: its text is most of a frame's drawing time."""
        self.dt, self.height, self.series, self.refresh = dt, height, series, refresh
        self.lock = threading.Lock()  # samples are added by the world thread and drawn by the view thread
        self.cached, self.cached_t = None, -math.inf
        self.real_time = {key: (real_time or {}).get(key, 1.0 / dt) for key, _, _ in series}
        self.smooth = max(1, round(1.0 / dt))  # samples in a second
        self.t = []
        self.events = {key: [] for key, _, _ in series}  # cumulative events at each sample
        self.wall = {key: [] for key, _, _ in series}  # cumulative wall seconds at each sample
        try:
            self.font = ImageFont.truetype("DejaVuSansMono.ttf", 18)
            self.big = ImageFont.truetype("DejaVuSansMono.ttf", 26)
            self.small = ImageFont.truetype("DejaVuSansMono.ttf", 13)
        except OSError:
            self.font = self.big = self.small = ImageFont.load_default()

    def add(self, t, controller, plant):
        """One lockstep step at simulated time t: the controller and the plant took these many wall seconds."""
        counts = {}
        for key, seconds in (("controller", controller), ("plant", plant)):
            events, wall = (self.events[key][-1], self.wall[key][-1]) if self.t else (0, 0.0)
            counts[key] = (events + 1, wall + seconds)
        self.add_counts(t, counts)

    def add_counts(self, t, counts):
        """A sample at simulated time t: {series: (events so far, wall seconds so far)}."""
        with self.lock:
            self.t.append(t)
            for key, _, _ in self.series:
                events, wall = counts[key]
                self.events[key].append(events)
                self.wall[key].append(wall)

    def _snapshot(self):
        """(t, events, wall) as arrays, copied together: the world thread may be adding a sample."""
        with self.lock:
            return (np.asarray(self.t, float), {k: np.asarray(v, float) for k, v in self.events.items()},
                    {k: np.asarray(v, float) for k, v in self.wall.items()})

    def average(self, key, snapshot=None):
        """Events per wall second over the whole run."""
        t, events, wall = snapshot or self._snapshot()
        if not t.size or wall[key][-1] <= 0.0:
            return float("nan")
        return events[key][-1] / wall[key][-1]

    def _rolling(self, key, snapshot):
        """Events per wall second over the last second of samples, at each sample; none over the first quarter
        second, where too few samples would make a spike."""
        _, events, wall = snapshot
        events = np.concatenate([[0.0], events[key]])
        wall = np.concatenate([[0.0], wall[key]])
        i = np.arange(1, len(events))
        lo = np.maximum(0, i - self.smooth)
        span = wall[i] - wall[lo]
        valid = (span > 0.0) & (i >= max(2, self.smooth // 4))
        return np.where(valid, (events[i] - events[lo]) / np.where(span > 0.0, span, 1.0), np.nan)

    def draw(self, rgb):
        """The camera image with the panel added on its right."""
        with self.lock:
            t = self.t[-1] if self.t else 0.0
        if self.cached is None or self.cached.shape[0] != rgb.shape[0] or t - self.cached_t >= self.refresh:
            self.cached, self.cached_t = self._render(rgb.shape[0]), t
        return np.concatenate([rgb, self.cached], axis=1)

    def _render(self, height):
        snapshot = self._snapshot()
        panel = Image.new("RGB", (self.WIDTH, height), (18, 18, 22))
        d = ImageDraw.Draw(panel)
        d.text((16, 12), "Rate (wall clock)", font=self.font, fill=(235, 235, 235))
        top, plot_h = 46, (height - 46) // len(self.series)
        for k, (key, title, color) in enumerate(self.series):
            self._plot(d, top + k * plot_h, plot_h, key, title, color, snapshot, compact=len(self.series) > 3)
        return np.asarray(panel)

    def _plot(self, d, y0, h, key, title, color, snapshot, compact=False):
        """With `compact` (many series) the title and average share a line above the plot."""
        left, right = 62, self.WIDTH - 14
        avg = self.average(key, snapshot)
        if compact:
            top, bottom = y0 + 30, y0 + h - 22
            d.text((16, y0 + 6), title, font=self.small, fill=(190, 190, 190))
            d.text((right - 150, y0 + 1), "avg " + _hz(avg), font=self.font, fill=color)
        else:
            top, bottom = y0 + 66, y0 + h - 30
            d.text((16, y0 + 4), title, font=self.small, fill=(190, 190, 190))
            d.text((16, y0 + 24), "avg " + _hz(avg), font=self.big, fill=color)
        d.rectangle([left, top, right, bottom], outline=(70, 70, 78))
        t = snapshot[0]
        if not t.size:
            return
        rate = self._rolling(key, snapshot)
        shown = t >= t[-1] - self.WINDOW
        t0 = max(0.0, t[-1] - self.WINDOW)
        real_time = self.real_time[key]
        visible = rate[shown][np.isfinite(rate[shown])]
        y_max = 1.15 * max([real_time, avg if math.isfinite(avg) else 0.0] + list(visible[-1:]) +
                           ([float(visible.max())] if visible.size else []))

        def px(ti, ri):
            return (left + (ti - t0) / self.WINDOW * (right - left), bottom - ri / y_max * (bottom - top))

        for frac in (0.0, 0.5, 1.0):  # the y axis
            y = bottom - frac * (bottom - top)
            d.line([left - 4, y, left, y], fill=(120, 120, 128))
            d.text((4, y - 7), _hz(frac * y_max, short=True), font=self.small, fill=(150, 150, 158))
        d.text((left, bottom + 4), "%.0f s" % t0, font=self.small, fill=(150, 150, 158))
        d.text((right - 48, bottom + 4), "%.0f s" % t[-1], font=self.small, fill=(150, 150, 158))
        # real time (dashed) and the run's average
        y = px(t0, real_time)[1]
        for x in range(left, right, 12):
            d.line([x, y, min(x + 6, right), y], fill=(150, 150, 158))
        d.text((left + 4, y - 16), "real time %s" % _hz(real_time), font=self.small, fill=(150, 150, 158))
        if math.isfinite(avg):
            y = px(t0, avg)[1]
            d.line([left, y, right, y], fill=tuple(c // 2 for c in color), width=1)
        points = [px(ti, ri) for ti, ri in zip(t[shown], rate[shown]) if math.isfinite(ri)]
        if len(points) > 1:
            d.line(points, fill=color, width=2)


def _hz(rate, short=False):
    if not math.isfinite(rate):
        return "-"
    if rate >= 1000.0:
        return ("%.0fk" if short else "%.1f kHz") % (rate / 1000.0)
    return ("%.0f" if short else "%.1f Hz") % rate


class Video:
    """MP4 writer (ffmpeg from the imageio-ffmpeg package)."""

    def __init__(self, path, size, fps):
        import imageio_ffmpeg

        # veryfast: a fraction of the default preset's CPU, which the CARLA server and the controllers need
        self.writer = imageio_ffmpeg.write_frames(path, size, fps=fps, codec="libx264", quality=7,
                                                  macro_block_size=8, ffmpeg_log_level="error",
                                                  output_params=["-preset", "veryfast"])
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
        self.viewers = 0  # open /stream connections: no JPEG is made without one
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
                with stream.ready:
                    stream.viewers += 1
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
                finally:
                    with stream.ready:
                        stream.viewers -= 1

        self.server = ThreadingHTTPServer(("0.0.0.0", port), Handler)
        self.server.daemon_threads = True
        threading.Thread(target=self.server.serve_forever, daemon=True).start()

    def add(self, rgb):
        if not self.viewers:
            return
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
        "%s on %s dynamics" % ({"nmpc": "NMPC", "pure_pursuit": "Pure pursuit",
                                "nmpc_fallback": "NMPC+PP fallback"}.get(controller, controller),
                               {"carla": "CARLA", "chrono": "Project Chrono"}.get(dynamics, dynamics)),
        "t %6.1f s   lap %5.1f %%" % (t, 100.0 * progress / lap_length),
        "speed %5.1f km/h" % speed,
        "lateral offset %+5.2f m" % e_y,
        "sideslip %+5.1f deg" % sideslip,
    ]
    if solve_time is not None:
        lines.append("NMPC solve %4.0f ms" % (1e3 * solve_time))
    return lines
