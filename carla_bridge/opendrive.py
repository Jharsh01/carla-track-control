"""The race track as an OpenDRIVE road, for CARLA's generate_opendrive_world.

The track's centreline (the lane divider) becomes the road's reference line,
piece by piece: a straight is a <line>, a corner an <arc>, exactly as the
controllers see it. Across the road, from the reference line outwards on each
side: a 3.5 m asphalt lane, then the grass strip. CARLA builds a road mesh
for every lane and a wall at the outer edge of the outermost ones; the grass
lanes are typed "driving" because CARLA meshes other lane types as raised
sidewalks. The grass is therefore asphalt-coloured in CARLA; its lower friction
is applied by the Chrono plant only (see the README).

Both lanes are driven in the same direction, which OpenDRIVE does not express
for a two-way road type; CARLA uses lane directions only for its own waypoint
API and traffic, neither of which this bridge uses.
"""
import math
from xml.sax.saxutils import quoteattr


def build(circuit, name="racetrack"):
    """OpenDRIVE (1.4) text for `circuit` (a trackctl.Circuit)."""
    track = circuit.track
    lengths, curvatures, poses = track.piece_lengths, track.piece_curvatures, track.piece_poses
    xs = [p.x for p in poses]
    ys = [p.y for p in poses]
    margin = circuit.lane_width + circuit.grass_width + 10.0

    geometry = []
    s = 0.0
    for length, kappa, pose in zip(lengths, curvatures, poses):
        shape = "<line/>" if kappa == 0.0 else '<arc curvature="%.12g"/>' % kappa
        geometry.append('      <geometry s="%.12g" x="%.12g" y="%.12g" hdg="%.12g" length="%.12g">%s</geometry>'
                        % (s, pose.x, pose.y, pose.psi, length, shape))
        s += length

    def lane(lane_id, width, mark):
        return ('          <lane id="%d" type="driving" level="false">\n'
                '            <link/>\n'
                '            <width sOffset="0" a="%.6g" b="0" c="0" d="0"/>\n'
                '            <roadMark sOffset="0" type="%s" weight="standard" color="standard" width="0.15" laneChange="none"/>\n'
                '          </lane>' % (lane_id, width, mark))

    lw, gw = circuit.lane_width, circuit.grass_width
    return "\n".join([
        '<?xml version="1.0" standalone="yes"?>',
        '<OpenDRIVE>',
        '  <header revMajor="1" revMinor="4" name=%s version="1.00" north="%.6g" south="%.6g" east="%.6g" west="%.6g"/>'
        % (quoteattr(name), max(ys) + margin, min(ys) - margin, max(xs) + margin, min(xs) - margin),
        '  <road name=%s length="%.12g" id="1" junction="-1">' % (quoteattr(name), track.length),
        '    <link/>',
        '    <type s="0" type="town"><speed max="%.6g" unit="km/h"/></type>' % (circuit.v_max * 3.6),
        '    <planView>',
        *geometry,
        '    </planView>',
        '    <elevationProfile><elevation s="0" a="0" b="0" c="0" d="0"/></elevationProfile>',
        '    <lateralProfile/>',
        '    <lanes>',
        '      <laneSection s="0">',
        '        <left>',
        lane(2, gw, "none"),
        lane(1, lw, "solid"),
        '        </left>',
        '        <center>',
        '          <lane id="0" type="driving" level="false">',
        '            <roadMark sOffset="0" type="broken" weight="standard" color="standard" width="0.15" laneChange="both"/>',
        '          </lane>',
        '        </center>',
        '        <right>',
        lane(-1, lw, "solid"),
        lane(-2, gw, "none"),
        '        </right>',
        '      </laneSection>',
        '    </lanes>',
        '  </road>',
        '</OpenDRIVE>',
        '',
    ])


def reference_line(xodr_text, step=1.0):
    """Points (x, y, hdg) along the reference line of the first road, every `step` metres; for checks."""
    import xml.etree.ElementTree as ET

    root = ET.fromstring(xodr_text)
    points = []
    for g in root.find("road").find("planView"):
        x, y, hdg, length = (float(g.get(k)) for k in ("x", "y", "hdg", "length"))
        arc = g.find("arc")
        kappa = float(arc.get("curvature")) if arc is not None else 0.0
        n = max(1, int(math.ceil(length / step)))
        for i in range(n):
            d = length * i / n
            if kappa == 0.0:
                points.append((x + d * math.cos(hdg), y + d * math.sin(hdg), hdg))
            else:
                end = hdg + kappa * d
                points.append((x + (math.sin(end) - math.sin(hdg)) / kappa,
                               y - (math.cos(end) - math.cos(hdg)) / kappa, end))
    return points
