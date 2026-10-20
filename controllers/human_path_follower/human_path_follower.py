"""Walk the human along a closed path so the robot has someone to track.

The path is the list of Pose children of the world's DEF HUMAN_PATH group, taken in
order and closed back to the first one. The controller smooths them with a
Catmull-Rom spline, walks it at constant speed and turns the body to face along it.
It moves the node by writing its translation and rotation, so the human needs no
physics and pushes nothing out of the way.

controllerArgs (all optional):
  --speed=0.6        walking speed, m/s
  --pause=0.0        seconds to stand still at every waypoint
  --yaw_offset=0.0   added to the heading, rad, if the mesh does not face +x
  --start=0          index of the waypoint the walk starts from
  --show_path=1      draw the loop as a line on the floor (0 hides it; cameras see it)
"""

import math
import sys

from controller import Supervisor

HUMAN_DEF = "HUMAN_Manual"
PATH_DEF = "HUMAN_PATH"
SAMPLES_PER_SEGMENT = 50


def parse_args(argv):
    args = {"speed": 0.6, "pause": 0.0, "yaw_offset": 0.0, "start": 0, "show_path": 1}
    for a in argv[1:]:
        if a.startswith("--") and "=" in a:
            key, value = a[2:].split("=", 1)
            if key in args:
                args[key] = type(args[key])(float(value))
    return args


def catmull_rom(p0, p1, p2, p3, t):
    t2, t3 = t * t, t * t * t
    return tuple(
        0.5 * (2 * b + (-a + c) * t + (2 * a - 5 * b + 4 * c - d) * t2
               + (-a + 3 * b - 3 * c + d) * t3)
        for a, b, c, d in zip(p0, p1, p2, p3))


def build_path(waypoints):
    """Densely sample the closed spline. Returns points, cumulative lengths and the
    sample index of every waypoint."""
    n = len(waypoints)
    points, marks = [], []
    for i in range(n):
        marks.append(len(points))
        p0, p1, p2, p3 = (waypoints[(i + k) % n] for k in (-1, 0, 1, 2))
        for s in range(SAMPLES_PER_SEGMENT):
            points.append(catmull_rom(p0, p1, p2, p3, s / SAMPLES_PER_SEGMENT))
    points.append(points[0])
    lengths = [0.0]
    for a, b in zip(points, points[1:]):
        lengths.append(lengths[-1] + math.dist(a, b))
    return points, lengths, marks


def draw_path(robot, points, waypoints):
    """Add the loop to the scene as an unlit line just above the floor, with a short
    cross at every waypoint. It is a bare Transform, so nothing collides with it."""
    line = " ".join(f"{x:.3f} {y:.3f} 0.01" for x, y in points[:-1])
    n = len(points) - 1
    line_idx = " ".join(str(i) for i in range(n)) + " 0 -1"
    cross, cross_idx = [], []
    for k, (x, y) in enumerate(waypoints):
        cross += [f"{x - 0.15:.3f} {y:.3f} 0.01", f"{x + 0.15:.3f} {y:.3f} 0.01",
                  f"{x:.3f} {y - 0.15:.3f} 0.01", f"{x:.3f} {y + 0.15:.3f} 0.01"]
        b = 4 * k
        cross_idx.append(f"{b} {b + 1} -1 {b + 2} {b + 3} -1")

    def shape(coords, idx, color):
        return (f"Shape {{ appearance Appearance {{ material Material {{ "
                f"diffuseColor 0 0 0 emissiveColor {color} }} }} "
                f"geometry IndexedLineSet {{ coord Coordinate {{ point [ {coords} ] }} "
                f"coordIndex [ {idx} ] }} }}")

    old = robot.getFromDef("HUMAN_PATH_VIEW")  # left behind by a save while running
    if old is not None:
        old.remove()
    robot.getRoot().getField("children").importMFNodeFromString(
        -1, "DEF HUMAN_PATH_VIEW Transform { children [ "
        + shape(line, line_idx, "1 0.5 0") + " "
        + shape(" ".join(cross), " ".join(cross_idx), "1 1 0") + " ] }")


def main():
    args = parse_args(sys.argv)
    robot = Supervisor()
    step = int(robot.getBasicTimeStep())

    human = robot.getFromDef(HUMAN_DEF)
    path_node = robot.getFromDef(PATH_DEF)
    if human is None or path_node is None:
        print(f"[human_path_follower] DEF {HUMAN_DEF} or DEF {PATH_DEF} not found, idle")
        while robot.step(step) != -1:
            pass
        return

    children = path_node.getField("children")
    waypoints = [tuple(children.getMFNode(i).getField("translation").getSFVec3f()[:2])
                 for i in range(children.getCount())]
    if len(waypoints) < 2:
        print(f"[human_path_follower] {PATH_DEF} needs at least 2 waypoints, idle")
        while robot.step(step) != -1:
            pass
        return

    points, lengths, marks = build_path(waypoints)
    total = lengths[-1]
    mark_dist = [lengths[m] for m in marks]
    print(f"[human_path_follower] {len(waypoints)} waypoints, loop {total:.1f} m, "
          f"{total / args['speed']:.0f} s per lap at {args['speed']} m/s")

    if args["show_path"]:
        draw_path(robot, points, waypoints)

    tr_field = human.getField("translation")
    rot_field = human.getField("rotation")
    z = tr_field.getSFVec3f()[2]

    s = mark_dist[args["start"] % len(waypoints)]
    next_mark = (args["start"] + 1) % len(waypoints)
    paused = 0.0
    j = 0
    dt = step / 1000.0

    while robot.step(step) != -1:
        if paused > 0.0:
            paused -= dt
            continue

        prev_s = s
        s = (s + args["speed"] * dt) % total
        crossed = (prev_s < mark_dist[next_mark] <= s if s >= prev_s
                   else mark_dist[next_mark] > prev_s or mark_dist[next_mark] <= s)
        if crossed:
            next_mark = (next_mark + 1) % len(waypoints)
            paused = args["pause"]

        # Find the segment containing s, searching forward from the last one.
        if s < lengths[j]:
            j = 0
        while lengths[j + 1] < s:
            j += 1
        a, b = points[j], points[j + 1]
        seg = lengths[j + 1] - lengths[j]
        u = (s - lengths[j]) / seg if seg > 0 else 0.0
        x = a[0] + u * (b[0] - a[0])
        y = a[1] + u * (b[1] - a[1])
        yaw = math.atan2(b[1] - a[1], b[0] - a[0]) + args["yaw_offset"]

        tr_field.setSFVec3f([x, y, z])
        rot_field.setSFRotation([0, 0, 1, yaw])


if __name__ == "__main__":
    main()
