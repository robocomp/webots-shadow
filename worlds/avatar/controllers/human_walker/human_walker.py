#!/usr/bin/env python3
"""High-level command controller for the Webots `Pedestrian` PROTO.

Cyberbotics' stock `pedestrian` controller plays a fixed walk cycle indexed by
*absolute simulation time* along a *fixed* waypoint polyline given on the
command line.  That makes it impossible to stop, reverse, re-steer or pose the
model at run time.  This controller keeps their (good) empirical gait and
replaces the clock and the interface:

  * the gait phase is advanced by the distance actually travelled, so the legs
    follow the body instead of the wall clock;
  * the root pose is integrated from a unicycle command (v, omega) rather than
    interpolated along a precomputed path;
  * commands arrive at run time as newline-delimited JSON over TCP, so any
    process -- an agent, a test, or `nc` -- can drive the human;
  * the arms can be taken off the gait and placed by inverse kinematics.

The Pedestrian carries no motors and no Physics, so this is kinematic
animation: the model is posed and moved, never simulated.  That is the right
trade for a moving person in a perception or navigation scene (it never falls
over and never needs balancing), and the wrong one if you need contact
dynamics.

Run it by pointing the Pedestrian node at this controller:

    Pedestrian {
      name "human"
      controller "human_walker"
      controllerArgs [ "--port=4321" ]
      enableBoundingObject TRUE
    }

`controllerArgs` MUST be non-empty.  The PROTO's template reads
`const rigid = fields.controllerArgs.value.length == 0` and, when empty,
replaces every HingeJoint with a fixed Solid -- a statue with joint fields that
silently accept writes and never move.  This controller detects that case at
startup and says so.
"""

import argparse
import json
import math
import socket
import sys
import threading

from controller import Supervisor

from pedestrian_kinematics import (
    ARMS, JOINT_NAMES, WalkCycle, clamp, root_to_world,
    solve_body_yaw_for_target, world_to_root, wrap_angle,
)

# Steering law shared by goto / follow / face.
K_YAW = 1.8              # rad/s per rad of heading error
APPROACH_DISTANCE = 0.6  # m over which the cruise speed tapers to zero
ARM_BLEND_TIME = 0.35    # s to fade an arm between the gait and a held pose
STOP_BLEND_TIME = 0.45   # s for the whole body to settle into a neutral stance
FACE_TOLERANCE = 0.005   # rad; a reach only aligns as well as the body does

MODE_VELOCITY = "velocity"
MODE_GOTO = "goto"
MODE_FOLLOW = "follow"
MODE_FACE = "face"


class _Pending:
    """One queued command and the slot its reply goes into."""

    def __init__(self, request):
        self.request = request
        self.reply = None
        self.done = threading.Event()

    def resolve(self, reply):
        self.reply = reply
        self.done.set()


class CommandServer(threading.Thread):
    """Newline-delimited JSON over TCP.  One request line, one reply line.

    Requests are handed to the simulation thread and answered only after the
    step that applied them, so a reply -- and in particular a `state` reply --
    describes the world as it is once the command has taken effect.
    """

    def __init__(self, host, port, submit):
        super().__init__(daemon=True)
        self.host, self.port, self.submit = host, port, submit
        self._socket = None
        self._stop = threading.Event()

    def run(self):
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self._socket.bind((self.host, self.port))
            self._socket.listen(8)
        except OSError as exc:
            print("[human_walker] cannot listen on %s:%d -- %s"
                  % (self.host, self.port, exc), file=sys.stderr, flush=True)
            return
        print("[human_walker] command server on %s:%d" % (self.host, self.port),
              flush=True)
        while not self._stop.is_set():
            try:
                conn, addr = self._socket.accept()
            except OSError:
                break
            threading.Thread(target=self._serve, args=(conn, addr),
                             daemon=True).start()

    def _serve(self, conn, addr):
        with conn, conn.makefile("rw", encoding="utf-8", newline="\n") as stream:
            for line in stream:
                line = line.strip()
                if not line:
                    continue
                try:
                    request = json.loads(line)
                    if not isinstance(request, dict):
                        raise ValueError("a command must be a JSON object")
                except ValueError as exc:
                    reply = {"ok": False, "error": "bad JSON: %s" % exc}
                else:
                    pending = _Pending(request)
                    self.submit(pending)
                    # Bounded so a paused or stopped simulation cannot wedge a
                    # client forever waiting for a step that will not come.
                    reply = ({"ok": False, "error": "simulation did not step"}
                             if not pending.done.wait(timeout=5.0)
                             else pending.reply)
                try:
                    stream.write(json.dumps(reply) + "\n")
                    stream.flush()
                except (BrokenPipeError, ConnectionResetError, ValueError):
                    return

    def stop(self):
        self._stop.set()
        if self._socket is not None:
            try:
                self._socket.close()
            except OSError:
                pass


class HumanWalker(Supervisor):
    """Drives one Pedestrian node from high-level commands."""

    def __init__(self, options):
        super().__init__()
        self.options = options
        self.time_step = options.step or int(self.getBasicTimeStep())
        self.dt = self.time_step / 1000.0

        self.node = self.getSelf()
        self._check_articulated()

        translation = self.node.getField("translation")
        rotation = self.node.getField("rotation")
        self.translation_field = translation
        self.rotation_field = rotation

        start = translation.getSFVec3f()
        axis_angle = rotation.getSFRotation()
        # Yaw only: the PROTO is always upright, and a Pedestrian tipped over in
        # a world file would be a modelling error rather than a state to honour.
        self.x, self.y = start[0], start[1]
        self.theta = wrap_angle(
            axis_angle[3] * (1.0 if axis_angle[2] >= 0.0 else -1.0))
        self.root_height = (options.root_height if options.root_height is not None
                            else start[2])

        self.joint_fields = {name: self.node.getField(name) for name in JOINT_NAMES}
        missing = [n for n, f in self.joint_fields.items() if f is None]
        if missing:
            raise RuntimeError("Pedestrian node is missing joint fields: %s"
                               % ", ".join(missing))

        self.cycle = WalkCycle()
        self.v = 0.0
        self.w = 0.0
        self.commanded_v = 0.0
        self.commanded_w = 0.0
        # NOT `self.mode`: Supervisor already defines a getter-only `mode` property, and assigning
        # over it raises "property 'mode' of 'HumanWalker' object has no setter" the moment a real
        # Webots runs this. The wire protocol still calls the field "mode".
        self.walk_mode = MODE_VELOCITY
        self.goal = None            # (x, y, tolerance) for goto
        self.route = None           # {"points": [...], "index": int, "loop": bool}
        self.cruise = options.speed
        self.face_target = None     # heading in radians
        self.rest_blend = 1.0       # 1.0 = neutral stance, 0.0 = full gait
        self.last_event = "initialised"

        # Per arm: None, or {"angles": (q1,q2,q3)} / {"target": (x,y,z), ...}
        self.arm_hold = {"left": None, "right": None}
        self.arm_blend = {"left": 0.0, "right": 0.0}
        self.arm_info = {"left": None, "right": None}
        self.head_hold = None

        self._queue = []
        self._resolved = []
        self._last_held = {}
        self._queue_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._state = {}
        self.server = None

    # -- startup ------------------------------------------------------------
    def _check_articulated(self):
        """Fail loudly on the PROTO's `rigid` mode instead of posing a statue."""
        if self.node is None:
            raise RuntimeError("human_walker must run on the Pedestrian node itself")
        if self.node.getFromProtoDef("LEFT_ARM") is None:
            raise RuntimeError(
                "this Pedestrian has no joints: its `controllerArgs` field is "
                "empty, so the PROTO built it in rigid mode (every HingeJoint "
                "replaced by a fixed Solid). Put at least one argument in "
                "controllerArgs, e.g. [ \"--port=4321\" ], and reload the world.")

    def start_server(self):
        if self.options.no_server:
            return
        self.server = CommandServer(self.options.host, self.options.port,
                                    self._submit)
        self.server.start()

    def _submit(self, pending):
        with self._queue_lock:
            self._queue.append(pending)

    # -- command handling ---------------------------------------------------
    def _drain(self):
        with self._queue_lock:
            pending, self._queue = self._queue, []
        for item in pending:
            self._resolved.append((item, self._apply(item.request)))

    def _flush_replies(self):
        """Answer this step's commands now that the step has been applied."""
        for item, reply in self._resolved:
            if reply.pop("_state", False):
                with self._state_lock:
                    snapshot = dict(self._state)
                snapshot["cmd"] = reply.get("cmd")
                reply = snapshot
            item.resolve(reply)
        self._resolved = []

    def _number(self, request, key, default=None):
        value = request.get(key, default)
        if value is None:
            raise ValueError("missing numeric field '%s'" % key)
        return float(value)

    def _apply(self, request):
        command = str(request.get("cmd", "")).lower()
        try:
            handler = getattr(self, "_cmd_" + command, None)
            if handler is None:
                raise ValueError("unknown command %r" % command)
            reply = handler(request) or {}
        except Exception as exc:  # a bad command must not kill the human
            return {"ok": False, "cmd": command,
                    "error": "%s: %s" % (type(exc).__name__, exc)}
        reply.setdefault("ok", True)
        reply.setdefault("cmd", command)
        return reply

    def _set_velocity(self, v, w):
        self.commanded_v = clamp(v, -self.options.max_speed, self.options.max_speed)
        self.commanded_w = clamp(w, -self.options.max_yaw_rate,
                                 self.options.max_yaw_rate)
        self.walk_mode = MODE_VELOCITY
        self.goal = self.route = self.face_target = None

    def _cmd_walk(self, request):
        self._set_velocity(self._number(request, "v"), self.commanded_w)
        self.last_event = "walk"
        return {"v": self.commanded_v, "w": self.commanded_w}

    def _cmd_turn(self, request):
        self._set_velocity(self.commanded_v, self._number(request, "w"))
        self.last_event = "turn"
        return {"v": self.commanded_v, "w": self.commanded_w}

    def _cmd_velocity(self, request):
        self._set_velocity(self._number(request, "v", 0.0),
                           self._number(request, "w", 0.0))
        self.last_event = "velocity"
        return {"v": self.commanded_v, "w": self.commanded_w}

    def _cmd_stop(self, request):
        self._set_velocity(0.0, 0.0)
        self.last_event = "stop"
        return {"v": 0.0, "w": 0.0}

    def _cmd_goto(self, request):
        target = (self._number(request, "x"), self._number(request, "y"))
        tolerance = self._number(request, "tol", 0.15)
        self.cruise = clamp(self._number(request, "speed", self.options.speed),
                            0.0, self.options.max_speed)
        self.walk_mode = MODE_GOTO
        self.goal = (target[0], target[1], tolerance)
        self.route = self.face_target = None
        self.last_event = "goto"
        return {"target": list(target), "tol": tolerance, "speed": self.cruise,
                "distance": math.hypot(target[0] - self.x, target[1] - self.y)}

    def _cmd_follow(self, request):
        points = request.get("points")
        if not isinstance(points, list) or len(points) < 1:
            raise ValueError("'points' must be a list of [x, y] pairs")
        route = [(float(p[0]), float(p[1])) for p in points]
        self.cruise = clamp(self._number(request, "speed", self.options.speed),
                            0.0, self.options.max_speed)
        self.walk_mode = MODE_FOLLOW
        self.route = {"points": route, "index": 0,
                      "loop": bool(request.get("loop", False)),
                      "tol": self._number(request, "tol", 0.25)}
        self.goal = self.face_target = None
        self.last_event = "follow"
        return {"points": len(route), "loop": self.route["loop"],
                "speed": self.cruise}

    def _cmd_face(self, request):
        if "theta" in request:
            heading = self._number(request, "theta")
        else:
            heading = math.atan2(self._number(request, "y") - self.y,
                                 self._number(request, "x") - self.x)
        self.walk_mode = MODE_FACE
        self.face_target = wrap_angle(heading)
        self.goal = self.route = None
        self.last_event = "face"
        return {"theta": self.face_target,
                "error": wrap_angle(self.face_target - self.theta)}

    def _cmd_teleport(self, request):
        self.x = self._number(request, "x", self.x)
        self.y = self._number(request, "y", self.y)
        if "theta" in request:
            self.theta = wrap_angle(self._number(request, "theta"))
        self._set_velocity(0.0, 0.0)
        self.last_event = "teleport"
        return {"pose": [self.x, self.y, self.theta]}

    def _cmd_reach(self, request):
        side = str(request.get("arm", "right")).lower()
        if side not in ARMS:
            raise ValueError("'arm' must be 'left' or 'right'")
        target = (self._number(request, "x"), self._number(request, "y"),
                  self._number(request, "z"))
        frame = str(request.get("frame", "world")).lower()
        if frame not in ("world", "body"):
            raise ValueError("'frame' must be 'world' or 'body'")
        # A world target is a place: walk away and the hand comes off it, which
        # the residual then reports. A body target rides with the human, which
        # is what you want for carrying something or holding a gesture.
        turn_body = bool(request.get("turn_body", True)) and frame == "world"
        wrist = self._number(request, "wrist", 0.0)
        self.arm_hold[side] = {"target": target, "wrist": wrist, "frame": frame}

        arm = ARMS[side]
        reply = {"arm": side, "target": list(target), "frame": frame,
                 "turn_body": turn_body}
        yaw_solution = (solve_body_yaw_for_target((self.x, self.y), target[:2],
                                                  arm.plane_y)
                        if frame == "world" else None)
        if frame == "body":
            reply["yaw"] = None
        elif yaw_solution is None:
            # The target sits inside the cylinder the shoulder sweeps: no body
            # yaw can bring it into the arm plane, the human has to step away.
            reply["error"] = ("target is closer to the body axis than the arm "
                              "plane offset (%.3f m); step back first"
                              % abs(arm.plane_y))
            reply["yaw"] = None
        else:
            yaw, forward_distance = yaw_solution
            reply["yaw"] = yaw
            reply["yaw_error"] = wrap_angle(yaw - self.theta)
            reply["stand_distance"] = forward_distance
            if turn_body:
                self.walk_mode = MODE_FACE
                self.face_target = yaw
                self.goal = self.route = None

        # Report the reach as it stands right now, from the current body pose.
        # After a turn_body the verdict improves as the yaw converges; query
        # `state` once the human has stopped turning for the settled answer.
        angles, info = self._solve_arm(side, self.arm_hold[side])
        reply.update({
            "angles": list(angles),
            "residual": info["residual"],
            "out_of_plane": info["out_of_plane"],
            "in_plane": info["in_plane"],
            "reachable": self._is_reached(info),
            "clamped_reach": info["clamped_reach"],
            "clamped_limits": info["clamped_limits"],
        })
        self.last_event = "reach:" + side
        return reply

    def _cmd_arm(self, request):
        side = str(request.get("arm", "right")).lower()
        if side not in ARMS:
            raise ValueError("'arm' must be 'left' or 'right'")
        angles = (self._number(request, "shoulder", 0.0),
                  self._number(request, "elbow", 0.0),
                  self._number(request, "wrist", 0.0))
        self.arm_hold[side] = {"angles": angles}
        self.last_event = "arm:" + side
        return {"arm": side, "angles": list(angles)}

    def _is_reached(self, info):
        """Whether the palm is on the target, judged in metres, not exactly.

        The arm is confined to a sagittal plane and the body only yaws to a
        finite tolerance, so demanding a zero residual would call every real
        reach a failure. `--reach-tolerance` is what "on the target" means.
        """
        return info is not None and info["residual"] <= self.options.reach_tolerance

    def _cmd_relax(self, request):
        side = str(request.get("arm", "both")).lower()
        sides = list(ARMS) if side == "both" else [side]
        for one in sides:
            if one not in ARMS:
                raise ValueError("'arm' must be 'left', 'right' or 'both'")
            self.arm_hold[one] = None
        self.last_event = "relax"
        return {"arms": sides}

    def _cmd_head(self, request):
        if request.get("relax"):
            self.head_hold = None
            return {"head": None}
        self.head_hold = self._number(request, "angle")
        return {"head": self.head_hold}

    def _cmd_state(self, request):
        # Filled in at reply time from the post-step snapshot.
        return {"_state": True}

    _cmd_pose = _cmd_state

    def _cmd_ping(self, request):
        return {"time": self.getTime()}

    # -- motion -------------------------------------------------------------
    def _steer_towards(self, target_xy, cruise):
        """Unicycle law: turn towards the point, creep forward as it lines up."""
        dx, dy = target_xy[0] - self.x, target_xy[1] - self.y
        distance = math.hypot(dx, dy)
        error = wrap_angle(math.atan2(dy, dx) - self.theta)
        w = clamp(K_YAW * error, -self.options.max_yaw_rate,
                  self.options.max_yaw_rate)
        # cos(error) rather than a heading gate: the forward drive fades out
        # continuously as the target goes off to the side, and reverses behind.
        v = cruise * math.cos(error) * min(1.0, distance / APPROACH_DISTANCE)
        return clamp(v, -self.options.max_speed, self.options.max_speed), w, distance

    def _update_motion(self):
        if self.walk_mode == MODE_VELOCITY:
            self.v, self.w = self.commanded_v, self.commanded_w
        elif self.walk_mode == MODE_GOTO:
            v, w, distance = self._steer_towards(self.goal[:2], self.cruise)
            if distance <= self.goal[2]:
                self.v = self.w = 0.0
                self.walk_mode, self.goal = MODE_VELOCITY, None
                self.commanded_v = self.commanded_w = 0.0
                self.last_event = "goto:arrived"
            else:
                self.v, self.w = v, w
        elif self.walk_mode == MODE_FOLLOW:
            route = self.route
            v, w, distance = self._steer_towards(
                route["points"][route["index"]], self.cruise)
            if distance <= route["tol"]:
                route["index"] += 1
                if route["index"] >= len(route["points"]):
                    if route["loop"]:
                        route["index"] = 0
                    else:
                        self.v = self.w = 0.0
                        self.walk_mode, self.route = MODE_VELOCITY, None
                        self.commanded_v = self.commanded_w = 0.0
                        self.last_event = "follow:finished"
                        return
                v, w, _ = self._steer_towards(
                    route["points"][route["index"]], self.cruise)
            self.v, self.w = v, w
        elif self.walk_mode == MODE_FACE:
            error = wrap_angle(self.face_target - self.theta)
            if abs(error) < FACE_TOLERANCE:
                self.v = self.w = 0.0
                self.walk_mode, self.face_target = MODE_VELOCITY, None
                self.commanded_v = self.commanded_w = 0.0
                self.last_event = "face:aligned"
            else:
                self.v = 0.0
                self.w = clamp(K_YAW * error, -self.options.max_yaw_rate,
                               self.options.max_yaw_rate)

        self.x += self.v * math.cos(self.theta) * self.dt
        self.y += self.v * math.sin(self.theta) * self.dt
        self.theta = wrap_angle(self.theta + self.w * self.dt)

    # -- posing -------------------------------------------------------------
    def _solve_arm(self, side, hold):
        """Angles for a held arm, re-solved every step so it tracks the body."""
        if "angles" in hold:
            return hold["angles"], {
                "residual": 0.0, "out_of_plane": 0.0, "in_plane": 0.0,
                "reachable": True, "clamped_reach": False,
                "clamped_limits": False, "achieved": None}
        if hold.get("frame") == "body":
            local = hold["target"]
        else:
            root = (self.x, self.y, self.root_height)
            local = world_to_root(hold["target"], root, self.theta)
        return ARMS[side].inverse(local, q3=hold.get("wrist", 0.0))

    def _pose(self):
        moving = abs(self.v) > 1e-4 or abs(self.w) > 1e-4
        rate = self.dt / STOP_BLEND_TIME
        self.rest_blend = clamp(self.rest_blend + (-rate if moving else rate),
                                0.0, 1.0)
        self.cycle.advance(self.v, self.w, self.dt)
        angles, height = self.cycle.sample(rest_blend=self.rest_blend)
        pose = dict(zip(JOINT_NAMES, angles))

        rate = self.dt / ARM_BLEND_TIME
        for side, arm in ARMS.items():
            hold = self.arm_hold[side]
            target_blend = 1.0 if hold is not None else 0.0
            step = math.copysign(rate, target_blend - self.arm_blend[side])
            if abs(target_blend - self.arm_blend[side]) <= abs(step):
                self.arm_blend[side] = target_blend
            else:
                self.arm_blend[side] += step
            if hold is None:
                if self.arm_blend[side] <= 0.0:
                    self.arm_info[side] = None
                    continue
                held, info = self._last_held.get(side, ((0.0, 0.0, 0.0), None))
            else:
                held, info = self._solve_arm(side, hold)
                self._last_held[side] = (held, info)
            self.arm_info[side] = info
            blend = self.arm_blend[side]
            for name, value in zip(arm.joint_names, held):
                pose[name] = (1.0 - blend) * pose[name] + blend * value

        if self.head_hold is not None:
            pose["headAngle"] = self.head_hold

        for name, value in pose.items():
            self.joint_fields[name].setSFFloat(value)
        self.translation_field.setSFVec3f(
            [self.x, self.y, self.root_height + height])
        self.rotation_field.setSFRotation([0.0, 0.0, 1.0, self.theta])
        return pose

    def _publish_state(self, pose):
        root = (self.x, self.y, self.root_height)
        hands = {}
        for side, arm in ARMS.items():
            q = [pose[name] for name in arm.joint_names]
            hands[side] = list(root_to_world(arm.hand_position(*q), root,
                                             self.theta))
        state = {
            "ok": True,
            "time": self.getTime(),
            "pose": [self.x, self.y, self.theta],
            "velocity": [self.v, self.w],
            "mode": self.walk_mode,
            "event": self.last_event,
            "gait_phase": self.cycle.phase % WalkCycle.SEQUENCES,
            "rest_blend": self.rest_blend,
            "hands": hands,
            "arms": {side: (None if self.arm_hold[side] is None else {
                "blend": self.arm_blend[side],
                "residual": (self.arm_info[side] or {}).get("residual"),
                "reachable": self._is_reached(self.arm_info[side]),
                "frame": self.arm_hold[side].get("frame"),
                "target": self.arm_hold[side].get("target"),
            }) for side in ARMS},
            "goal": (list(self.goal) if self.goal else None),
            "route_index": (self.route["index"] if self.route else None),
        }
        with self._state_lock:
            self._state = state

    # -- main loop ----------------------------------------------------------
    def run(self):
        if self.options.trajectory:
            self._apply(self._trajectory_command(self.options.trajectory))
        # Publish a state before the first command can ask for one.
        self._publish_state(self._pose())
        while self.step(self.time_step) != -1:
            self._drain()
            self._update_motion()
            self._publish_state(self._pose())
            self._flush_replies()
        if self.server is not None:
            self.server.stop()

    def _trajectory_command(self, text):
        """Back-compatibility with the stock controller's --trajectory syntax."""
        points = []
        for chunk in text.split(","):
            parts = chunk.split()
            if len(parts) != 2:
                raise ValueError("--trajectory wants 'x1 y1, x2 y2, ...'")
            points.append([float(parts[0]), float(parts[1])])
        if len(points) < 2:
            raise ValueError("--trajectory needs at least two points")
        return {"cmd": "follow", "points": points, "loop": True,
                "speed": self.options.speed}


def parse_arguments(argv):
    parser = argparse.ArgumentParser(
        prog="human_walker",
        description="High-level command controller for the Webots Pedestrian.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=4321)
    parser.add_argument("--no-server", action="store_true",
                        help="run without the TCP command server")
    parser.add_argument("--speed", type=float, default=1.0,
                        help="default cruise speed for goto/follow, m/s")
    parser.add_argument("--max-speed", type=float, default=1.8, help="m/s")
    parser.add_argument("--max-yaw-rate", type=float, default=1.5, help="rad/s")
    parser.add_argument("--reach-tolerance", type=float, default=0.005,
                        help="m; how close the palm must be to count as reached")
    parser.add_argument("--step", type=int, default=None,
                        help="control period in ms (default: world basic step)")
    parser.add_argument("--root-height", type=float, default=None,
                        help="hip height in m (default: the world's own z)")
    parser.add_argument("--trajectory", default="",
                        help="'x1 y1, x2 y2, ...' looped, as in the stock "
                             "Cyberbotics controller")
    # Webots passes the controller name through; ignore anything unexpected so
    # a stray argument cannot stop the human from starting.
    options, unknown = parser.parse_known_args(argv)
    if unknown:
        print("[human_walker] ignoring unknown arguments: %s" % " ".join(unknown),
              flush=True)
    return options


def main():
    options = parse_arguments(sys.argv[1:])
    walker = HumanWalker(options)
    walker.start_server()
    print("[human_walker] '%s' ready at (%.2f, %.2f, %.0f deg), hip height %.3f m"
          % (walker.node.getField("name").getSFString(), walker.x, walker.y,
             math.degrees(walker.theta), walker.root_height), flush=True)
    walker.run()


if __name__ == "__main__":
    main()
