#!/usr/bin/env python3
"""Headless integration tests for `human_walker`.

Webots is never started.  A stand-in `controller` module supplies a Supervisor
and a Pedestrian node whose fields are plain Python values, which is enough to
exercise everything the controller actually decides: the gait clock, the pose
integration, the behaviour modes, the arm IK hold, and the TCP protocol
end to end through the real `human_client`.

Run:  python3 test_walker.py
"""

import json
import locale
import math
import socket
import sys
import threading

locale.setlocale(locale.LC_ALL, "")  # match the agents' es_ES environment

from webots_stub import (  # noqa: E402
    BASIC_TIME_STEP, Node as _Node, READ_ONLY_PROPERTIES, Supervisor as _Supervisor,
    install as install_stub,
)

install_stub()

import human_walker  # noqa: E402
from human_client import HumanClient  # noqa: E402
from pedestrian_kinematics import ARMS  # noqa: E402

failures = []


def check(name, ok, detail=""):
    print(("  PASS  " if ok else "  FAIL  ") + name + ("   " + detail if detail else ""))
    if not ok:
        failures.append(name)


def make_walker(argv=("--no-server",), translation=(0.0, 0.0, 1.27),
                rotation=(0.0, 0.0, 1.0, 0.0), articulated=True):
    _Supervisor.node = _Node(translation, rotation, "human", articulated)
    _Supervisor.step_budget = 10 ** 9
    options = human_walker.parse_arguments(list(argv))
    return human_walker.HumanWalker(options)


def tick(walker, count=1):
    """One control period, mirroring the body of `HumanWalker.run`."""
    for _ in range(count):
        walker.step(walker.time_step)
        walker._drain()
        walker._update_motion()
        walker._publish_state(walker._pose())
        walker._flush_replies()


def steps_for(seconds):
    return int(round(seconds * 1000.0 / BASIC_TIME_STEP))


print("startup")
try:
    make_walker(articulated=False)
    check("a rigid Pedestrian is refused", False, "no error raised")
except RuntimeError as exc:
    check("a rigid Pedestrian is refused with an actionable message",
          "controllerArgs" in str(exc))

walker = make_walker(translation=(1.0, -2.0, 1.23), rotation=(0, 0, 1, math.pi))
check("initial pose is taken from the world",
      abs(walker.x - 1.0) < 1e-12 and abs(walker.y + 2.0) < 1e-12
      and abs(abs(walker.theta) - math.pi) < 1e-9)
check("hip height is taken from the world's own z", abs(walker.root_height - 1.23) < 1e-12,
      "%.3f m" % walker.root_height)

print("\nwalking")
walker = make_walker()
walker._apply({"cmd": "walk", "v": 1.0})
tick(walker, steps_for(2.0))
check("two seconds at 1 m/s covers two metres", abs(walker.x - 2.0) < 0.05,
      "x=%.3f m" % walker.x)
check("it does not drift sideways", abs(walker.y) < 1e-9)
check("the legs are actually swinging",
      abs(walker.node.getField("leftLegAngle").value) > 0.05,
      "leftLegAngle=%.3f rad" % walker.node.getField("leftLegAngle").value)
z_walking = walker.node.getField("translation").value[2]
check("the hip bobs around the nominal height", abs(z_walking - 1.27) < 0.09,
      "z=%.3f m" % z_walking)

phase_walking = walker.cycle.phase
walker._apply({"cmd": "stop"})
tick(walker, steps_for(1.5))
check("stopping freezes the gait clock", abs(walker.cycle.phase - phase_walking) < 1e-9)
check("a stopped human settles into a neutral stance",
      max(abs(walker.node.getField(n).value)
          for n in ("leftLegAngle", "rightLegAngle", "leftArmAngle")) < 1e-6)
x_stopped = walker.x
tick(walker, steps_for(1.0))
check("a stopped human stays put", abs(walker.x - x_stopped) < 1e-12)

walker._apply({"cmd": "walk", "v": -0.5})
tick(walker, steps_for(1.0))
check("walking backwards moves backwards", walker.x < x_stopped - 0.4,
      "x=%.3f m" % walker.x)
check("walking backwards rewinds the gait", walker.cycle.phase < phase_walking)

print("\nturning")
walker = make_walker()
walker._apply({"cmd": "turn", "w": 0.5})
tick(walker, steps_for(2.0))
check("two seconds at 0.5 rad/s turns one radian", abs(walker.theta - 1.0) < 0.02,
      "theta=%.3f rad" % walker.theta)
check("a pivot still shuffles the feet", walker.cycle.phase > 0.0,
      "phase=%.3f" % walker.cycle.phase)
check("the rotation field carries the yaw about z",
      walker.node.getField("rotation").value[:3] == [0.0, 0.0, 1.0]
      and abs(walker.node.getField("rotation").value[3] - walker.theta) < 1e-12)

print("\nspeed and yaw limits are respected")
walker = make_walker(("--no-server", "--max-speed=1.2", "--max-yaw-rate=0.8"))
reply = walker._apply({"cmd": "velocity", "v": 9.0, "w": -9.0})
check("commanded speed is clamped, and the clamp is reported",
      abs(reply["v"] - 1.2) < 1e-12 and abs(reply["w"] + 0.8) < 1e-12,
      "v=%.2f w=%.2f" % (reply["v"], reply["w"]))

print("\ngoto")
walker = make_walker()
reply = walker._apply({"cmd": "goto", "x": 3.0, "y": 2.0, "tol": 0.15})
check("goto reports the distance it has to cover",
      abs(reply["distance"] - math.hypot(3.0, 2.0)) < 1e-12)
tick(walker, steps_for(20.0))
check("goto arrives inside its tolerance",
      math.hypot(walker.x - 3.0, walker.y - 2.0) <= 0.15,
      "miss %.3f m" % math.hypot(walker.x - 3.0, walker.y - 2.0))
check("goto hands control back and stops", walker.walk_mode == human_walker.MODE_VELOCITY
      and walker.v == 0.0 and walker.last_event == "goto:arrived")

print("\nfollow")
walker = make_walker()
walker._apply({"cmd": "follow",
               "points": [[2.0, 0.0], [2.0, 2.0], [0.0, 2.0]], "tol": 0.2})
tick(walker, steps_for(30.0))
check("an open route finishes", walker.last_event == "follow:finished",
      "event=%s" % walker.last_event)
check("it finishes at the last waypoint",
      math.hypot(walker.x - 0.0, walker.y - 2.0) < 0.3,
      "at (%.2f, %.2f)" % (walker.x, walker.y))

walker = make_walker()
walker._apply({"cmd": "follow", "points": [[2.0, 0.0], [0.0, 0.0]], "loop": True})
tick(walker, steps_for(30.0))
check("a looped route keeps going", walker.walk_mode == human_walker.MODE_FOLLOW)

print("\nreach: hand to a world point")
walker = make_walker()
target = (0.0, -0.45, 1.15)
reply = walker._apply({"cmd": "reach", "x": target[0], "y": target[1],
                       "z": target[2], "arm": "right", "turn_body": True})
check("reach returns the body yaw that makes the target reachable",
      reply["yaw"] is not None, "yaw=%.3f rad" % (reply["yaw"] or 0.0))
tick(walker, steps_for(4.0))
state = walker._state
palm = state["hands"]["right"]
error = math.dist(palm, target)
check("the palm converges onto the target", error < 0.01,
      "miss %.4f m at (%.3f, %.3f, %.3f)" % (error, palm[0], palm[1], palm[2]))
check("the state reports the reach as reachable",
      state["arms"]["right"]["reachable"] is True)
check("the held arm is fully faded in", abs(state["arms"]["right"]["blend"] - 1.0) < 1e-9)
check("the other arm is untouched by the reach", state["arms"]["left"] is None)

walker._apply({"cmd": "walk", "v": 0.8})
tick(walker, steps_for(2.0))
after = walker._state["arms"]["right"]
check("walking away from a world target breaks the reach, and says so",
      after["reachable"] is False and after["residual"] > 0.5,
      "residual %.3f m" % after["residual"])
check("the free arm still swings with the gait",
      abs(walker.node.getField("leftArmAngle").value) > 0.05)

walker._apply({"cmd": "relax", "arm": "right"})
tick(walker, steps_for(1.0))
check("relax gives the arm back to the gait",
      walker.arm_blend["right"] == 0.0 and walker._state["arms"]["right"] is None)

print("\nreach: a body-frame target rides with the human")
walker = make_walker()
body_target = (0.35, -0.26, -0.15)  # 35 cm in front, at the right hand's plane
reply = walker._apply({"cmd": "reach", "x": body_target[0], "y": body_target[1],
                       "z": body_target[2], "arm": "right", "frame": "body"})
check("a body-frame reach needs no body yaw", reply["yaw"] is None
      and reply["turn_body"] is False)
tick(walker, steps_for(1.0))
check("the palm lands on the body-frame point",
      walker._state["arms"]["right"]["reachable"] is True,
      "residual %.4f m" % walker._state["arms"]["right"]["residual"])
held_elbow = walker.node.getField("rightLowerArmAngle").value
walker._apply({"cmd": "velocity", "v": 0.8, "w": 0.4})
tick(walker, steps_for(3.0))
check("the pose survives walking and turning",
      abs(walker.node.getField("rightLowerArmAngle").value - held_elbow) < 1e-9,
      "elbow %.3f -> %.3f" % (held_elbow,
                              walker.node.getField("rightLowerArmAngle").value))
check("and it is still reported as reached",
      walker._state["arms"]["right"]["reachable"] is True)
hand = walker._state["hands"]["right"]
check("the palm has travelled with the body",
      math.dist(hand[:2], (walker.x, walker.y)) < 0.7 and walker.x > 1.0,
      "hand (%.2f, %.2f) body (%.2f, %.2f)" % (hand[0], hand[1], walker.x, walker.y))

print("\nreach: honest about what it cannot do")
walker = make_walker()
reply = walker._apply({"cmd": "reach", "x": 5.0, "y": 0.0, "z": 1.2,
                       "arm": "left", "turn_body": False})
check("an out-of-range target is not reported as reachable", reply["reachable"] is False)
check("an out-of-range target reports a metre-scale residual", reply["residual"] > 1.0,
      "residual %.2f m" % reply["residual"])
reply = walker._apply({"cmd": "reach", "x": 0.02, "y": 0.0, "z": 1.4,
                       "arm": "left", "turn_body": True})
check("a target on the body axis is refused with a reason",
      reply["yaw"] is None and "step back" in reply.get("error", ""),
      reply.get("error", ""))

print("\ndirect joint control and the head")
walker = make_walker()
walker._apply({"cmd": "arm", "arm": "left", "shoulder": -1.2, "elbow": -0.6})
tick(walker, steps_for(1.0))
check("a directly posed arm lands on the requested angles",
      abs(walker.node.getField("leftArmAngle").value + 1.2) < 1e-6
      and abs(walker.node.getField("leftLowerArmAngle").value + 0.6) < 1e-6)
walker._apply({"cmd": "head", "angle": 0.4})
tick(walker)
check("the head holds its nod", abs(walker.node.getField("headAngle").value - 0.4) < 1e-9)
walker._apply({"cmd": "head", "relax": True})
tick(walker, steps_for(0.5))
check("relaxing the head returns it to the gait", walker.head_hold is None)

print("\nteleport and bad input")
walker = make_walker()
walker._apply({"cmd": "teleport", "x": -4.0, "y": 1.5, "theta": 1.0})
tick(walker)
check("teleport moves and stops the human",
      abs(walker.x + 4.0) < 1e-12 and abs(walker.theta - 1.0) < 1e-12
      and walker.v == 0.0)
reply = walker._apply({"cmd": "fly"})
check("an unknown command is refused, not obeyed",
      reply["ok"] is False and "unknown command" in reply["error"])
reply = walker._apply({"cmd": "walk"})
check("a missing argument is refused with the field name",
      reply["ok"] is False and "'v'" in reply["error"], reply["error"])
reply = walker._apply({"cmd": "reach", "x": 1, "y": 1, "z": 1, "arm": "third"})
check("an invalid arm is refused", reply["ok"] is False)

print("\nback-compatibility with the stock --trajectory syntax")
walker = make_walker(("--trajectory=-1 -1, 1 -1, 1 1", "--speed=0.9", "--no-server"))
command = walker._trajectory_command(walker.options.trajectory)
check("--trajectory becomes a looped follow",
      command["cmd"] == "follow" and command["loop"] is True
      and len(command["points"]) == 3 and abs(command["speed"] - 0.9) < 1e-12)

print("\nTCP protocol, end to end through human_client")
probe = socket.socket()
probe.bind(("127.0.0.1", 0))
port = probe.getsockname()[1]
probe.close()

walker = make_walker(("--port=%d" % port,))
_Supervisor.step_budget = steps_for(120.0)
_Supervisor.step_delay = 0.001  # keep the sim from outrunning the client
walker.start_server()
runner = threading.Thread(target=walker.run, daemon=True)
runner.start()

try:
    with HumanClient(port=port, timeout=10.0) as human:
        check("ping answers", human.ping().get("ok") is True)
        reply = human.walk(1.0)
        check("walk is accepted over the wire", abs(reply["v"] - 1.0) < 1e-12)
        first = human.state()
        check("state carries a pose, a mode and both hands",
              "pose" in first and first["mode"] == "velocity"
              and set(first["hands"]) == {"left", "right"})
        walked = 0.0
        for _ in range(40):
            later = human.state()
            walked = later["pose"][0] - first["pose"][0]
            if walked > 0.3:
                break
        check("the human really moves while the client watches",
              0.3 < walked < 30.0, "advanced %.3f m" % walked)
        check("a state reply reflects the step that has just run",
              later["time"] > first["time"])
        human.stop()
        reply = human.reach(0.35, -0.26, -0.15, arm="right", frame="body")
        check("reach over the wire reports its residual",
              "residual" in reply and reply["residual"] < 0.01,
              "residual %.4f m" % reply["residual"])
        check("reach over the wire says the target is reached",
              reply["reachable"] is True)
        reply = human.send(cmd="nonsense")
        check("a bad command over the wire returns ok=false, connection intact",
              reply["ok"] is False)
        check("the connection survives a bad command",
              human.ping().get("ok") is True)
except Exception as exc:  # noqa: BLE001 - the test itself reports the failure
    check("TCP session completed without raising", False,
          "%s: %s" % (type(exc).__name__, exc))

_Supervisor.step_budget = 0
runner.join(timeout=10.0)
check("the controller exits when the simulation ends", not runner.is_alive())

# The real Supervisor defines getter-only properties; assigning over one raises
# "property '<name>' of 'HumanWalker' object has no setter" on the first real Webots step while a
# stub without them runs perfectly. `mode` cost exactly that, so name the failure here rather than
# leaving it as a confusing AttributeError from deep inside the stub.
print("\nthe stub reproduces the real Supervisor's getter-only properties")
missing = [name for name in READ_ONLY_PROPERTIES
           if not isinstance(getattr(_Supervisor, name, None), property)
           or getattr(_Supervisor, name).fset is not None]
check("every property the real Supervisor guards is guarded here too", not missing,
      ("NOT guarded: %s" % ", ".join(missing)) if missing
      else "%d: %s" % (len(READ_ONLY_PROPERTIES), ", ".join(READ_ONLY_PROPERTIES)))
# The point of the net: a controller assigning over one of these must fail HERE, the way it fails
# on the first step of a real Webots, instead of running perfectly under the stub and dying in the
# simulator. `self.mode = ...` cost exactly that.
try:
    walker.mode = "anything"
    check("assigning over one raises, as the real Supervisor does", False, "the assignment stuck")
except AttributeError as exc:
    check("assigning over one raises, as the real Supervisor does", True, str(exc))

print()
if failures:
    print("%d FAILED: %s" % (len(failures), ", ".join(failures)))
    sys.exit(1)
print("all checks passed")
