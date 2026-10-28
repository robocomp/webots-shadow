"""Webots supervisor for the evaluation recordings (docs_output/resultados/35_banco_de_verificacion, 36).

Once, when the robot (DEF shadow) first reaches x >= trigger_x in the world, it does one of:
  push     a constant force for duration_s on the robot (DEF shadow), at height_m above the floor on its
           axis, as a person's hand would push it, or on the bottle (DEF flacon), at its centre of mass;
           the force is in the world frame;
  occlude  a plate with no physics in front of the zed camera for duration_s: the perception loses the
           bottle with nothing physical happening to it.
What it did goes to a JSON log in experiments/ground_truth/supervisor/ of robocomp-insight, which
experiments/ground_truth.py joins to the recording that was running then.

The plan is the controller's first argument, a JSON object (controllerArgs in the world), e.g.
    {"mode": "push", "target": "shadow", "trigger_x": 0.0, "duration_s": 0.2, "force_n": [0, 150, 0]}
    {"mode": "push", "target": "flacon", "trigger_x": 0.0, "duration_s": 0.15, "force_n": [0, 3, 0]}
    {"mode": "occlude", "trigger_x": 0.0, "duration_s": 1.0}
    {"mode": "off"}                                   (does nothing: the bump runs)

Two fixes of 21/10/2026 (schema_version 2 of the log; docs_output/resultados/36, sections 5 and 6):
  * Webots R2025a's Python Node.addForceWithOffset calls wb_supervisor_node_add_force (the offset pointer
    lands on `relative`, which is then true): the pushes of 20/10 went in the robot's frame and at its centre
    of mass. The C function is called here directly.
  * The occluding plate stood 1 cm in front of the lens, on the camera's near clipping plane (Camera.near,
    0.01 m by default): its front face was clipped and its back face culled, so the camera never saw it. It
    now stands 5 cm in front, large enough to cover the zed's 110 x 78 degrees there.
"""
import ctypes
import json
import math
import os
import sys
import time
from pathlib import Path

from controller import Supervisor
from controller.wb import wb

ROBOT_DEF, BOTTLE_DEF = "shadow", "flacon"
ROBOT_ORIGIN_HEIGHT_M = 0.033   # the Shadow's origin above the floor (wheel radius - axle height)
# In the robot's frame (x right, y forward, z up; Shadow.proto) the zed camera is at (0, -0.075, 0.945), looking
# forward with 110 x 77.5 degrees of view (fieldOfView 1.9198, 1280 x 720), with near = 0.01 m. A plate
# OCCLUDER_DISTANCE_M in front of the lens must be at least 2 d tan(55 deg) wide and 2 d tan(38.8 deg) high:
# 14.3 x 8.0 cm at 5 cm. There it stays below the helios (at (0, -0.155, 1.075), hung upside down: its lowest
# beam goes 15.5 deg down, 1.039 m high 13 cm ahead of it, above the plate's top at 1.005) and above the
# bpearl (0.67 m, looking down). The Shadow PROTO exposes no children, so the plate stands in the world, with
# no boundingObject and no physics (it touches nothing), and follows the robot at every step.
ZED_IN_ROBOT_M = (0.0, -0.075, 0.945)
OCCLUDER_DISTANCE_M = 0.05
OCCLUDER_SIZE_M = (0.18, 0.002, 0.12)        # x (across), y (along the view), z (up), in the robot's frame
OCCLUDER_OFFSET_M = [ZED_IN_ROBOT_M[0], ZED_IN_ROBOT_M[1] + OCCLUDER_DISTANCE_M, ZED_IN_ROBOT_M[2]]
OCCLUDER = ('DEF PUSH_SUPERVISOR_OCCLUDER Pose { children [ Shape { '
            'appearance PBRAppearance { baseColor 0.1 0.1 0.1 roughness 1 metalness 0 } '
            'geometry Box { size %g %g %g } } ] }' % OCCLUDER_SIZE_M)
LOG_DIR = Path(os.environ.get("INSIGHT_SUPERVISOR_LOG_DIR") or
               Path(os.environ.get("ROBOCOMP", str(Path.home() / "robocomp"))) /
               "components/robocomp-insight/experiments/ground_truth/supervisor")
MECHANISM = {("push", ROBOT_DEF): "robot_push", ("push", BOTTLE_DEF): "bottle_push",
             ("occlude", None): "perception_failure"}
LOG_SCHEMA_VERSION = 2


def add_force_with_offset(node, force, offset, relative: bool = False) -> None:
    """wb_supervisor_node_add_force_with_offset: `force` (world frame, or the node's if relative) at `offset`
    in the node's frame. Not Node.addForceWithOffset, which calls the function without offset (see above)."""
    wb.wb_supervisor_node_add_force_with_offset(node._ref, (ctypes.c_double * 3)(*force),
                                                (ctypes.c_double * 3)(*offset), ctypes.c_int(1 if relative else 0))


def main() -> None:
    plan = json.loads(sys.argv[1]) if len(sys.argv) > 1 else {}
    mode = plan.get("mode", "push")
    target_def = plan.get("target", ROBOT_DEF) if mode == "push" else None
    trigger_x = float(plan.get("trigger_x", 0.0))
    duration_s = float(plan.get("duration_s", 0.2 if mode == "push" else 1.0))
    force = [float(v) for v in plan.get("force_n", [0.0, 150.0, 0.0])]
    height_m = float(plan.get("height_m", 1.0))

    supervisor = Supervisor()
    step_ms = int(supervisor.getBasicTimeStep())
    if mode == "off":
        while supervisor.step(step_ms) != -1:
            pass
        return
    robot = supervisor.getFromDef(ROBOT_DEF)
    target = supervisor.getFromDef(target_def) if target_def else None
    if robot is None or (mode == "push" and target is None) or mode not in ("push", "occlude"):
        sys.exit(f"push_supervisor: bad plan {plan} (or no DEF {ROBOT_DEF}/{target_def} in the world)")
    offset = [0.0, 0.0, height_m - ROBOT_ORIGIN_HEIGHT_M] if target_def == ROBOT_DEF else [0.0, 0.0, 0.0]

    def follow(node):
        """Put the plate in front of the camera, in the robot's frame (its origin is the robot's)."""
        position, rotation = robot.getPosition(), robot.getOrientation()   # rotation: 3x3, row-major
        world = [position[i] + sum(rotation[3 * i + j] * OCCLUDER_OFFSET_M[j] for j in range(3)) for i in range(3)]
        node.getField("translation").setSFVec3f(world)
        node.getField("rotation").setSFRotation(robot.getField("rotation").getSFRotation())

    fired_at, occluder = None, None
    while supervisor.step(step_ms) != -1:
        now = supervisor.getTime()
        if fired_at is None and robot.getPosition()[0] >= trigger_x:
            fired_at = now
            intervention = {"mechanism": MECHANISM[(mode, target_def)], "mode": mode, "target": target_def,
                            "duration_s": duration_s, "trigger_x_m": trigger_x}
            if mode == "push":
                intervention.update({"force_n": force, "force_frame": "world", "offset_in_target_frame_m": offset})
            else:
                intervention.update({"occluder_distance_m": OCCLUDER_DISTANCE_M, "occluder_size_m": list(OCCLUDER_SIZE_M)})
            log = {"schema_version": LOG_SCHEMA_VERSION, "intervention": intervention, "plan": plan,
                   "simulation_time_s": now, "basic_time_step_ms": step_ms,
                   "wall_time_local": time.strftime("%Y-%m-%dT%H:%M:%S"), "wall_time_unix": time.time(),
                   "robot_position_m": robot.getPosition(), "robot_orientation": robot.getOrientation()}
            LOG_DIR.mkdir(parents=True, exist_ok=True)
            path = LOG_DIR / f"supervisor_{time.strftime('%Y%m%d_%H%M%S')}_{int(time.time() * 1000) % 1000:03d}.json"
            path.write_text(json.dumps(log, indent=1))
            print(f"push_supervisor: {intervention['mechanism']} at t={now:.3f} s ({path})")
            if mode == "occlude":
                supervisor.getRoot().getField("children").importMFNodeFromString(-1, OCCLUDER)
                occluder = supervisor.getFromDef("PUSH_SUPERVISOR_OCCLUDER")
                follow(occluder)
        if fired_at is None:
            continue
        if now - fired_at < duration_s:
            if mode == "push":
                # Webots clears applied forces after every step: apply it again at each one.
                add_force_with_offset(target, force, offset, relative=False)
            elif occluder is not None:
                follow(occluder)
        elif occluder is not None:
            occluder.remove()
            occluder = None


if __name__ == "__main__":
    main()
