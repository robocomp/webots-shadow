#!/usr/bin/env python3
"""Offline checks for `pedestrian_kinematics`.  No Webots needed.

Run:  python3 test_kinematics.py
"""

import locale
import math
import sys

# The fleet runs under LANG=es_ES.UTF-8 and this file parses nothing, but a
# harness that silently differs from the agent's locale is exactly the trap
# CLAUDE.md warns about, so make the harness match the agent's environment.
locale.setlocale(locale.LC_ALL, "")

from pedestrian_kinematics import (  # noqa: E402
    ARMS, WalkCycle, solve_body_yaw_for_target, world_to_root, root_to_world,
    wrap_angle,
)

failures = []


def check(name, ok, detail=""):
    print(("  PASS  " if ok else "  FAIL  ") + name + ("   " + detail if detail else ""))
    if not ok:
        failures.append(name)


print("geometry")
for side, arm in ARMS.items():
    lo, hi = arm.reach_span()
    check("%-5s reach span plausible for a human arm" % side,
          0.0 <= lo < 0.10 and 0.55 < hi < 0.72, "span=[%.3f, %.3f] m" % (lo, hi))
    rest = arm.hand_position(0.0, 0.0, 0.0)
    check("%-5s rest palm matches the PROTO hand slot" % side,
          abs(rest[2] - (-0.45)) < 1e-9, "palm=(%.3f, %.3f, %.3f)" % rest)

print("\nround trip: IK then FK over the reachable annulus")
worst = 0.0
tested = 0
for side, arm in ARMS.items():
    lo, hi = arm.reach_span()
    for k in range(24):
        bearing = -math.pi + 2.0 * math.pi * k / 24.0
        for frac in (0.15, 0.35, 0.55, 0.75, 0.9, 0.99):
            radius = lo + frac * (hi - lo)
            target = (arm.shoulder.real + radius * math.cos(bearing),
                      arm.plane_y,
                      arm.shoulder.imag + radius * math.sin(bearing))
            angles, info = arm.inverse(target)
            if not info["reachable"]:
                continue  # joint limits legitimately exclude part of the annulus
            tested += 1
            worst = max(worst, info["residual"])
check("IK reproduces the requested palm position", worst < 1e-9,
      "%d targets, worst residual %.2e m" % (tested, worst))
check("a useful fraction of the annulus is inside the joint limits",
      tested > 150, "%d of 288 targets reachable" % tested)

print("\nunreachable targets are reported, not silently faked")
arm = ARMS["right"]
far = (arm.shoulder.real + 3.0, arm.plane_y, arm.shoulder.imag)
angles, info = arm.inverse(far)
check("over-extended target flags clamped_reach", info["clamped_reach"])
check("over-extended target reports a large residual", info["residual"] > 2.0,
      "residual %.3f m" % info["residual"])
off = (0.3, arm.plane_y + 0.6, -0.2)
angles, info = arm.inverse(off)
check("off-plane target reports out_of_plane", abs(info["out_of_plane"] - 0.6) < 1e-9,
      "out_of_plane %.3f m" % info["out_of_plane"])
check("off-plane target is not called reachable", not info["reachable"])

print("\nelbow flexes the anatomical way")
arm = ARMS["right"]
front = (0.35, arm.plane_y, 0.05)  # palm out in front at chest height
angles, info = arm.inverse(front)
check("reaching forward gives a negative elbow angle", angles[1] <= 0.0,
      "q=(%.3f, %.3f, %.3f)" % angles)
check("reaching forward is inside the joint limits", info["reachable"],
      "residual %.2e m" % info["residual"])

print("\nbody yaw brings an arbitrary world target into the arm plane")
worst_plane = 0.0
for side, arm in ARMS.items():
    for k in range(16):
        bearing = -math.pi + 2.0 * math.pi * k / 16.0
        for distance in (0.4, 1.0, 3.0):
            root = (1.5, -0.7, 1.27)
            target = (root[0] + distance * math.cos(bearing),
                      root[1] + distance * math.sin(bearing),
                      1.10)
            solved = solve_body_yaw_for_target(root[:2], target[:2], arm.plane_y)
            if solved is None:
                check("no-yaw-exists is only reported inside the shoulder circle",
                      distance < abs(arm.plane_y), "d=%.2f" % distance)
                continue
            yaw, _ = solved
            local = world_to_root(target, root, yaw)
            worst_plane = max(worst_plane, abs(local[1] - arm.plane_y))
check("yaw puts the target exactly in the arm plane", worst_plane < 1e-12,
      "worst plane error %.2e m" % worst_plane)

print("\nframe transforms invert each other")
worst_rt = 0.0
for yaw in (0.0, 0.7, -2.4, 3.0):
    for p in ((1.0, 2.0, 0.5), (-3.0, 0.25, 1.9)):
        root = (0.3, -1.2, 1.27)
        back = root_to_world(world_to_root(p, root, yaw), root, yaw)
        worst_rt = max(worst_rt, max(abs(a - b) for a, b in zip(p, back)))
check("world->root->world is the identity", worst_rt < 1e-12, "worst %.2e m" % worst_rt)

print("\ngait clock is driven by distance, not by the wall clock")
cycle = WalkCycle()
before = cycle.phase
cycle.advance(0.0, 0.0, 1.0)
check("standing still does not advance the gait", cycle.phase == before)
cycle.advance(1.0, 0.0, 0.22)
check("one CYCLE_TO_DISTANCE_RATIO of travel is one keyframe",
      abs(cycle.phase - (before + 1.0)) < 1e-12, "phase=%.6f" % cycle.phase)
cycle.advance(-1.0, 0.0, 0.22)
check("walking backwards rewinds the gait", abs(cycle.phase - before) < 1e-12)
cycle.advance(0.0, 1.0, 1.0)
check("pivoting on the spot still shuffles the feet", cycle.phase > before,
      "phase=%.4f" % cycle.phase)

angles, height = cycle.sample(rest_blend=1.0)
check("a fully relaxed pose is the neutral stance",
      max(abs(a) for a in angles) < 1e-12 and abs(height) < 1e-12)
angles, height = cycle.sample(rest_blend=0.0)
check("a walking pose is not neutral", max(abs(a) for a in angles) > 0.1)

print("\ngait table integrity")
check("13 joint rows", len(WalkCycle.ANGLES) == 13)
check("8 keyframes per row", all(len(r) == 8 for r in WalkCycle.ANGLES))
check("8 height offsets", len(WalkCycle.HEIGHT_OFFSETS) == 8)
check("wrap_angle folds onto (-pi, pi]",
      abs(wrap_angle(3.0 * math.pi / 2.0) + math.pi / 2.0) < 1e-12
      and abs(wrap_angle(0.5) - 0.5) < 1e-12)

# A negative phase happens the moment the pedestrian is asked to back up from
# a standing start, and Python's floor/% pair is what keeps the table lookup
# on the right keyframes there.
probe = WalkCycle()
probe.phase = -0.5
back_angles, _ = probe.sample()
probe.phase = 7.5
same_angles, _ = probe.sample()
check("a negative phase wraps onto the same keyframes as its positive twin",
      max(abs(a - b) for a, b in zip(back_angles, same_angles)) < 1e-12)

probe.phase = 7.999999
before_wrap, _ = probe.sample()
probe.phase = 8.000001
after_wrap, _ = probe.sample()
check("the gait is continuous across the cycle boundary",
      max(abs(a - b) for a, b in zip(before_wrap, after_wrap)) < 1e-4)

print()
if failures:
    print("%d FAILED: %s" % (len(failures), ", ".join(failures)))
    sys.exit(1)
print("all checks passed")
