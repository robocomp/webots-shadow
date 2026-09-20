"""Pure kinematics and gait model for the Webots `Pedestrian` PROTO.

This module deliberately imports nothing from Webots so that it can be unit
tested offline (see `test_kinematics.py`).  All geometry below is read off
`Pedestrian.proto` (R2025a) and is expressed in the PROTO root frame:

    +x forward, +y left, +z up, root origin at hip height (1.27 m by default).

Every joint of the Pedestrian is a HingeJoint whose axis is `0 1 0`, i.e. the
body-lateral axis.  Consequence, and it is the single most important fact about
this model: each arm is a *planar* 3-link chain confined to the sagittal plane
at its own constant y.  There is no shoulder abduction and no humerus twist, so
an arbitrary 3-D target is only reachable if the body first yaws to bring the
target into that plane.  `solve_body_yaw_for_target` does exactly that.

A rotation of angle q about +y maps a point p to R_y(q) p.  Writing a point of
the sagittal plane as the complex number c = x + i*z, that rotation is simply

    c' = c * exp(-i q)

which is what the forward/inverse kinematics below use throughout.
"""

import cmath
import math

# --- Pedestrian.proto geometry, root frame ----------------------------------
# Hinge anchors of the left chain; the right chain mirrors y.
_LEFT_SHOULDER = (-0.010, 0.280, 0.190)
_LEFT_ELBOW = (-0.040, 0.250, -0.090)
_LEFT_WRIST = (-0.035, 0.250, -0.370)
# `leftHandSlot` / `rightHandSlot` origin: the palm, i.e. what "the hand" means.
_LEFT_HAND = (0.000, 0.250, -0.450)

_RIGHT_SHOULDER = (-0.010, -0.280, 0.190)
_RIGHT_ELBOW = (-0.040, -0.250, -0.090)
_RIGHT_WRIST = (-0.035, -0.250, -0.370)
_RIGHT_HAND = (0.020, -0.260, -0.450)

HEAD_ANCHOR = (0.010, 0.0, 0.370)  # axis 0 1 0 -> headAngle is a nod, not a yaw

# Joint limits.  The PROTO declares none (the joints carry no motor and no
# stops), so these are anatomical bounds we impose ourselves to keep IK poses
# from folding through the body.  Elbow flexion is negative, see below.
SHOULDER_RANGE = (-3.00, 2.00)
ELBOW_RANGE = (-2.70, 0.10)
WRIST_RANGE = (-1.20, 1.20)

# The 13 hidden joint fields of the PROTO, in the order the gait table uses.
JOINT_NAMES = [
    "leftArmAngle", "leftLowerArmAngle", "leftHandAngle",
    "rightArmAngle", "rightLowerArmAngle", "rightHandAngle",
    "leftLegAngle", "leftLowerLegAngle", "leftFootAngle",
    "rightLegAngle", "rightLowerLegAngle", "rightFootAngle",
    "headAngle",
]

ARM_JOINTS = {
    "left": ("leftArmAngle", "leftLowerArmAngle", "leftHandAngle"),
    "right": ("rightArmAngle", "rightLowerArmAngle", "rightHandAngle"),
}


def _c(p):
    """Sagittal-plane complex coordinate x + i*z of a root-frame point."""
    return complex(p[0], p[2])


def clamp(value, low, high):
    return low if value < low else (high if value > high else value)


def wrap_angle(a):
    """Wrap to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


class ArmChain:
    """The 3-link planar arm of one side of the Pedestrian.

    Link 1 is shoulder->elbow, link 2 elbow->wrist, link 3 wrist->palm.  Joint
    angles are the PROTO's `*ArmAngle` / `*LowerArmAngle` / `*HandAngle` and are
    absolute hinge positions, composed down the chain.
    """

    def __init__(self, side):
        if side == "left":
            shoulder, elbow, wrist, hand = (
                _LEFT_SHOULDER, _LEFT_ELBOW, _LEFT_WRIST, _LEFT_HAND)
        elif side == "right":
            shoulder, elbow, wrist, hand = (
                _RIGHT_SHOULDER, _RIGHT_ELBOW, _RIGHT_WRIST, _RIGHT_HAND)
        else:
            raise ValueError("side must be 'left' or 'right', got %r" % (side,))

        self.side = side
        self.joint_names = ARM_JOINTS[side]
        # The plane the hand is confined to.  The shoulder sits marginally
        # outboard of it; the palm's y is what a reach target has to match.
        self.plane_y = hand[1]
        self.shoulder = _c(shoulder)
        self.shoulder_y = shoulder[1]
        self._l1 = _c(elbow) - _c(shoulder)
        self._l2 = _c(wrist) - _c(elbow)
        self._l3 = _c(hand) - _c(wrist)

    # -- forward kinematics --------------------------------------------------
    def forward(self, q1, q2, q3):
        """Root-frame (x, y, z) of elbow, wrist and palm for the given angles."""
        elbow = self.shoulder + self._l1 * cmath.exp(-1j * q1)
        wrist = elbow + self._l2 * cmath.exp(-1j * (q1 + q2))
        hand = wrist + self._l3 * cmath.exp(-1j * (q1 + q2 + q3))
        return (
            (elbow.real, self.plane_y, elbow.imag),
            (wrist.real, self.plane_y, wrist.imag),
            (hand.real, self.plane_y, hand.imag),
        )

    def hand_position(self, q1, q2, q3):
        return self.forward(q1, q2, q3)[2]

    def reach_span(self, q3=0.0):
        """(min, max) palm distance from the shoulder for a fixed wrist angle."""
        lumped = self._l2 + self._l3 * cmath.exp(-1j * q3)
        l1, lb = abs(self._l1), abs(lumped)
        return (abs(l1 - lb), l1 + lb)

    # -- inverse kinematics --------------------------------------------------
    def inverse(self, target, q3=0.0):
        """Place the palm on `target`, a root-frame (x, y, z).

        The wrist angle `q3` is held fixed, which makes forearm+hand a rigid
        segment and reduces the problem to exact 2-link planar IK.  Returns
        (angles, info) where `angles` is (q1, q2, q3) already clamped to the
        anatomical ranges and `info` reports how far the achieved palm ends up
        from the request, split into the two ways this arm can miss:

            out_of_plane  |target.y - plane_y|, irreducible without a body yaw
            in_plane      the residual left after clamping reach and limits
            residual      the true 3-D distance |achieved - target|
            reachable     True when nothing had to be clamped
        """
        q3 = clamp(q3, *WRIST_RANGE)
        lumped = self._l2 + self._l3 * cmath.exp(-1j * q3)
        l1, lb = abs(self._l1), abs(lumped)
        a, b = cmath.phase(self._l1), cmath.phase(lumped)

        r = complex(target[0], target[2]) - self.shoulder
        radius = abs(r)
        lo, hi = abs(l1 - lb), l1 + lb
        clamped = False
        if radius > hi:
            radius, clamped = hi, True
        elif radius < lo:
            radius, clamped = max(lo, 1e-9), True

        cos_beta = clamp((radius * radius - l1 * l1 - lb * lb) / (2.0 * l1 * lb),
                         -1.0, 1.0)
        # The positive branch is the anatomical one: it yields q2 <= 0, i.e.
        # elbow flexion swinging the palm forward, matching the walk cycle's
        # sign convention for `*LowerArmAngle`.
        beta = math.acos(cos_beta)
        theta1 = (cmath.phase(r) if radius > 1e-9 else a) - math.atan2(
            lb * math.sin(beta), l1 + lb * math.cos(beta))

        q1 = a - theta1
        q2 = b - a - beta
        q1c, q2c = clamp(q1, *SHOULDER_RANGE), clamp(q2, *ELBOW_RANGE)
        limited = (abs(q1c - q1) > 1e-9) or (abs(q2c - q2) > 1e-9)

        achieved = self.hand_position(q1c, q2c, q3)
        out_of_plane = abs(target[1] - self.plane_y)
        in_plane = math.hypot(achieved[0] - target[0], achieved[2] - target[2])
        residual = math.sqrt(in_plane * in_plane + out_of_plane * out_of_plane)
        info = {
            "out_of_plane": out_of_plane,
            "in_plane": in_plane,
            "residual": residual,
            "reachable": (not clamped) and (not limited) and out_of_plane < 1e-6,
            "clamped_reach": clamped,
            "clamped_limits": limited,
            "achieved": achieved,
        }
        return (q1c, q2c, q3), info


LEFT_ARM = ArmChain("left")
RIGHT_ARM = ArmChain("right")
ARMS = {"left": LEFT_ARM, "right": RIGHT_ARM}


def solve_body_yaw_for_target(root_xy, target_xy, plane_y):
    """Body yaw that brings `target_xy` into the arm plane at y = `plane_y`.

    The arm cannot leave its sagittal plane, so the only way to reach a target
    off to the side is to turn the torso until the plane contains it.  Returns
    (yaw, forward_distance) or None when the target is closer to the root axis
    than the plane offset itself, in which case no yaw exists (the target is
    inside the cylinder the shoulder sweeps) and the caller should step away.
    """
    dx = target_xy[0] - root_xy[0]
    dy = target_xy[1] - root_xy[1]
    distance = math.hypot(dx, dy)
    if distance < abs(plane_y):
        return None
    # Rotating the world into the body frame must send the target to y=plane_y.
    yaw = math.atan2(dy, dx) - math.asin(plane_y / distance)
    return wrap_angle(yaw), math.sqrt(max(distance * distance - plane_y * plane_y, 0.0))


def world_to_root(point, root_xyz, yaw):
    """Express a world point in the pedestrian root frame."""
    dx = point[0] - root_xyz[0]
    dy = point[1] - root_xyz[1]
    c, s = math.cos(yaw), math.sin(yaw)
    return (c * dx + s * dy, -s * dx + c * dy, point[2] - root_xyz[2])


def root_to_world(point, root_xyz, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return (root_xyz[0] + c * point[0] - s * point[1],
            root_xyz[1] + s * point[0] + c * point[1],
            root_xyz[2] + point[2])


# --- walk cycle -------------------------------------------------------------

class WalkCycle:
    """The stock 8-keyframe Pedestrian gait, re-parameterised by distance.

    The tables are Cyberbotics' empirical walk from `pedestrian.py`, unchanged.
    What changes is the clock: the original controller indexes them with
    absolute simulation time, which makes the gait impossible to stop, reverse
    or re-steer.  Here the phase is advanced by the distance actually travelled,
    so the legs stop when the body stops and step backwards when it reverses.
    """

    SEQUENCES = 8
    CYCLE_TO_DISTANCE_RATIO = 0.22  # metres of travel per keyframe

    HEIGHT_OFFSETS = [-0.02, 0.04, 0.08, -0.03, -0.02, 0.04, 0.08, -0.03]

    ANGLES = [
        [-0.52, -0.15, 0.58, 0.70, 0.52, 0.17, -0.36, -0.74],   # left arm
        [0.00, -0.16, -0.70, -0.38, -0.47, -0.30, -0.58, -0.21],  # left lower arm
        [0.12, 0.00, 0.12, 0.20, 0.00, -0.17, -0.25, 0.00],     # left hand
        [0.52, 0.17, -0.36, -0.74, -0.52, -0.15, 0.58, 0.70],   # right arm
        [-0.47, -0.30, -0.58, -0.21, 0.00, -0.16, -0.70, -0.38],  # right lower arm
        [0.00, -0.17, -0.25, 0.00, 0.12, 0.00, 0.12, 0.20],     # right hand
        [-0.55, -0.85, -1.14, -0.70, -0.56, 0.12, 0.24, 0.40],  # left leg
        [1.40, 1.58, 1.71, 0.49, 0.84, 0.00, 0.14, 0.26],       # left lower leg
        [0.07, 0.07, -0.07, -0.36, 0.00, 0.00, 0.32, -0.07],    # left foot
        [-0.56, 0.12, 0.24, 0.40, -0.55, -0.85, -1.14, -0.70],  # right leg
        [0.84, 0.00, 0.14, 0.26, 1.40, 1.58, 1.71, 0.49],       # right lower leg
        [0.00, 0.00, 0.42, -0.07, 0.07, 0.07, -0.07, -0.36],    # right foot
        [0.18, 0.09, 0.00, 0.09, 0.18, 0.09, 0.00, 0.09],       # head
    ]

    # Half the hip separation: a pivot on the spot still moves each foot along
    # an arc of this radius, which is what drives the shuffle-in-place gait.
    HIP_HALF_WIDTH = 0.17

    def __init__(self):
        self.phase = 0.0  # in keyframes, may be negative; wrapped on read

    def advance(self, forward_speed, yaw_rate, dt):
        """Step the gait clock by the ground distance the feet covered."""
        travelled = forward_speed * dt
        pivot = abs(yaw_rate) * self.HIP_HALF_WIDTH * dt
        # A pivot always advances the cycle forwards: both feet shuffle whether
        # you turn left or right, only the translation carries a sign.
        step = (travelled + math.copysign(pivot, travelled or 1.0)) \
            if abs(travelled) > 1e-9 else pivot
        self.phase += step / self.CYCLE_TO_DISTANCE_RATIO
        return self.phase

    def sample(self, rest_blend=0.0):
        """Interpolated joint angles and hip height offset at the current phase.

        `rest_blend` in [0, 1] fades the whole cycle towards the neutral stance,
        which is how a stopped pedestrian settles instead of freezing mid-stride.
        """
        base = math.floor(self.phase)
        ratio = self.phase - base
        i = int(base) % self.SEQUENCES
        j = (i + 1) % self.SEQUENCES
        keep = 1.0 - clamp(rest_blend, 0.0, 1.0)
        angles = [keep * (row[i] * (1.0 - ratio) + row[j] * ratio)
                  for row in self.ANGLES]
        height = keep * (self.HEIGHT_OFFSETS[i] * (1.0 - ratio)
                         + self.HEIGHT_OFFSETS[j] * ratio)
        return angles, height
