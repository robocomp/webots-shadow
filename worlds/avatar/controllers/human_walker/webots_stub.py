"""A stand-in for the Webots `controller` module.

It supplies just enough of `Supervisor` and of a `Pedestrian` node -- fields as plain Python
values -- to run `human_walker` with no simulator present. Used by `test_walker.py` and by the
webots-human-bridge tests, which drive the real controller over its real socket.

`Supervisor.step_budget` bounds the run; `Supervisor.step_delay` slows it to something like real
time so a client can interleave. Both are read off `Supervisor` itself rather than `type(self)`,
because the latter would create a shadowing copy on the subclass the first time it is written and
the value set here would never be seen again.
"""

import sys
import time
import types

BASIC_TIME_STEP = 32.0

JOINT_FIELDS = (
    "leftArmAngle", "leftLowerArmAngle", "leftHandAngle",
    "rightArmAngle", "rightLowerArmAngle", "rightHandAngle",
    "leftLegAngle", "leftLowerLegAngle", "leftFootAngle",
    "rightLegAngle", "rightLowerLegAngle", "rightFootAngle",
    "headAngle",
)


class Field:
    def __init__(self, value):
        self.value = value

    def getSFVec3f(self):
        return list(self.value)

    def getSFRotation(self):
        return list(self.value)

    def getSFString(self):
        return self.value

    def getSFFloat(self):
        return self.value

    def setSFVec3f(self, value):
        self.value = list(value)

    def setSFRotation(self, value):
        self.value = list(value)

    def setSFFloat(self, value):
        self.value = float(value)


class Node:
    def __init__(self, translation, rotation, name, articulated=True):
        self.fields = {
            "translation": Field(list(translation)),
            "rotation": Field(list(rotation)),
            "name": Field(name),
        }
        for joint in JOINT_FIELDS:
            self.fields[joint] = Field(0.0)
        self.articulated = articulated

    def getField(self, name):
        return self.fields.get(name)

    def getFromProtoDef(self, name):
        # `LEFT_ARM` is the HingeJoint DEF the PROTO only emits when it is not in rigid mode,
        # which is exactly what the controller probes for.
        return object() if self.articulated else None


# Every getter-only property the real `controller.Supervisor` defines, as of Webots R2025a. They
# are reproduced here for ONE reason: a controller that assigns `self.<name>` over one of them runs
# perfectly against a stub that does not have it, and dies on the first real Webots step with
# "property '<name>' of 'X' object has no setter". `mode` cost exactly that, so the stub now owns
# the same minefield the real class does. Refresh with:
#     WEBOTS_HOME=/usr/local/webots python3 -c "import sys; \
#       sys.path.insert(0, '/usr/local/webots/lib/controller/python'); from controller import \
#       Supervisor; print([n for n in dir(Supervisor) if isinstance(getattr(Supervisor, n, None), \
#       property) and getattr(Supervisor, n).fset is None])"
READ_ONLY_PROPERTIES = (
    "basic_time_step", "mode", "model", "name", "number_of_devices", "project_path",
    "supervisor", "synchronization", "time", "world_path",
)


class Supervisor:
    node = None
    step_budget = 10 ** 9
    step_delay = 0.0

    def __init__(self):
        self._time = 0.0

    def getBasicTimeStep(self):
        return BASIC_TIME_STEP

    def getSelf(self):
        return Supervisor.node

    def getTime(self):
        return self._time

    def step(self, ms):
        if Supervisor.step_budget <= 0:
            return -1
        Supervisor.step_budget -= 1
        self._time += ms / 1000.0
        if Supervisor.step_delay:
            time.sleep(Supervisor.step_delay)
        return 0


def install():
    """Put this stub in `sys.modules` as `controller`, before `human_walker` is imported."""
    module = types.ModuleType("controller")
    module.Supervisor = Supervisor
    module.Robot = Supervisor
    sys.modules["controller"] = module
    return module


def _install_read_only_properties():
    """Give the stub the same getter-only properties the real Supervisor has."""
    for name in READ_ONLY_PROPERTIES:
        if name in vars(Supervisor):
            continue   # the stub implements it for real; that one already behaves correctly
        setattr(Supervisor, name, property(
            lambda self, _name=name: getattr(self, "_stub_" + _name, None)))


_install_read_only_properties()
