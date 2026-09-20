# `human_walker` — an articulated human you can command

A drop-in replacement for Cyberbotics' stock `pedestrian` controller that turns
the Webots `Pedestrian` PROTO into something you can drive at run time:
**walk forward, turn, go to a point, follow a route, put a hand on X**.

It lives in `webots-shadow/worlds/avatar/`, which is **its own Webots project**, not a
plain world folder — see "Why this is a nested project" below.

```
worlds/beta_empty.wbt    the Pedestrian node, wired to this controller
controllers/human_walker/
  human_walker.py        the controller (Supervisor + TCP command server)
  pedestrian_kinematics.py  gait model and arm FK/IK — no Webots imports
  human_client.py        client library and command-line tool
  test_kinematics.py     26 offline checks of the maths
  test_walker.py         56 headless checks of the controller, Webots not needed
```

Both test files run without Webots: `python3 test_kinematics.py && python3 test_walker.py`.

## What this is, and what it is not

The `Pedestrian` PROTO has **13 hinge joints and no motors and no `Physics`**.
It is posed and moved, never simulated. That is the right trade for a person in
a perception or navigation scene — it never falls over, never needs balancing,
and obeys a velocity command exactly — and the wrong one if you need contact
dynamics or a gait that has to stay upright. Nothing here does force control.

The stock controller drives the same gait from *absolute simulation time* along
a *fixed* polyline, which is why it cannot be stopped, reversed or re-steered.
This controller keeps Cyberbotics' empirical 8-keyframe walk cycle unchanged and
replaces the clock and the interface: the gait phase advances with the distance
actually travelled, so the legs follow the body — they stop when it stops, step
backwards when it reverses, and shuffle in place during a pivot.

## The one trap in the PROTO

```
%< const rigid = fields.controllerArgs.value.length == 0; >%
```

**An empty `controllerArgs` makes the PROTO build a statue.** Every `HingeJoint`
is replaced by a fixed `Solid`, while the 13 joint fields remain on the node and
go on silently accepting writes that move nothing. `controllerArgs` must contain
at least one entry. The controller probes for the `LEFT_ARM` DEF at startup and
refuses to run with a message saying exactly this, rather than animating a statue.

## Why this is a nested project

`worlds/avatar/` holds its own `worlds/` and `controllers/` because Webots resolves a
controller against the **parent of the world file's own directory**, not the repo root.
A world at `webots-shadow/worlds/avatar/beta_empty.wbt` makes Webots search
`webots-shadow/worlds/controllers/`, which does not exist — it then silently starts
`<generic>` instead and the person never moves. The other worlds in this repo never hit
this because their robot is an `<extern>` controller, so Webots is never asked to find a
controller directory at all.

Keep the entry file named after its directory (`controllers/human_walker/human_walker.py`)
or the same silent fallback happens.

## The other robot gates the clock

`beta_empty.wbt` also contains a Shadow, and `protos/Shadow.proto` defaults it to
`controller "<extern>" synchronization TRUE`. A *synchronized extern* robot stops the whole
simulation until something connects to it, so with no `webots-bridge` running the world
loads, this controller starts and announces itself, and then every command comes back
`{"ok": false, "error": "simulation did not step"}`. That is the Shadow waiting, not a fault
here. Either run the bridge, or add `synchronization FALSE` to the world's `DEF shadow`
node to let the person move on their own.

## World setup

```
Pedestrian {
  translation -2.75764 -0.49671 1.23
  rotation 0 0 1 3.14159
  name "human"
  controller "human_walker"
  controllerArgs [ "--port=4321" ]
  enableBoundingObject TRUE
}
```

The initial `translation` and `rotation` are honoured: the human starts where the
world puts it, at the hip height the world gives it (`z`, not the PROTO's nominal
1.27), standing still until commanded.

`enableBoundingObject TRUE` gives it collision geometry, so the robot's navigation
sees a person rather than empty air. Because the body is teleported rather than
simulated, it behaves as an immovable obstacle that can walk into things; drop the
field to `FALSE` for a purely visual person that cameras and LiDAR see but nothing
can collide with.

### Controller arguments

| argument | default | meaning |
|---|---|---|
| `--port` / `--host` | `4321` / `127.0.0.1` | command server endpoint |
| `--no-server` | off | run with no server (use `--trajectory`) |
| `--speed` | `1.0` | cruise speed for `goto` / `follow`, m/s |
| `--max-speed` | `1.8` | m/s, clamps every command |
| `--max-yaw-rate` | `1.5` | rad/s, clamps every command |
| `--reach-tolerance` | `0.005` | m; how close the palm must be to count as reached |
| `--root-height` | world `z` | hip height, m |
| `--step` | world basic step | control period, ms |
| `--trajectory` | — | `"x1 y1, x2 y2, ..."`, looped: the stock controller's syntax |

## Driving it

```python
from human_client import HumanClient

with HumanClient(port=4321) as human:
    human.walk(0.9)                       # m/s, negative walks backwards
    human.turn(0.5)                       # rad/s, positive turns left
    human.goto(-2.0, 1.5, tol=0.2)        # walk there and stop
    human.follow([(0, 0), (2, 0), (2, 2)], loop=True)
    human.face(x=-3.0, y=0.0)
    human.stop()

    reply = human.reach(-1.8, 1.9, 1.10, arm="right")
    print(reply["residual"], reply["reachable"])
    human.relax("right")

    print(human.state()["pose"], human.state()["hands"]["right"])
```

From a shell:

```
python3 human_client.py walk 0.9
python3 human_client.py goto -2.0 1.5
python3 human_client.py reach -1.8 1.9 1.10 --arm right
python3 human_client.py state
python3 human_client.py raw '{"cmd":"follow","points":[[0,0],[2,0]],"loop":true}'
```

Or with nothing installed at all — the protocol is one JSON object per line:

```
printf '{"cmd":"walk","v":0.8}\n{"cmd":"state"}\n' | nc 127.0.0.1 4321
```

A reply is written only after the simulation step that applied the command, so a
`state` reply describes the world as it is once the command has taken effect, not
what was asked for.

## Commands

| command | fields | effect |
|---|---|---|
| `walk` | `v` | forward speed, m/s |
| `turn` | `w` | yaw rate, rad/s |
| `velocity` | `v`, `w` | both at once |
| `stop` | — | halt; the stance settles to neutral over ~0.45 s |
| `goto` | `x`, `y`, `tol`, `speed` | walk to a world point and stop |
| `follow` | `points`, `loop`, `tol`, `speed` | walk a route |
| `face` | `x`,`y` or `theta` | turn on the spot to a heading |
| `teleport` | `x`, `y`, `theta` | jump, and stop |
| `reach` | `x`,`y`,`z`, `arm`, `frame`, `turn_body`, `wrist` | put a palm on a point |
| `arm` | `arm`, `shoulder`, `elbow`, `wrist` | pose an arm in joint space |
| `relax` | `arm` (`left`/`right`/`both`) | hand the arm back to the gait |
| `head` | `angle`, or `relax` | hold a nod (the head hinge is a pitch, not a yaw) |
| `state` / `pose` | — | full state: pose, velocity, mode, gait phase, hand positions, arm residuals |
| `ping` | — | liveness and simulation time |

Anything unrecognised, or missing an argument, comes back as
`{"ok": false, "error": "..."}` and the connection stays up.

## `reach`: what the geometry actually allows

Every joint in the PROTO has axis `0 1 0`. **Each arm is therefore a planar
3-link chain confined to one sagittal plane** (the right palm's plane is
y = −0.26 m in the body frame, the left's y = +0.25 m). There is no shoulder
abduction and no humerus twist. An arbitrary 3-D point is not reachable by the
arm alone, and no amount of IK changes that.

So `reach` does the physically honest thing: with `turn_body` (default) it solves
in closed form the body yaw that brings the target into the arm plane, turns on
the spot, and runs 2-link planar IK for the shoulder and elbow with the wrist held
fixed. The reply never claims more than it achieved:

- `residual` — the true 3-D distance from palm to target, in metres
- `out_of_plane` / `in_plane` — the two ways this arm can miss
- `reachable` — `residual <= --reach-tolerance`
- `clamped_reach` / `clamped_limits` — hit the arm's span, or the joint limits
- `stand_distance` — how far from the target the body would have to stand

Palm reach from the shoulder spans **0.08 m to 0.65 m**. A target nearer the body
axis than the arm plane offset has *no* yaw solution — the target is inside the
cylinder the shoulder sweeps — and is refused with `"step back first"` rather than
approximated.

A held arm is re-solved every step, so it tracks the target as the body moves, and
it fades in and out over 0.35 s instead of snapping.

**`frame`** decides what the target means:

- `world` (default) — a place in the room. Walk away and the hand comes off it;
  `residual` grows and `reachable` goes false. This is "put your hand on that box".
- `body` — a pose relative to the human, which travels with them. This is
  "hold your arm out in front", or carrying something.

## Tests

`test_kinematics.py` checks the maths against the PROTO geometry: FK/IK round-trip
over the reachable annulus (worst residual 3.5e-16 m), unreachable and off-plane
targets reported rather than faked, anatomical elbow sign, the body-yaw solution,
frame transforms, and the distance-driven gait clock.

`test_walker.py` runs the controller headless against a stand-in `controller`
module — no Webots — and covers the rigid-PROTO refusal, distance covered while
walking, settling to neutral on stop, reversing, pivoting, speed clamps, `goto`
arrival, open and looped routes, world- and body-frame reaches, arm blending,
malformed commands, and the TCP protocol end to end through `human_client`.

## Adding more people

Each Pedestrian needs its own `name` and its own `--port`. They are independent;
nothing here coordinates them.

## If you want a photoreal person instead

`CharacterSkin` (`WEBOTS_HOME/projects/humans/skin_animated_humans/`) gives four
MakeHuman characters driven by BVH mocap, which look far more like people to a
detector than this low-poly mesh. They have no collision geometry and no joints
to command — the `Skin` API is bone-level only — so the natural combination is to
keep this controller for steering and mount a `CharacterSkin` for appearance.
That is not wired up here.
