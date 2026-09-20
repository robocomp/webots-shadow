#!/usr/bin/env python3
"""Client for the `human_walker` Webots controller.

As a library:

    from human_client import HumanClient
    with HumanClient(port=4321) as human:
        human.walk(0.9)
        human.goto(-2.0, 1.5, tol=0.2)
        print(human.reach(-1.8, 1.9, 1.10, arm="right"))
        print(human.state()["pose"])

From the shell:

    python3 human_client.py walk 0.9
    python3 human_client.py goto -2.0 1.5
    python3 human_client.py reach -1.8 1.9 1.10 --arm right
    python3 human_client.py state
    python3 human_client.py raw '{"cmd": "follow", "points": [[0,0],[2,0]], "loop": true}'

Every method returns the controller's reply as a dict; `ok` is False and
`error` carries the reason when a command could not be honoured.
"""

import argparse
import json
import socket
import sys


class HumanError(RuntimeError):
    pass


class HumanClient:
    def __init__(self, host="127.0.0.1", port=4321, timeout=10.0):
        self._socket = socket.create_connection((host, port), timeout=timeout)
        self._stream = self._socket.makefile("rw", encoding="utf-8", newline="\n")

    # -- plumbing ------------------------------------------------------------
    def send(self, **command):
        self._stream.write(json.dumps(command) + "\n")
        self._stream.flush()
        line = self._stream.readline()
        if not line:
            raise HumanError("controller closed the connection")
        return json.loads(line)

    def close(self):
        try:
            self._stream.close()
        finally:
            self._socket.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    # -- locomotion ----------------------------------------------------------
    def walk(self, v):
        """Forward speed in m/s; negative walks backwards."""
        return self.send(cmd="walk", v=v)

    def turn(self, w):
        """Yaw rate in rad/s; positive turns left."""
        return self.send(cmd="turn", w=w)

    def velocity(self, v, w):
        return self.send(cmd="velocity", v=v, w=w)

    def stop(self):
        return self.send(cmd="stop")

    def goto(self, x, y, tol=0.15, speed=None):
        """Walk to a world point and stop there.  Returns once *accepted*."""
        command = {"cmd": "goto", "x": x, "y": y, "tol": tol}
        if speed is not None:
            command["speed"] = speed
        return self.send(**command)

    def follow(self, points, loop=False, tol=0.25, speed=None):
        command = {"cmd": "follow", "points": [list(p) for p in points],
                   "loop": loop, "tol": tol}
        if speed is not None:
            command["speed"] = speed
        return self.send(**command)

    def face(self, x=None, y=None, theta=None):
        if theta is not None:
            return self.send(cmd="face", theta=theta)
        return self.send(cmd="face", x=x, y=y)

    def teleport(self, x, y, theta=None):
        command = {"cmd": "teleport", "x": x, "y": y}
        if theta is not None:
            command["theta"] = theta
        return self.send(**command)

    # -- arms and head -------------------------------------------------------
    def reach(self, x, y, z, arm="right", turn_body=True, wrist=0.0,
              frame="world"):
        """Put the palm on a point.

        The Pedestrian's shoulder has a single hinge, so the arm lives in one
        sagittal plane; with `turn_body` the human first turns so that plane
        contains the target.  The reply carries `residual` (how far the palm
        actually ends up from the request) and `reachable`.

        `frame="world"` is a place in the room: walk away and the hand comes
        off it and the residual says so.  `frame="body"` is a pose relative to
        the human and travels with them, for carrying or gesturing.
        """
        return self.send(cmd="reach", x=x, y=y, z=z, arm=arm, frame=frame,
                         turn_body=turn_body, wrist=wrist)

    def arm(self, shoulder=0.0, elbow=0.0, wrist=0.0, arm="right"):
        return self.send(cmd="arm", arm=arm, shoulder=shoulder, elbow=elbow,
                         wrist=wrist)

    def relax(self, arm="both"):
        return self.send(cmd="relax", arm=arm)

    def head(self, angle=None):
        """Nod angle in rad (the head hinge is a pitch); None hands it back."""
        if angle is None:
            return self.send(cmd="head", relax=True)
        return self.send(cmd="head", angle=angle)

    # -- introspection -------------------------------------------------------
    def state(self):
        return self.send(cmd="state")

    def ping(self):
        return self.send(cmd="ping")


def main(argv):
    parser = argparse.ArgumentParser(prog="human_client")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=4321)
    parser.add_argument("--arm", default="right", choices=["left", "right"])
    parser.add_argument("--no-turn-body", action="store_true")
    parser.add_argument("action")
    parser.add_argument("values", nargs="*")
    options = parser.parse_args(argv)

    numbers = [float(v) for v in options.values if options.action != "raw"]
    with HumanClient(options.host, options.port) as human:
        action = options.action
        if action == "walk":
            reply = human.walk(numbers[0])
        elif action == "turn":
            reply = human.turn(numbers[0])
        elif action == "velocity":
            reply = human.velocity(numbers[0], numbers[1])
        elif action == "stop":
            reply = human.stop()
        elif action == "goto":
            reply = human.goto(numbers[0], numbers[1],
                               *(numbers[2:3] or [0.15]))
        elif action == "face":
            reply = (human.face(theta=numbers[0]) if len(numbers) == 1
                     else human.face(x=numbers[0], y=numbers[1]))
        elif action == "teleport":
            reply = human.teleport(*numbers)
        elif action == "reach":
            reply = human.reach(numbers[0], numbers[1], numbers[2],
                                arm=options.arm,
                                turn_body=not options.no_turn_body)
        elif action == "arm":
            reply = human.arm(*numbers, arm=options.arm)
        elif action == "relax":
            reply = human.relax(options.arm)
        elif action == "head":
            reply = human.head(numbers[0] if numbers else None)
        elif action in ("state", "pose"):
            reply = human.state()
        elif action == "ping":
            reply = human.ping()
        elif action == "raw":
            reply = human.send(**json.loads(options.values[0]))
        else:
            parser.error("unknown action %r" % action)
    print(json.dumps(reply, indent=2, sort_keys=True))
    return 0 if reply.get("ok", False) else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
