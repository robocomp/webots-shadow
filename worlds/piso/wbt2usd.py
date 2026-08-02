#!/usr/bin/env python3
"""
wbt2usd.py — Convert a Webots .wbt world into an OpenUSD stage for Isaac Sim.

Usage
-----
    # With Isaac Sim's bundled interpreter (recommended: has pxr + omni):
    ./python.sh wbt2usd.py ~/robocomp/components/webots-piso/worlds/Piso.wbt -o piso.usda

    # Or standalone:
    pip install usd-core trimesh
    python wbt2usd.py Piso.wbt -o piso.usda --webots-home /usr/local/webots

What it does
------------
  * Tokenises + parses the VRML97 subset that Webots uses.
  * Resolves EXTERNPROTO / local .proto files and inlines them, substituting IS fields.
  * Maps Solid/Pose/Transform/Group -> Xform, Shape geometry -> UsdGeom gprims,
    PBRAppearance/Appearance -> UsdPreviewSurface, boundingObject -> collision prims,
    Physics -> RigidBodyAPI + MassAPI, HingeJoint/SliderJoint -> UsdPhysics joints,
    lights -> UsdLux.
  * Handles the ENU/NUE coordinate-system switch and Webots' Y-up cylinder convention.

What it does NOT do
-------------------
  * Procedural (JavaScript-templated) PROTOs. Those contain `%<` ... `>%` blocks and
    require Webots' template engine. They are reported and emitted as empty, correctly
    placed Xform placeholders so you can fill them in. See --list-procedural.
  * Motors/sensors/controllers. Joints get their kinematic structure only.
"""

from __future__ import annotations

import argparse
import copy
import math
import os
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable

try:
    from pxr import Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade, Gf, Sdf, Tf, Vt
except ImportError:
    sys.exit("pxr not found. Use Isaac Sim's python.sh, or: pip install usd-core")


# --------------------------------------------------------------------------- #
#  Tokeniser
# --------------------------------------------------------------------------- #

TOKEN_RE = re.compile(
    r'''
      (?P<string>"(?:[^"\\]|\\.)*")
    | (?P<number>[-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][-+]?\d+)?)
    | (?P<punct>[{}\[\],])
    | (?P<ident>[^\s{}\[\],"#]+)
    ''',
    re.VERBOSE,
)


def tokenize(text: str) -> list[str]:
    """Strip comments (outside strings) and split into tokens."""
    out: list[str] = []
    for raw_line in text.splitlines():
        line, in_str, cut = raw_line, False, None
        for i, ch in enumerate(line):
            if ch == '"' and (i == 0 or line[i - 1] != "\\"):
                in_str = not in_str
            elif ch == "#" and not in_str:
                cut = i
                break
        if cut is not None:
            line = line[:cut]
        for m in TOKEN_RE.finditer(line):
            out.append(m.group(0))
    return out


# --------------------------------------------------------------------------- #
#  Node model + parser
# --------------------------------------------------------------------------- #


@dataclass
class Node:
    type: str
    fields: dict[str, Any] = field(default_factory=dict)
    def_name: str | None = None

    def get(self, name: str, default=None):
        return self.fields.get(name, default)


@dataclass
class IsRef:
    """Placeholder for `field IS protoField` inside a PROTO body."""
    name: str


class Parser:
    def __init__(self, tokens: list[str]):
        self.t = tokens
        self.i = 0
        self.defs: dict[str, Node] = {}

    # -- token helpers ----------------------------------------------------- #
    def peek(self, k: int = 0) -> str | None:
        j = self.i + k
        return self.t[j] if j < len(self.t) else None

    def next(self) -> str:
        tok = self.t[self.i]
        self.i += 1
        return tok

    def expect(self, tok: str) -> None:
        got = self.next()
        if got != tok:
            raise SyntaxError(f"expected {tok!r}, got {got!r} near {self.t[max(0,self.i-6):self.i+4]}")

    @staticmethod
    def is_number(tok: str | None) -> bool:
        if tok is None:
            return False
        try:
            float(tok)
            return True
        except ValueError:
            return False

    @staticmethod
    def is_string(tok: str | None) -> bool:
        return bool(tok) and tok.startswith('"')

    @staticmethod
    def unquote(tok: str) -> str:
        return tok[1:-1].replace('\\"', '"')

    # -- grammar ----------------------------------------------------------- #
    def parse_top(self) -> list[Node]:
        nodes = []
        while self.i < len(self.t):
            if self.peek() in ("EXTERNPROTO", "IMPORTABLE"):
                self.next()
                if self.is_string(self.peek()):
                    self.next()
                continue
            nodes.append(self.parse_node())
        return nodes

    def parse_node(self) -> Node:
        def_name = None
        if self.peek() == "DEF":
            self.next()
            def_name = self.next()
        if self.peek() == "USE":
            self.next()
            ref = self.next()
            src = self.defs.get(ref)
            if src is None:
                raise SyntaxError(f"USE {ref} before DEF")
            return copy.deepcopy(src)

        ntype = self.next()
        self.expect("{")
        node = Node(ntype, def_name=def_name)
        while self.peek() != "}":
            fname = self.next()
            node.fields[fname] = self.parse_value()
        self.expect("}")
        if def_name:
            self.defs[def_name] = node
        return node

    def parse_value(self) -> Any:
        tok = self.peek()
        if tok == "[":
            self.next()
            items: list[Any] = []
            while self.peek() != "]":
                if self.peek() == ",":
                    self.next()
                    continue
                items.append(self.parse_item())
            self.expect("]")
            # flatten [[1,2],[3,4]] scalars into one list
            flat: list[Any] = []
            for it in items:
                if isinstance(it, list):
                    flat.extend(it)
                else:
                    flat.append(it)
            return flat
        if tok == "IS":
            self.next()
            return IsRef(self.next())
        if tok == "NULL":
            self.next()
            return None
        if tok in ("TRUE", "FALSE"):
            self.next()
            return tok == "TRUE"
        if self.is_number(tok):
            vals = []
            while self.is_number(self.peek()):
                vals.append(float(self.next()))
            return vals[0] if len(vals) == 1 else vals
        if self.is_string(tok):
            vals = []
            while self.is_string(self.peek()):
                vals.append(self.unquote(self.next()))
            return vals[0] if len(vals) == 1 else vals
        return self.parse_node()

    def parse_item(self) -> Any:
        tok = self.peek()
        if self.is_number(tok):
            vals = []
            while self.is_number(self.peek()):
                vals.append(float(self.next()))
            return vals
        if self.is_string(tok):
            return self.unquote(self.next())
        if tok in ("TRUE", "FALSE"):
            self.next()
            return tok == "TRUE"
        return self.parse_node()


# --------------------------------------------------------------------------- #
#  PROTO resolution
# --------------------------------------------------------------------------- #


@dataclass
class Proto:
    name: str
    defaults: dict[str, Any]
    body: Node
    procedural: bool = False


class ProtoLibrary:
    """Finds .proto files on disk and expands PROTO instances into raw nodes."""

    def __init__(self, world: Path, webots_home: Path | None, extra: Iterable[Path] = ()):
        self.world = world
        self.webots_home = webots_home
        self.search: list[Path] = [world.parent, world.parent / ".." / "protos"]
        self.search += [Path(p) for p in extra]
        if webots_home:
            self.search.append(webots_home / "projects")
        self.cache: dict[str, Proto | None] = {}
        self.procedural: set[str] = set()
        self.missing: set[str] = set()
        self._index: dict[str, Path] | None = None

    def _build_index(self) -> dict[str, Path]:
        idx: dict[str, Path] = {}
        for root in self.search:
            root = root.resolve()
            if not root.is_dir():
                continue
            for p in root.rglob("*.proto"):
                idx.setdefault(p.stem, p)
        return idx

    def find(self, name: str) -> Path | None:
        if self._index is None:
            self._index = self._build_index()
        return self._index.get(name)

    def load(self, name: str) -> Proto | None:
        if name in self.cache:
            return self.cache[name]
        path = self.find(name)
        if path is None:
            self.missing.add(name)
            self.cache[name] = None
            return None
        text = path.read_text(encoding="utf-8", errors="replace")
        if "%<" in text:
            # JavaScript-templated PROTO: cannot expand without Webots' engine.
            self.procedural.add(name)
            self.cache[name] = None
            return None
        proto = self._parse_proto(name, text)
        self.cache[name] = proto
        return proto

    def _parse_proto(self, name: str, text: str) -> Proto | None:
        toks = tokenize(text)
        try:
            start = toks.index("PROTO")
        except ValueError:
            return None
        p = Parser(toks[start:])
        p.expect("PROTO")
        p.next()                      # proto name
        p.expect("[")
        defaults: dict[str, Any] = {}
        while p.peek() != "]":
            access = p.next()         # field / vrmlField / exposedField / hiddenField
            if access not in ("field", "vrmlField", "exposedField", "hiddenField"):
                continue
            p.next()                  # SF*/MF* type
            fname = p.next()
            defaults[fname] = p.parse_value()
        p.expect("]")
        body = p.parse_node()
        return Proto(name, defaults, body)

    # -- expansion --------------------------------------------------------- #
    def expand(self, node: Node) -> Node | None:
        """Recursively inline PROTO instances. Returns None if unresolvable."""
        proto = self.load(node.type)
        if proto is None:
            return None
        args = dict(proto.defaults)
        args.update(node.fields)
        body = copy.deepcopy(proto.body)
        _substitute(body, args)
        # A PROTO instance's own translation/rotation/name override the body's.
        for k in ("translation", "rotation", "scale", "name"):
            if k in node.fields:
                body.fields[k] = node.fields[k]
        if node.def_name:
            body.def_name = node.def_name
        return body


KNOWN_TYPES = {
    # containers
    "Solid", "Pose", "Transform", "Group", "Shape", "Robot", "Slot",
    # geometry
    "Box", "Sphere", "Cylinder", "Capsule", "Cone", "Plane", "Mesh",
    "IndexedFaceSet", "ElevationGrid", "IndexedLineSet",
    # appearance
    "Appearance", "PBRAppearance", "Material", "ImageTexture", "TextureTransform",
    "Coordinate", "TextureCoordinate", "Normal",
    # physics
    "Physics", "HingeJoint", "HingeJointParameters", "JointParameters",
    "SliderJoint", "Hinge2Joint", "BallJoint",
    # lights / world
    "DirectionalLight", "PointLight", "SpotLight",
    "WorldInfo", "Viewpoint", "Background", "TexturedBackground",
    "TexturedBackgroundLight", "Fog",
    # devices we skip but recognise
    "RotationalMotor", "LinearMotor", "PositionSensor", "Brake",
}


def _substitute(node: Node, args: dict[str, Any]) -> None:
    for k, v in list(node.fields.items()):
        node.fields[k] = _sub_value(v, args)


def _sub_value(v: Any, args: dict[str, Any]) -> Any:
    if isinstance(v, IsRef):
        return copy.deepcopy(args.get(v.name))
    if isinstance(v, Node):
        _substitute(v, args)
        return v
    if isinstance(v, list):
        return [_sub_value(x, args) for x in v]
    return v


# --------------------------------------------------------------------------- #
#  Built-in synthesis for the structural PROTOs
#
#  Floor / Wall / Ceiling are JavaScript-templated in Webots, but their geometry
#  is simple enough to reconstruct from `size` alone. This lets us get the shell
#  of the apartment out without running Webots' template engine.
# --------------------------------------------------------------------------- #

BUILTIN_PROTOS = {"Floor", "Wall", "Ceiling", "CircleArena", "RectangleArena"}


def synth_builtin(node: Node, enu: bool, wall_base_at_origin: bool) -> Node | None:
    """Return a plain Solid{children[Shape{...}]} equivalent, or None."""
    t = node.type
    up = 2 if enu else 1                      # index of the vertical axis

    def solid(children: list[Node], bo: Node | None) -> Node:
        s = Node("Solid")
        for k in ("translation", "rotation", "name"):
            if k in node.fields:
                s.fields[k] = node.fields[k]
        s.fields.setdefault("name", t.lower())
        s.fields["children"] = children
        if bo is not None:
            s.fields["boundingObject"] = bo
        return s

    def shape(geom: Node) -> Node:
        sh = Node("Shape")
        sh.fields["geometry"] = geom
        app = node.get("appearance")
        if isinstance(app, Node):
            sh.fields["appearance"] = app
        return sh

    def posed(geom: Node, offset: float) -> Node:
        if abs(offset) < 1e-12:
            return shape(geom)
        p = Node("Pose")
        tr = [0.0, 0.0, 0.0]
        tr[up] = offset
        p.fields["translation"] = tr
        p.fields["children"] = [shape(geom)]
        return p

    if t in ("Floor", "Ceiling"):
        sz = node.get("size") or [10.0, 10.0]
        plane = Node("Plane", {"size": [float(sz[0]), float(sz[1])]})
        return solid([shape(plane)], copy.deepcopy(plane))

    if t == "Wall":
        sz = node.get("size") or [0.2, 1.0, 2.4]
        sz = [float(x) for x in sz]
        box = Node("Box", {"size": sz})
        # Webots' Wall.proto sits its box on the floor rather than centring it.
        off = sz[up] / 2.0 if wall_base_at_origin else 0.0
        return solid([posed(box, off)], posed(copy.deepcopy(box), off))

    if t == "RectangleArena":
        sz = node.get("floorSize") or [4.0, 4.0]
        plane = Node("Plane", {"size": [float(sz[0]), float(sz[1])]})
        return solid([shape(plane)], copy.deepcopy(plane))

    if t == "CircleArena":
        r = as_float(node.get("radius"), 1.0)
        cyl = Node("Cylinder", {"radius": r, "height": 0.01})
        return solid([shape(cyl)], copy.deepcopy(cyl))

    return None


# --------------------------------------------------------------------------- #
#  Small helpers
# --------------------------------------------------------------------------- #


def as_vec3(v: Any, default=(0.0, 0.0, 0.0)) -> Gf.Vec3d:
    if isinstance(v, (list, tuple)) and len(v) >= 3:
        return Gf.Vec3d(float(v[0]), float(v[1]), float(v[2]))
    if isinstance(v, (int, float)):
        return Gf.Vec3d(float(v), float(v), float(v))
    return Gf.Vec3d(*default)


def as_rot(v: Any) -> Gf.Quatf:
    """Webots rotation = axis(x,y,z) + angle(rad)."""
    if not (isinstance(v, (list, tuple)) and len(v) >= 4):
        return Gf.Quatf(1.0, 0.0, 0.0, 0.0)
    axis = Gf.Vec3d(float(v[0]), float(v[1]), float(v[2]))
    if axis.GetLength() < 1e-12:
        return Gf.Quatf(1.0, 0.0, 0.0, 0.0)
    q = Gf.Rotation(axis.GetNormalized(), math.degrees(float(v[3]))).GetQuat()
    return Gf.Quatf(float(q.GetReal()), Gf.Vec3f(*q.GetImaginary()))


def as_float(v: Any, default: float) -> float:
    if isinstance(v, (int, float)):
        return float(v)
    if isinstance(v, list) and v and isinstance(v[0], (int, float)):
        return float(v[0])
    return default


def align_x_to(axis: Gf.Vec3d) -> Gf.Quatf:
    """Rotation taking +X onto `axis` (UsdPhysics joints reference their X axis)."""
    a = axis.GetNormalized() if axis.GetLength() > 1e-12 else Gf.Vec3d(1, 0, 0)
    q = Gf.Rotation(Gf.Vec3d(1, 0, 0), a).GetQuat()
    return Gf.Quatf(float(q.GetReal()), Gf.Vec3f(*q.GetImaginary()))


# --------------------------------------------------------------------------- #
#  Converter
# --------------------------------------------------------------------------- #


class Converter:
    def __init__(self, stage: Usd.Stage, protos: ProtoLibrary, args: argparse.Namespace,
                 out_dir: Path):
        self.stage = stage
        self.protos = protos
        self.args = args
        self.out_dir = out_dir
        self.enu = True
        self.used: set[str] = set()
        self.materials: dict[Any, UsdShade.Material] = {}
        self.joints: list[tuple] = []
        self.stats: dict[str, int] = {}
        self.placeholders: list[str] = []
        self.notes: set[str] = set()
        self.missing_meshes: set[str] = set()
        self.missing_textures: set[str] = set()
        self._mesh_cache: dict[Path, str] = {}
        self._dae_seen: set[Path] = set()
        self._keep_re = re.compile(args.only) if args.only else None

    # -- naming ------------------------------------------------------------ #
    def unique(self, parent: str, hint: str) -> str:
        base = Tf.MakeValidIdentifier(str(hint) or "Node")
        cand, n = base, 1
        while f"{parent}/{cand}" in self.used:
            n += 1
            cand = f"{base}_{n}"
        self.used.add(f"{parent}/{cand}")
        return f"{parent}/{cand}"

    def bump(self, key: str) -> None:
        self.stats[key] = self.stats.get(key, 0) + 1

    # -- entry point ------------------------------------------------------- #
    def run(self, roots: list[Node]) -> None:
        for n in roots:
            if n.type == "WorldInfo":
                cs = n.get("coordinateSystem", "ENU")
                self.enu = (cs if isinstance(cs, str) else "ENU").upper().startswith("ENU")

        world = UsdGeom.Xform.Define(self.stage, "/World")
        self.stage.SetDefaultPrim(world.GetPrim())
        if not self.enu:
            # Webots NUE (Y-up) -> USD Z-up.
            world.AddRotateXOp().Set(90.0)
            print("[i] world uses NUE; inserted +90deg X rotation at /World")

        for n in roots:
            self.emit(n, "/World")

        for jd in self.joints:
            self.make_joint(*jd)

        if self.args.prune_empty:
            self.prune_empty()

    def prune_empty(self) -> None:
        """Filtering can leave childless Xform/Scope husks; drop them bottom-up."""
        removed = 1
        while removed:
            removed = 0
            for prim in list(self.stage.Traverse()):
                if prim.GetPath() == Sdf.Path("/World"):
                    continue
                if prim.GetTypeName() not in ("Xform", "Scope"):
                    continue
                if prim.GetChildren() or prim.HasAuthoredReferences():
                    continue
                self.stage.RemovePrim(prim.GetPath())
                removed += 1
                self.bump("pruned_empty")

    # -- filtering --------------------------------------------------------- #
    def wanted(self, node: Node) -> bool:
        """A Solid survives --only if it, or any descendant Solid, matches."""
        if self._keep_re is None:
            return True
        name = node.get("name")
        if isinstance(name, str) and self._keep_re.search(name):
            return True
        for child in node.get("children", []) or []:
            if isinstance(child, Node) and child.type in ("Solid", "Robot", "Group", "Pose",
                                                          "Transform", "Slot"):
                if self.wanted(child):
                    return True
        return False

    # -- dispatch ---------------------------------------------------------- #
    def emit(self, node: Node, parent: str) -> None:
        t = node.type

        if t in ("WorldInfo", "Viewpoint", "Background", "TexturedBackground", "Fog"):
            return
        if self._keep_re is not None and t in ("Solid", "Robot") and not self.wanted(node):
            self.bump("filtered_out")
            return
        if t == "TexturedBackgroundLight":
            dl = UsdLux.DomeLight.Define(self.stage, self.unique(parent, "DomeLight"))
            dl.CreateIntensityAttr(1000.0)
            return
        if t in ("DirectionalLight", "PointLight", "SpotLight"):
            self.emit_light(node, parent)
            return
        if t == "Robot" and self.args.skip_robots:
            print(f"[i] skipping Robot '{node.get('name', '?')}' (import separately via URDF)")
            return
        if t in ("Solid", "Robot", "Pose", "Transform", "Group", "Slot"):
            self.emit_xform(node, parent)
            return
        if t == "Shape":
            self.emit_shape(node, parent)
            return
        if t in ("HingeJoint", "SliderJoint", "Hinge2Joint", "BallJoint"):
            self.emit_joint(node, parent)
            return
        if t in KNOWN_TYPES:
            return

        # Structural PROTOs we can rebuild from first principles.
        if t in BUILTIN_PROTOS:
            syn = synth_builtin(node, self.enu, self.args.wall_base_at_origin)
            if syn is not None:
                self.bump(f"builtin:{t}")
                self.emit_xform(syn, parent)
                return

        if self.args.shell_only:
            return

        # Unknown type -> assume PROTO instance.
        expanded = self.protos.expand(node)
        if expanded is not None:
            self.bump(f"proto:{t}")
            expanded.fields.setdefault("name", node.get("name", t))
            self.emit(expanded, parent)
            return

        # Unresolvable: leave a correctly-posed empty Xform so nothing shifts.
        path = self.unique(parent, node.get("name", t))
        xf = UsdGeom.Xform.Define(self.stage, path)
        self.set_xform(xf, node)
        xf.GetPrim().SetCustomDataByKey("webots:unresolvedProto", t)
        self.placeholders.append(f"{path}  <- {t}")

    # -- transforms -------------------------------------------------------- #
    def set_xform(self, xf: UsdGeom.Xformable, node: Node) -> None:
        tr = node.get("translation")
        if tr:
            xf.AddTranslateOp().Set(as_vec3(tr))
        rot = node.get("rotation")
        if rot:
            xf.AddOrientOp().Set(as_rot(rot))
        sc = node.get("scale")
        if sc:
            v = as_vec3(sc, (1, 1, 1))
            if v != Gf.Vec3d(1, 1, 1):
                xf.AddScaleOp().Set(Gf.Vec3f(v))

    def emit_xform(self, node: Node, parent: str) -> None:
        name = node.get("name") or node.def_name or node.type
        path = self.unique(parent, name)
        xf = UsdGeom.Xform.Define(self.stage, path)
        self.set_xform(xf, node)
        prim = xf.GetPrim()
        self.bump(node.type)

        phys = node.get("physics")
        dynamic = isinstance(phys, Node) and phys.type == "Physics"
        if dynamic and not self.args.no_physics:
            UsdPhysics.RigidBodyAPI.Apply(prim)
            m = as_float(phys.get("mass"), -1.0)
            if m > 0:
                UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(m)
        prim.SetCustomDataByKey("webots:type", node.type)
        if isinstance(name, str):
            prim.SetCustomDataByKey("webots:name", name)

        for child in node.get("children", []) or []:
            if isinstance(child, Node):
                self.emit(child, path)

        bo = node.get("boundingObject")
        if bo is not None and not self.args.no_physics:
            self.emit_collision(bo, path, dynamic)

    # -- collisions -------------------------------------------------------- #
    dynamic_ctx = False

    def emit_collision(self, node: Any, parent: str, dynamic: bool) -> None:
        if not isinstance(node, Node):
            return
        scope = f"{parent}/Collisions"
        if not self.stage.GetPrimAtPath(scope):
            UsdGeom.Scope.Define(self.stage, scope)
            self.used.add(scope)
        prev, self.dynamic_ctx = self.dynamic_ctx, dynamic
        self._collider(node, scope)
        self.dynamic_ctx = prev

    def _collider(self, node: Node, parent: str) -> None:
        if node.type in ("Pose", "Transform", "Group"):
            path = self.unique(parent, node.type)
            xf = UsdGeom.Xform.Define(self.stage, path)
            self.set_xform(xf, node)
            for c in node.get("children", []) or []:
                if isinstance(c, Node):
                    self._collider(c, path)
            return
        if node.type == "Shape":
            geo = node.get("geometry")
            if isinstance(geo, Node):
                self._collider(geo, parent)
            return
        prim = self.make_geometry(node, parent)
        if not prim:
            return
        UsdPhysics.CollisionAPI.Apply(prim)
        if prim.IsA(UsdGeom.Mesh):
            # PhysX needs to be told how to treat an arbitrary triangle soup.
            # Static geometry can use the raw triangles; dynamic bodies cannot.
            approx = self.args.dynamic_mesh_approx if self.dynamic_ctx else "none"
            UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr(approx)
        UsdGeom.Imageable(prim).CreatePurposeAttr(UsdGeom.Tokens.guide)
        self.bump("collider")

    # -- shapes ------------------------------------------------------------ #
    def emit_shape(self, node: Node, parent: str) -> None:
        geo = node.get("geometry")
        if not isinstance(geo, Node):
            return
        prim = self.make_geometry(geo, parent)
        if prim is None:
            return
        app = node.get("appearance")
        if isinstance(app, Node):
            mat = self.make_material(app)
            if mat:
                UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat)
        if self.args.static_colliders and not self.args.no_physics:
            UsdPhysics.CollisionAPI.Apply(prim)

    def make_geometry(self, g: Node, parent: str) -> Usd.Prim | None:
        t = g.type
        path = self.unique(parent, g.def_name or t)
        self.bump(f"geom:{t}")

        if t == "Box":
            size = as_vec3(g.get("size"), (2, 2, 2))
            cube = UsdGeom.Cube.Define(self.stage, path)
            cube.CreateSizeAttr(1.0)
            cube.AddScaleOp().Set(Gf.Vec3f(size))
            cube.CreateExtentAttr([Gf.Vec3f(-0.5, -0.5, -0.5), Gf.Vec3f(0.5, 0.5, 0.5)])
            return cube.GetPrim()

        if t == "Sphere":
            r = as_float(g.get("radius"), 1.0)
            s = UsdGeom.Sphere.Define(self.stage, path)
            s.CreateRadiusAttr(r)
            s.CreateExtentAttr([Gf.Vec3f(-r, -r, -r), Gf.Vec3f(r, r, r)])
            return s.GetPrim()

        if t in ("Cylinder", "Capsule", "Cone"):
            r = as_float(g.get("radius"), 1.0)
            h = as_float(g.get("height"), 2.0)
            if t == "Cylinder":
                p = UsdGeom.Cylinder.Define(self.stage, path)
                p.CreateHeightAttr(h)
                p.CreateRadiusAttr(r)
            elif t == "Capsule":
                p = UsdGeom.Capsule.Define(self.stage, path)
                p.CreateHeightAttr(h)
                p.CreateRadiusAttr(r)
            else:
                p = UsdGeom.Cone.Define(self.stage, path)
                p.CreateHeightAttr(h)
                p.CreateRadiusAttr(as_float(g.get("bottomRadius"), r))
            # Webots stacks these along local +Y; USD defaults to +Z.
            p.CreateAxisAttr("Y")
            half = h / 2 + (r if t == "Capsule" else 0.0)
            p.CreateExtentAttr([Gf.Vec3f(-r, -half, -r), Gf.Vec3f(r, half, r)])
            return p.GetPrim()

        if t == "Plane":
            sx, sy = 1.0, 1.0
            sz = g.get("size")
            if isinstance(sz, list) and len(sz) >= 2:
                sx, sy = float(sz[0]), float(sz[1])
            hx, hy = sx / 2, sy / 2
            mesh = UsdGeom.Mesh.Define(self.stage, path)
            if self.enu:   # plane lies in local XY, normal +Z
                pts = [(-hx, -hy, 0), (hx, -hy, 0), (hx, hy, 0), (-hx, hy, 0)]
            else:          # NUE: plane lies in local XZ, normal +Y
                pts = [(-hx, 0, -hy), (-hx, 0, hy), (hx, 0, hy), (hx, 0, -hy)]
            mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in pts])
            mesh.CreateFaceVertexCountsAttr([4])
            mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
            UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
                "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex
            ).Set([Gf.Vec2f(0, 0), Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)])
            return mesh.GetPrim()

        if t == "IndexedFaceSet":
            return self.make_ifs(g, path)

        if t == "Mesh":
            return self.make_mesh_file(g, path)

        print(f"[!] unsupported geometry: {t}")
        return None

    def make_ifs(self, g: Node, path: str) -> Usd.Prim | None:
        coord = g.get("coord")
        if not isinstance(coord, Node):
            return None
        flat = coord.get("point") or []
        pts = [Gf.Vec3f(float(flat[i]), float(flat[i + 1]), float(flat[i + 2]))
               for i in range(0, len(flat) - 2, 3)] if flat and isinstance(flat[0], float) \
              else [Gf.Vec3f(*p[:3]) for p in flat if isinstance(p, list)]
        idx = g.get("coordIndex") or []
        counts, indices, run = [], [], []
        for v in idx:
            v = int(v)
            if v < 0:
                if len(run) >= 3:
                    counts.append(len(run))
                    indices.extend(run)
                run = []
            else:
                run.append(v)
        if len(run) >= 3:
            counts.append(len(run))
            indices.extend(run)
        if not counts:
            return None
        mesh = UsdGeom.Mesh.Define(self.stage, path)
        mesh.CreatePointsAttr(pts)
        mesh.CreateFaceVertexCountsAttr(counts)
        mesh.CreateFaceVertexIndicesAttr(indices)
        mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        return mesh.GetPrim()

    def resolve_url(self, g: Node) -> Path | None:
        url = g.get("url")
        url = url[0] if isinstance(url, list) and url else url
        if not isinstance(url, str):
            return None
        if url.startswith(("http://", "https://")):
            print(f"[!] remote url not fetched: {url}")
            return None
        return (self.protos.world.parent / url).resolve()

    def report_dae_up_axis(self, src: Path) -> None:
        """Webots and trimesh may disagree on COLLADA <up_axis>; surface it."""
        if src.suffix.lower() != ".dae" or src in self._dae_seen:
            return
        self._dae_seen.add(src)
        m = re.search(rb"<up_axis>\s*([A-Z_]+)\s*</up_axis>", src.read_bytes()[:4096])
        axis = m.group(1).decode() if m else "unspecified"
        if axis != "Z_UP":
            print(f"[i] {src.name}: <up_axis> is {axis} — verify orientation in Isaac")

    def make_mesh_file(self, g: Node, path: str) -> Usd.Prim | None:
        src = self.resolve_url(g)
        if src is None:
            return None
        if not src.exists():
            self.missing_meshes.add(str(src))
            return None
        if src.suffix.lower() in (".usd", ".usda", ".usdc", ".usdz"):
            xf = UsdGeom.Xform.Define(self.stage, path)
            xf.GetPrim().GetReferences().AddReference(str(src))
            return xf.GetPrim()

        cached = self._mesh_cache.get(src)
        if cached is not None and self.args.instance_meshes:
            xf = UsdGeom.Xform.Define(self.stage, path)
            xf.GetPrim().GetReferences().AddInternalReference(Sdf.Path(cached))
            return xf.GetPrim()

        try:
            import trimesh
        except ImportError:
            sys.exit("this world uses Mesh{url}; install trimesh (and pycollada for .dae)")

        self.report_dae_up_axis(src)
        tm = trimesh.load(src, force="mesh", process=False)
        if tm is None or len(getattr(tm, "faces", [])) == 0:
            print(f"[!] empty mesh: {src.name}")
            return None

        mesh = UsdGeom.Mesh.Define(self.stage, path)
        mesh.CreatePointsAttr([Gf.Vec3f(*map(float, p)) for p in tm.vertices])
        mesh.CreateFaceVertexCountsAttr([3] * len(tm.faces))
        mesh.CreateFaceVertexIndicesAttr([int(i) for f in tm.faces for i in f])
        mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        lo, hi = tm.bounds
        mesh.CreateExtentAttr([Gf.Vec3f(*map(float, lo)), Gf.Vec3f(*map(float, hi))])

        # UVs: without these the PBR maps below have nothing to sample against.
        uv = getattr(getattr(tm, "visual", None), "uv", None)
        if uv is not None and len(uv) == len(tm.vertices):
            pv = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
                "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex
            )
            pv.Set([Gf.Vec2f(float(u), float(v)) for u, v in uv])
        else:
            print(f"[!] {src.name} has no UVs; texture maps will not display")

        n = getattr(tm, "vertex_normals", None)
        if n is not None and len(n) == len(tm.vertices):
            mesh.CreateNormalsAttr([Gf.Vec3f(*map(float, v)) for v in n])
            mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)

        self._mesh_cache[src] = path
        return mesh.GetPrim()

    # -- materials --------------------------------------------------------- #
    # Webots map name -> (UsdPreviewSurface input, output channel, value type)
    TEXTURE_MAPS = {
        "baseColorMap":   ("diffuseColor", "rgb", Sdf.ValueTypeNames.Float3),
        "roughnessMap":   ("roughness",    "r",   Sdf.ValueTypeNames.Float),
        "metalnessMap":   ("metallic",     "r",   Sdf.ValueTypeNames.Float),
        "normalMap":      ("normal",       "rgb", Sdf.ValueTypeNames.Float3),
        "occlusionMap":   ("occlusion",    "r",   Sdf.ValueTypeNames.Float),
        "emissiveColorMap": ("emissiveColor", "rgb", Sdf.ValueTypeNames.Float3),
    }

    def texture_path(self, tex: Node) -> str | None:
        src = self.resolve_url(tex)
        if src is None:
            return None
        if not src.exists():
            self.missing_textures.add(str(src))
            return None
        try:
            return os.path.relpath(src, self.out_dir)
        except ValueError:
            return str(src)

    def make_material(self, app: Node) -> UsdShade.Material | None:
        if app.type == "PBRAppearance":
            base = as_vec3(app.get("baseColor"), (1, 1, 1))
            rough = as_float(app.get("roughness"), 0.5)
            metal = as_float(app.get("metalness"), 1.0)
            emis = as_vec3(app.get("emissiveColor"), (0, 0, 0))
            emis *= as_float(app.get("emissiveIntensity"), 1.0)
            trans = as_float(app.get("transparency"), 0.0)
            maps = {k: v for k, v in app.fields.items()
                    if k in self.TEXTURE_MAPS and isinstance(v, Node)}
        elif app.type == "Appearance":
            m = app.get("material")
            base = as_vec3(m.get("diffuseColor"), (0.8, 0.8, 0.8)) if isinstance(m, Node) else Gf.Vec3d(0.8, 0.8, 0.8)
            emis = as_vec3(m.get("emissiveColor"), (0, 0, 0)) if isinstance(m, Node) else Gf.Vec3d(0, 0, 0)
            shine = as_float(m.get("shininess"), 0.2) if isinstance(m, Node) else 0.2
            rough, metal = 1.0 - shine, 0.0
            trans = as_float(m.get("transparency"), 0.0) if isinstance(m, Node) else 0.0
            tex = app.get("texture")
            maps = {"baseColorMap": tex} if isinstance(tex, Node) else {}
        else:
            return None

        resolved = {k: self.texture_path(v) for k, v in maps.items()}
        resolved = {k: v for k, v in resolved.items() if v}

        # The Webots appearance name is part of the identity: two appearances with
        # identical maps but different names stay distinct, so the names survive
        # into USD and remain usable as scene-graph labels.
        aname = app.get("name") if isinstance(app.get("name"), str) else None
        key = (app.type, aname, tuple(base), round(rough, 4), round(metal, 4),
               round(trans, 4), tuple(emis), tuple(sorted(resolved.items())))
        if key in self.materials:
            return self.materials[key]

        scope = "/World/Looks"
        if not self.stage.GetPrimAtPath(scope):
            UsdGeom.Scope.Define(self.stage, scope)
        mpath = self.unique(scope, aname or f"Mat_{len(self.materials):03d}")

        mat = UsdShade.Material.Define(self.stage, mpath)
        sh = UsdShade.Shader.Define(self.stage, f"{mpath}/Surface")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(base))
        sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(emis))
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
        sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metal)
        sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(1.0 - trans)

        if resolved:
            st = UsdShade.Shader.Define(self.stage, f"{mpath}/stReader")
            st.CreateIdAttr("UsdPrimvarReader_float2")
            st.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
            st_out = st.CreateOutput("result", Sdf.ValueTypeNames.Float2)

            for wname, relpath in resolved.items():
                inp_name, chan, vtype = self.TEXTURE_MAPS[wname]
                tex = UsdShade.Shader.Define(self.stage, f"{mpath}/{wname}")
                tex.CreateIdAttr("UsdUVTexture")
                tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(relpath)
                tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(st_out)
                tex.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat")
                tex.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("repeat")
                if wname in ("baseColorMap", "emissiveColorMap"):
                    tex.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
                else:
                    tex.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("raw")
                if wname == "normalMap":
                    tex.CreateInput("scale", Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(2, 2, 2, 1))
                    tex.CreateInput("bias", Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(-1, -1, -1, 0))
                if wname == "metalnessMap" and "arm" in Path(relpath).name.lower():
                    # ARM packing = ambient/roughness/metallic; metal lives in blue.
                    chan = "b"
                    self.notes.add(f"{Path(relpath).name}: assumed ARM packing, "
                                   "reading metalness from blue channel")
                out = tex.CreateOutput(chan, vtype)
                sh.CreateInput(inp_name, Sdf.ValueTypeNames.Normal3f
                               if inp_name == "normal" else
                               (Sdf.ValueTypeNames.Color3f if chan == "rgb"
                                else Sdf.ValueTypeNames.Float)).ConnectToSource(out)

        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        self.materials[key] = mat
        self.bump("material")
        return mat

    # -- lights ------------------------------------------------------------ #
    def emit_light(self, node: Node, parent: str) -> None:
        path = self.unique(parent, node.type)
        inten = as_float(node.get("intensity"), 1.0)
        color = as_vec3(node.get("color"), (1, 1, 1))
        if node.type == "DirectionalLight":
            lt = UsdLux.DistantLight.Define(self.stage, path)
            d = as_vec3(node.get("direction"), (0, 0, -1))
            if d.GetLength() > 1e-9:
                q = Gf.Rotation(Gf.Vec3d(0, 0, -1), d.GetNormalized()).GetQuat()
                lt.AddOrientOp().Set(Gf.Quatf(float(q.GetReal()), Gf.Vec3f(*q.GetImaginary())))
            lt.CreateIntensityAttr(inten * 1000.0)
        elif node.type == "PointLight":
            lt = UsdLux.SphereLight.Define(self.stage, path)
            lt.CreateRadiusAttr(0.05)
            lt.CreateTreatAsPointAttr(True)
            lt.AddTranslateOp().Set(as_vec3(node.get("location")))
            lt.CreateIntensityAttr(inten * 1000.0)
        else:
            lt = UsdLux.DiskLight.Define(self.stage, path)
            lt.AddTranslateOp().Set(as_vec3(node.get("location")))
            lt.CreateIntensityAttr(inten * 1000.0)
            UsdLux.ShapingAPI.Apply(lt.GetPrim()).CreateShapingConeAngleAttr(
                math.degrees(as_float(node.get("cutOffAngle"), 0.8))
            )
        lt.CreateColorAttr(Gf.Vec3f(color))
        self.bump(node.type)

    # -- joints ------------------------------------------------------------ #
    def emit_joint(self, node: Node, parent: str) -> None:
        ep = node.get("endPoint")
        if not isinstance(ep, Node):
            return
        before = set(self.stage.Traverse())
        self.emit(ep, parent)
        new = [p for p in self.stage.Traverse()
               if p not in before and p.GetParent().GetPath() == Sdf.Path(parent)]
        if not new:
            return
        if not self.args.no_physics:
            self.joints.append((node, parent, str(new[0].GetPath())))

    def make_joint(self, node: Node, parent: str, child: str) -> None:
        jp = node.get("jointParameters")
        axis = as_vec3(jp.get("axis") if isinstance(jp, Node) else None, (1, 0, 0))
        anchor = as_vec3(jp.get("anchor") if isinstance(jp, Node) else None)
        scope = f"{parent}/Joints"
        if not self.stage.GetPrimAtPath(scope):
            UsdGeom.Scope.Define(self.stage, scope)
        name = Tf.MakeValidIdentifier(Path(child).name + "_joint")
        jpath = self.unique(scope, name)

        if node.type == "HingeJoint":
            j = UsdPhysics.RevoluteJoint.Define(self.stage, jpath)
        elif node.type == "SliderJoint":
            j = UsdPhysics.PrismaticJoint.Define(self.stage, jpath)
        else:
            print(f"[!] {node.type} unsupported; emitting fixed joint")
            j = UsdPhysics.FixedJoint.Define(self.stage, jpath)

        j.CreateBody0Rel().SetTargets([parent])
        j.CreateBody1Rel().SetTargets([child])
        j.CreateLocalPos0Attr(Gf.Vec3f(anchor))
        j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        if node.type in ("HingeJoint", "SliderJoint"):
            j.CreateAxisAttr("X")
            q = align_x_to(axis)
            j.CreateLocalRot0Attr(q)
            j.CreateLocalRot1Attr(q)
            lo = as_float(jp.get("minStop") if isinstance(jp, Node) else None, 0.0)
            hi = as_float(jp.get("maxStop") if isinstance(jp, Node) else None, 0.0)
            if lo != hi:
                conv = math.degrees if node.type == "HingeJoint" else (lambda x: x)
                j.CreateLowerLimitAttr(conv(lo))
                j.CreateUpperLimitAttr(conv(hi))
        self.bump(node.type)


# --------------------------------------------------------------------------- #
#  main
# --------------------------------------------------------------------------- #


def main() -> int:
    ap = argparse.ArgumentParser(description="Convert a Webots .wbt world to USD.")
    ap.add_argument("wbt", type=Path)
    ap.add_argument("-o", "--output", type=Path, default=None)
    ap.add_argument("--webots-home", type=Path,
                    default=Path(os.environ["WEBOTS_HOME"]) if "WEBOTS_HOME" in os.environ else None)
    ap.add_argument("--proto-path", action="append", default=[],
                    help="extra directory to search for .proto files (repeatable)")
    ap.add_argument("--skip-robots", action="store_true",
                    help="omit Robot nodes (import them separately as URDF)")
    ap.add_argument("--static-colliders", action="store_true",
                    help="apply CollisionAPI to visual geometry lacking a boundingObject")
    ap.add_argument("--no-physics", action="store_true")
    ap.add_argument("--shell-only", action="store_true",
                    help="drop PROTO instances that cannot be expanded, silently")
    ap.add_argument("--only", metavar="REGEX", default=None,
                    help="keep only Solids whose name matches REGEX (at any depth; a Solid "
                         r"is kept if a descendant matches), e.g. --only '^(Wall|Floor)$'")
    ap.add_argument("--prune-empty", action=argparse.BooleanOptionalAction, default=True,
                    help="remove childless Xform/Scope prims left by filtering")
    ap.add_argument("--instance-meshes", action="store_true",
                    help="reference a repeated mesh instead of duplicating its points")
    ap.add_argument("--dynamic-mesh-approx", default="convexDecomposition",
                    choices=["convexHull", "convexDecomposition", "boundingCube",
                             "boundingSphere", "none"],
                    help="PhysX approximation for mesh colliders on dynamic bodies")
    ap.add_argument("--wall-base-at-origin", action=argparse.BooleanOptionalAction, default=True,
                    help="Webots Wall boxes rest on the floor rather than centring on "
                         "the origin (default: true)")
    ap.add_argument("--list-procedural", action="store_true",
                    help="only report which PROTOs need the Webots template engine")
    a = ap.parse_args()

    if not a.wbt.exists():
        sys.exit(f"no such file: {a.wbt}")
    out = a.output or a.wbt.with_suffix(".usda")

    roots = Parser(tokenize(a.wbt.read_text(encoding="utf-8", errors="replace"))).parse_top()
    protos = ProtoLibrary(a.wbt.resolve(), a.webots_home, a.proto_path)

    stage = Usd.Stage.CreateNew(str(out))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    conv = Converter(stage, protos, a, out.resolve().parent)
    conv.run(roots)

    if not a.no_physics:
        UsdPhysics.Scene.Define(stage, "/World/PhysicsScene").CreateGravityMagnitudeAttr(9.81)

    stage.GetRootLayer().Save()

    print(f"\nwrote {out}")
    for k in sorted(conv.stats):
        print(f"  {k:28s} {conv.stats[k]}")
    for n in sorted(conv.notes):
        print(f"\n[i] {n}")
    if conv.missing_meshes:
        print("\n[!] meshes not found (geometry omitted):")
        for n in sorted(conv.missing_meshes):
            print(f"      {n}")
    if conv.missing_textures:
        print(f"\n[!] {len(conv.missing_textures)} textures not found; "
              "materials fall back to flat colour")
    if protos.procedural:
        print("\n[!] procedural PROTOs (JS-templated, not expanded):")
        for n in sorted(protos.procedural):
            print(f"      {n}")
    if protos.missing:
        print("\n[!] PROTOs not found on disk:")
        for n in sorted(protos.missing):
            print(f"      {n}")
    if conv.placeholders:
        print("\n[!] empty placeholder Xforms (pose is correct, geometry is missing):")
        for p in conv.placeholders:
            print(f"      {p}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
