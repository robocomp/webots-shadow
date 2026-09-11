#!/usr/bin/env python3
"""Build meshes/floor_tworooms.dae -- the piso floor with room 2 welded onto it.

tworooms-piso.wbt adds a rectangular room north of DOOR_1.  The apartment floor, meshes/floor.dae,
is exactly the air polygon of piso.wbt and stops dead at the north wall's INNER face, so on its own
it leaves the doorway and the whole of room 2 with no ground under them.

Butting a separate room-2 slab against it does not work cleanly: the apartment's north wall is a
degree off-axis (y = 9.256 at its west end, 9.231 at its east), so a straight-edged slab leaves an
open wedge up to 25 mm wide right across the threshold, and closing that by overlapping the two
puts two surfaces at z = 0 in the same place, which z-fights in the one patch of floor a crossing
run stares at.  Cutting room 2's slab against the apartment polygon removes the seam instead of
hiding it: the addition's south border IS the apartment's north border, same vertices, so the two
meet edge to edge with nothing coincident and nothing open.

The two pieces are triangulated SEPARATELY and the triangle lists concatenated, rather than ear
clipping their union in one go.  earclip.triangulate is exact on each piece (22 tris / 60.913 m2
and 6 tris / 27.233 m2, zero uncovered and zero spill), but on the merged 28-gon it emits n-2
triangles that overlap by 16 m2 and spill 2 m2 into the thin wall slots -- the notches for the
double-sheet partitions defeat its "is any remaining vertex strictly inside this ear" test.  Two
clean triangulations of the same plane butted along a shared edge is the same surface and is
provably right here, so there is no reason to go fix a general-purpose ear clipper for it.

This leaves two T-junctions, where the addition's corners at y = 9.20 land mid-edge on the
apartment's corridor walls.  Both sit at a wall base, the surfaces are exactly coplanar with one
shared normal, and the mismatch is at float precision, so nothing cracks.

Written to a NEW file on purpose.  meshes/ is shared with piso.wbt, and piso.wbt's Floor node
points at floor.dae; extending that in place would hang a 6 x 4 m slab of floor in mid-air off the
north wall of the single-room world.

UVs are just world (x, y) -- that is what floor.dae does (its Floor-map array is a verbatim copy of
its Floor-positions xy), giving one texture tile per metre.  Copying the convention keeps the
laminate grain continuous across the doorway instead of restarting in room 2.
"""
import numpy as np
from shapely.geometry import Polygon
from shapely.ops import unary_union
from earclip import triangulate

OUT = "meshes/floor_tworooms.dae"

# The apartment's inner outline, identical to LOOP in thicken_walls.py (and hence to floor.dae).
LOOP = [(-0.000, 6.892), (-0.215, 0.000), (-3.236, 0.078), (-3.232, 0.792), (-4.222, 0.820),
        (-4.184, 2.506), (-4.329, 2.497), (-4.331, 0.814), (-5.207, 0.822), (-5.184, 0.068),
        (-8.262, 0.178), (-8.508, 0.437), (-8.392, 7.090), (-5.932, 7.102), (-5.920, 7.163),
        (-6.336, 7.169), (-6.330, 8.026), (-5.884, 8.007), (-5.871, 9.256), (-4.196, 9.231),
        (-4.296, 5.022), (-4.212, 5.012), (-4.172, 7.166), (-0.222, 7.166)]

# Room 2's slab, out to the OUTER faces of its walls so they have something to stand on, and run
# south to y = 9.20 so it overlaps the apartment polygon by ~3 cm across the full corridor width.
# The overlap is then cut away against the apartment, which is what leaves the addition's south
# border sitting exactly on the apartment's, tilt and all, instead of near it.
ROOM2 = [(-7.88, 9.20), (-1.66, 9.20), (-1.66, 13.59), (-7.88, 13.59)]


def collada(tris, path):
    """Write a flat z=0 triangle soup as COLLADA, byte-for-byte in floor.dae's dialect."""
    verts, index = [], {}
    idx = []
    for t in tris:
        for p in t:
            k = (round(p[0], 9), round(p[1], 9))
            if k not in index:
                index[k] = len(verts)
                verts.append(k)
            idx.append(index[k])
    pos = " ".join("%.6f %.6f 0.000000" % v for v in verts)
    uv = " ".join("%.6f %.6f" % v for v in verts)
    # VERTEX / NORMAL / TEXCOORD, offsets 0 / 1 / 2, one shared normal -- floor.dae's layout.
    p = " ".join("%d 0 %d" % (i, i) for i in idx)
    xml = f"""<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset>
    <contributor><authoring_tool>robocomp apartamento mesh generator</authoring_tool></contributor>
    <created>2026-07-14T00:00:00</created>
    <modified>2026-07-14T00:00:00</modified>
    <unit name="meter" meter="1"/>
    <up_axis>Z_UP</up_axis>
  </asset>
  <library_materials>
    <material id="LaminateFloor-material" name="LaminateFloor">
      <instance_effect url="#LaminateFloor-effect"/>
    </material>
  </library_materials>
  <library_effects>
    <effect id="LaminateFloor-effect"><profile_COMMON><technique sid="common">
      <lambert><diffuse><color>0.8 0.8 0.8 1</color></diffuse></lambert>
    </technique></profile_COMMON></effect>
  </library_effects>
  <library_geometries>
    <geometry id="Floor-mesh" name="Floor"><mesh>
      <source id="Floor-positions">
        <float_array id="Floor-positions-array" count="{len(verts)*3}">{pos}</float_array>
        <technique_common><accessor source="#Floor-positions-array" count="{len(verts)}" stride="3">
          <param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/>
        </accessor></technique_common>
      </source>
      <source id="Floor-normals">
        <float_array id="Floor-normals-array" count="3">0 0 1</float_array>
        <technique_common><accessor source="#Floor-normals-array" count="1" stride="3">
          <param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/>
        </accessor></technique_common>
      </source>
      <source id="Floor-map">
        <float_array id="Floor-map-array" count="{len(verts)*2}">{uv}</float_array>
        <technique_common><accessor source="#Floor-map-array" count="{len(verts)}" stride="2">
          <param name="S" type="float"/><param name="T" type="float"/>
        </accessor></technique_common>
      </source>
      <vertices id="Floor-vertices"><input semantic="POSITION" source="#Floor-positions"/></vertices>
      <triangles material="LaminateFloor-material" count="{len(tris)}">
        <input semantic="VERTEX" source="#Floor-vertices" offset="0"/>
        <input semantic="NORMAL" source="#Floor-normals" offset="1"/>
        <input semantic="TEXCOORD" source="#Floor-map" offset="2" set="0"/>
        <p>{p}</p>
      </triangles>
    </mesh></geometry>
  </library_geometries>
  <library_visual_scenes><visual_scene id="Scene" name="Scene">
    <node id="Floor" name="Floor" type="NODE">
      <matrix sid="transform">1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1</matrix>
      <instance_geometry url="#Floor-mesh" name="Floor">
        <bind_material><technique_common>
          <instance_material symbol="LaminateFloor-material" target="#LaminateFloor-material"/>
        </technique_common></bind_material>
      </instance_geometry>
    </node>
  </visual_scene></library_visual_scenes>
  <scene><instance_visual_scene url="#Scene"/></scene>
</COLLADA>
"""
    open(path, "w").write(xml)


def exact(poly, what):
    """Ear clip one piece and refuse anything that is not an exact cover of it."""
    tris = triangulate(list(poly.exterior.coords)[:-1])
    assert not poly.interiors, f"{what} has a hole"
    cover = unary_union([Polygon(t) for t in tris])
    assert cover.difference(poly).area < 1e-9, f"{what}: triangles spill outside"
    assert poly.difference(cover).area < 1e-9, f"{what}: triangles leave a hole"
    summed = sum(abs(np.cross(np.subtract(t[1], t[0]), np.subtract(t[2], t[0]))) / 2 for t in tris)
    assert abs(summed - poly.area) < 1e-9, f"{what}: triangles overlap"   # cover alone misses this
    return tris


def main():
    air = Polygon(LOOP)
    add = Polygon(ROOM2).difference(air)
    assert add.geom_type == "Polygon", add.geom_type

    tris = exact(air, "apartment") + exact(add, "room 2")

    collada(tris, OUT)
    whole = unary_union([air, add])
    print(f"{OUT}: {len(tris)} tris, {whole.area:.3f} m2 "
          f"(apartment {air.area:.3f} + room 2 {add.area:.3f}), "
          f"bounds {tuple(round(v, 3) for v in whole.bounds)}")


if __name__ == "__main__":
    main()
