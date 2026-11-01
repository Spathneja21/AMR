#!/usr/bin/env python3
"""Export a Gazebo (ign gazebo / gz-sim) model of the office from the data we already generated
for MuJoCo: reuses office_world.xml's box-sliced collision geoms (so doorways stay open, matching
office_scene.xml's behavior) and the original office.dae mesh for the visual (Gazebo, unlike
MuJoCo, loads a single multi-material COLLADA mesh fine, so none of office_visuals.py's per-material
OBJ split is needed here).

Output: models/gazebo/office/ (a Gazebo model: model.config + model.sdf + a flat copy of office.dae
and its textures - COLLADA resolves textures relative to the .dae file's own directory, and the
source ones live one level down in source/textures/, so they're copied flat alongside the mesh).
The model dir's name ("office") must match the "model://office" URI in office.world.sdf, and its
parent (models/gazebo/) is what GZ_SIM_RESOURCE_PATH needs to point at.

Usage: python3 office_to_sdf.py   (run from anywhere; paths are relative to this script)
"""
import re
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

HERE = Path(__file__).resolve().parent
OFFICE = HERE.parent / "models" / "office"
WORLD_XML = OFFICE / "office_world.xml"
OUT = HERE.parent / "models" / "gazebo" / "office"


def quat_to_yaw(qw, qx, qy, qz):
    # Every box here is a pure yaw (Z-axis) rotation (qx = qy = 0 by construction in
    # office_to_mjcf.py), so this is exact, not an approximation of a general quaternion.
    assert abs(qx) < 1e-6 and abs(qy) < 1e-6, "expected a Z-only rotation"
    return 2.0 * __import__("math").atan2(qz, qw)


def boxes_from_mjcf():
    root = ET.parse(WORLD_XML).getroot()
    boxes = []
    for geom in root.iter("geom"):
        if geom.get("type") != "box":
            continue
        x, y, z = (float(v) for v in geom.get("pos").split())
        sx, sy, sz = (float(v) for v in geom.get("size").split())
        qw, qx, qy, qz = (float(v) for v in geom.get("quat", "1 0 0 0").split())
        boxes.append({
            "name": geom.get("name"),
            "pos": (x, y, z),
            "size": (2 * sx, 2 * sy, 2 * sz),  # MuJoCo size = half-extent, SDF box size = full
            "yaw": quat_to_yaw(qw, qx, qy, qz),
            "fence": "fence" in geom.get("name", ""),
        })
    return boxes


def collision_sdf(b):
    x, y, z = b["pos"]
    sx, sy, sz = b["size"]
    return f'''    <collision name="{b["name"]}">
      <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 {b["yaw"]:.6f}</pose>
      <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
    </collision>'''


def visual_box_sdf(b):
    # Only the fence gets a matching visual box (it's not part of office.dae); interior walls
    # are collision-only, same as office_world.xml - the mesh is their visual.
    x, y, z = b["pos"]
    sx, sy, sz = b["size"]
    return f'''    <visual name="{b["name"]}_visual">
      <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 {b["yaw"]:.6f}</pose>
      <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
      <material><ambient>0.85 0.85 0.82 1</ambient><diffuse>0.85 0.85 0.82 1</diffuse></material>
    </visual>'''


def main():
    boxes = boxes_from_mjcf()
    walls = [b for b in boxes if not b["fence"]]
    fence = [b for b in boxes if b["fence"]]
    print(f"{len(walls)} wall boxes, {len(fence)} fence boxes")

    OUT.mkdir(parents=True, exist_ok=True)
    shutil.copy(OFFICE / "source" / "office.dae", OUT / "office.dae")
    tex_dir = OFFICE / "source" / "textures"
    for jpg in tex_dir.glob("*.jpg"):
        shutil.copy(jpg, OUT / jpg.name)
    print(f"copied office.dae + {len(list(tex_dir.glob('*.jpg')))} textures into {OUT}")

    collisions = "\n".join(collision_sdf(b) for b in walls + fence)
    fence_visuals = "\n".join(visual_box_sdf(b) for b in fence)

    sdf = f'''<?xml version="1.0"?>
<sdf version="1.9">
  <model name="office">
    <static>true</static>
    <link name="link">
      <collision name="ground">
        <geometry><plane><normal>0 0 1</normal><size>44 26</size></plane></geometry>
      </collision>
      <visual name="ground_visual">
        <pose>0 0 -0.003 0 0 0</pose>
        <geometry><plane><normal>0 0 1</normal><size>44 26</size></plane></geometry>
        <material><ambient>0.55 0.54 0.51 1</ambient><diffuse>0.55 0.54 0.51 1</diffuse></material>
      </visual>
      <visual name="mesh">
        <geometry><mesh><uri>office.dae</uri></mesh></geometry>
      </visual>
{collisions}
{fence_visuals}
    </link>
  </model>
</sdf>
'''
    (OUT / "model.sdf").write_text(sdf)
    (OUT / "model.config").write_text('''<?xml version="1.0"?>
<model>
  <name>office</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>Office, converted from cpr_office_gazebo/office.dae (Clearpath, BSD) -
    collision is the box-sliced geometry from office_to_mjcf.py, visual is the original mesh.
    Regenerate with scripts/office_to_sdf.py, do not edit model.sdf by hand.</description>
</model>
''')
    print(f"wrote {OUT / 'model.sdf'} ({len(walls) + len(fence)} collision boxes)")


if __name__ == "__main__":
    main()
