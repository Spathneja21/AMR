#!/usr/bin/env python3
"""Export the textured office mesh as MuJoCo visual geometry: one OBJ per material + an include file.

usage: office_visuals.py [--dae SRC] [--out-dir DIR]
Writes  <DIR>/meshes/office_<material>.obj  and  <DIR>/office_visuals.xml  (DIR = models/office).
The textures must already be converted by office_textures.py (PNG in <DIR>/textures/); a material
whose PNG is missing falls back to a flat colour. The meshes are visual only (contype=0): physics
uses the boxes from office_to_mjcf.py, the lidar rays hit these meshes. Needs only numpy.
"""
import argparse
import os
import xml.etree.ElementTree as ET
from collections import defaultdict

import numpy as np

NS = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
HERE = os.path.dirname(os.path.abspath(__file__))
# DAE image file -> short name (the same table as office_textures.py)
NAMES = {
    "InteriorDesignRugStarryNight001_COL_VAR1_3K.jpg.001.jpg": "rug",
    "MetalCorrugatedIronSheet002_METALNESS_3K_METALNESS.jpg.002.jpg": "metal",
    "Tiles05_COL_VAR1_3K.jpg.001.jpg": "tiles_dark",
    "TilesRectangularMirrorGray001_COL_3K.jpg.001.jpg": "tiles_gray",
    "WoodFlooring044_COL_3K.jpg.001.jpg": "wood",
    "cardboard.jpg": "cardboard",
    "cp_walls.jpg": "cp_walls",
    "whiteboard.jpg": "whiteboard",
}
FALLBACK_RGBA = {"rug": "0.25 0.28 0.45 1", "metal": "0.5 0.52 0.55 1", "tiles_dark": "0.35 0.37 0.40 1",
                 "tiles_gray": "0.6 0.62 0.65 1", "wood": "0.40 0.27 0.17 1", "cardboard": "0.72 0.55 0.35 1",
                 "cp_walls": "0.85 0.85 0.82 1", "whiteboard": "0.95 0.95 0.95 1"}


def material_textures(root):
    """material id -> short texture name, following material -> effect -> sampler -> surface -> image."""
    images = {i.attrib["id"]: i.find("c:init_from", NS).text for i in root.findall(".//c:library_images/c:image", NS)}
    eff = {}
    for e in root.findall(".//c:library_effects/c:effect", NS):
        surf, samp = {}, {}
        for npar in e.findall(".//c:newparam", NS):
            s, sm = npar.find("c:surface", NS), npar.find("c:sampler2D", NS)
            if s is not None:
                surf[npar.attrib["sid"]] = s.find("c:init_from", NS).text
            if sm is not None:
                samp[npar.attrib["sid"]] = sm.find("c:source", NS).text
        tex = e.find(".//c:diffuse/c:texture", NS)
        img = None
        if tex is not None:
            t = tex.attrib["texture"]
            img = images.get(surf.get(samp.get(t, t), t), images.get(t, surf.get(t)))
        eff[e.attrib["id"]] = img
    out = {}
    for m in root.findall(".//c:library_materials/c:material", NS):
        img = eff.get(m.find("c:instance_effect", NS).attrib["url"][1:])
        out[m.attrib["id"]] = NAMES.get(img)
    return out


def load_triangles(path):
    """-> {texture name: (N,3,3) world corner positions, (N,3,2) corner UVs} for every triangle in the mesh."""
    root = ET.parse(path).getroot()
    mat_tex = material_textures(root)
    geoms = {g.attrib["id"]: g for g in root.findall(".//c:library_geometries/c:geometry", NS)}
    pos, uvs = defaultdict(list), defaultdict(list)
    for nd in root.findall(".//c:library_visual_scenes//c:node", NS):
        ig = nd.find("c:instance_geometry", NS)
        if ig is None:
            continue
        bind = {im.attrib["symbol"]: im.attrib["target"][1:] for im in nd.findall(".//c:instance_material", NS)}
        m = nd.find("c:matrix", NS)
        M = np.array(m.text.split(), float).reshape(4, 4) if m is not None else np.eye(4)
        mesh = geoms[ig.attrib["url"][1:]].find("c:mesh", NS)
        srcs = {s.attrib["id"]: s for s in mesh.findall("c:source", NS)}
        pos_id = [i for i in mesh.find("c:vertices", NS).findall("c:input", NS)
                  if i.attrib["semantic"] == "POSITION"][0].attrib["source"][1:]
        P = np.array(srcs[pos_id].find("c:float_array", NS).text.split(), float).reshape(-1, 3)
        W = (M @ np.c_[P, np.ones(len(P))].T).T[:, :3]
        for tag in ("triangles", "polylist"):
            for t in mesh.findall(f"c:{tag}", NS):
                ins = t.findall("c:input", NS)
                stride = max(int(i.attrib["offset"]) for i in ins) + 1
                p = np.array(t.find("c:p", NS).text.split(), int).reshape(-1, stride)
                voff = [int(i.attrib["offset"]) for i in ins if i.attrib["semantic"] == "VERTEX"][0]
                tc = [i for i in ins if i.attrib["semantic"] == "TEXCOORD"][0]
                UV = np.array(srcs[tc.attrib["source"][1:]].find("c:float_array", NS).text.split(), float).reshape(-1, 2)
                vi, ti = p[:, voff], p[:, int(tc.attrib["offset"])]
                if tag == "triangles":
                    tri = np.arange(len(vi)).reshape(-1, 3)
                else:                                              # fan-triangulate polygons
                    vc = np.array(t.find("c:vcount", NS).text.split(), int)
                    k, tri = 0, []
                    for n in vc:
                        tri += [(k, k + j, k + j + 1) for j in range(1, n - 1)]
                        k += n
                    tri = np.array(tri)
                name = mat_tex.get(bind.get(t.attrib.get("material"), t.attrib.get("material")))
                if name is None:
                    continue
                pos[name].append(W[vi[tri]])
                uvs[name].append(UV[ti[tri]])
    return {k: (np.concatenate(pos[k]), np.concatenate(uvs[k])) for k in pos}


def write_obj(path, P, UV):
    """Flat-shaded OBJ: shared (position, uv) vertices, one normal per triangle."""
    key = {}
    verts, vts, faces = [], [], []
    for tri_p, tri_u in zip(P, UV):
        idx = []
        for p, u in zip(tri_p, tri_u):
            k = (tuple(np.round(p, 5)), tuple(np.round(u, 5)))
            if k not in key:
                key[k] = len(verts) + 1
                verts.append(p)
                vts.append(u)
            idx.append(key[k])
        faces.append(idx)
    N = np.cross(P[:, 1] - P[:, 0], P[:, 2] - P[:, 0])
    N /= np.linalg.norm(N, axis=1, keepdims=True) + 1e-12
    with open(path, "w") as f:
        f.write("# generated by office_visuals.py from office.dae\n")
        f.writelines(f"v {p[0]:.5f} {p[1]:.5f} {p[2]:.5f}\n" for p in verts)
        f.writelines(f"vt {u[0]:.5f} {u[1]:.5f}\n" for u in vts)
        f.writelines(f"vn {n[0]:.4f} {n[1]:.4f} {n[2]:.4f}\n" for n in N)
        f.writelines(f"f {a}/{a}/{i + 1} {b}/{b}/{i + 1} {c}/{c}/{i + 1}\n" for i, (a, b, c) in enumerate(faces))
    return len(verts), len(faces)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dae", default=os.path.join(HERE, "..", "models", "office", "source", "office.dae"))
    ap.add_argument("--out-dir", default=os.path.join(HERE, "..", "models", "office"))
    a = ap.parse_args()
    os.makedirs(os.path.join(a.out_dir, "meshes"), exist_ok=True)

    data = load_triangles(a.dae)
    assets, geoms = [], []
    for name in sorted(data):
        P, UV = data[name]
        nv, nf = write_obj(os.path.join(a.out_dir, "meshes", f"office_{name}.obj"), P, UV)
        have_png = os.path.exists(os.path.join(a.out_dir, "textures", f"{name}.png"))
        if have_png:
            assets.append(f'    <texture name="office_tex_{name}" type="2d" file="../office/textures/{name}.png"/>')
            assets.append(f'    <material name="office_vis_{name}" texture="office_tex_{name}" specular="0.1" shininess="0.1"/>')
        else:
            assets.append(f'    <material name="office_vis_{name}" rgba="{FALLBACK_RGBA[name]}" specular="0.1" shininess="0.1"/>')
        assets.append(f'    <mesh name="office_mesh_{name}" file="../office/meshes/office_{name}.obj" inertia="shell"/>')
        geoms.append(f'    <geom name="office_vis_{name}" type="mesh" mesh="office_mesh_{name}" material="office_vis_{name}" '
                     f'contype="0" conaffinity="0" group="1"/>')
        print(f"{name:11s} {nf:5d} faces {nv:5d} vertices  texture: {'png' if have_png else 'MISSING -> flat colour'}")
    assets.insert(0, '    <texture type="skybox" builtin="gradient" rgb1="0.6 0.7 0.9" rgb2="1 1 1" width="256" height="256"/>')
    xml = ['<mujoco model="office_visuals">',
           '  <!-- Generated by scripts/office_visuals.py from source/office.dae (Clearpath Robotics, BSD, see',
           '       source/LICENSE.txt). Visual only: contype=0, so nothing collides with these meshes;',
           '       the lidar rays do hit them. Collision comes from office_world.xml. Do not edit by hand. -->',
           '  <visual><headlight ambient="0.40 0.40 0.40" diffuse="0.55 0.55 0.55" specular="0.05 0.05 0.05"/></visual>',
           '  <asset>', *assets, '  </asset>', '  <worldbody>', *geoms, '  </worldbody>', '</mujoco>']
    with open(os.path.join(a.out_dir, "office_visuals.xml"), "w") as f:
        f.write("\n".join(xml) + "\n")
    print("wrote", os.path.join(a.out_dir, "office_visuals.xml"))


if __name__ == "__main__":
    main()
