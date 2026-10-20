#!/usr/bin/env python3
"""Convert the Clearpath office COLLADA mesh into a MuJoCo world fragment made of boxes.

usage: office_to_mjcf.py [--dae SRC] [--out DST] [--z 0.15] [--wall-t 0.05] [--no-fence] [--invisible]

Slices every part of the mesh at height --z (the lidar height), merges the wall lines found there
into oriented boxes, and writes DST (office_world.xml) for <include> in a scene file. MuJoCo
collides meshes as their convex hull, so a walls mesh cannot be used directly: boxes are exact
for walls and also work for the lidar. Needs only numpy.
"""
import argparse
import math
import os
import xml.etree.ElementTree as ET
from collections import Counter

import numpy as np

NS = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
HERE = os.path.dirname(os.path.abspath(__file__))


def load_dae(path):
    root = ET.parse(path).getroot()
    geoms = {g.attrib["id"]: g for g in root.findall(".//c:library_geometries/c:geometry", NS)}
    objs = []
    for nd in root.findall(".//c:library_visual_scenes//c:node", NS):
        ig = nd.find("c:instance_geometry", NS)
        if ig is None:
            continue
        m = nd.find("c:matrix", NS)
        M = np.array(m.text.split(), float).reshape(4, 4) if m is not None else np.eye(4)
        mesh = geoms[ig.attrib["url"][1:]].find("c:mesh", NS)
        srcs = {s.attrib["id"]: s for s in mesh.findall("c:source", NS)}
        pos_id = [i for i in mesh.find("c:vertices", NS).findall("c:input", NS)
                  if i.attrib["semantic"] == "POSITION"][0].attrib["source"][1:]
        P = np.array(srcs[pos_id].find("c:float_array", NS).text.split(), float).reshape(-1, 3)
        W = (M @ np.c_[P, np.ones(len(P))].T).T[:, :3]            # vertices in world coordinates
        tris = []
        for tag in ("triangles", "polylist"):
            for t in mesh.findall(f"c:{tag}", NS):
                ins = t.findall("c:input", NS)
                stride = max(int(i.attrib["offset"]) for i in ins) + 1
                voff = [int(i.attrib["offset"]) for i in ins if i.attrib["semantic"] == "VERTEX"][0]
                p = np.array(t.find("c:p", NS).text.split(), int).reshape(-1, stride)[:, voff]
                if tag == "triangles":
                    tri = p.reshape(-1, 3)
                else:                                              # fan-triangulate polygons
                    vc = np.array(t.find("c:vcount", NS).text.split(), int)
                    k, tri = 0, []
                    for n in vc:
                        tri += [(p[k], p[k + j], p[k + j + 1]) for j in range(1, n - 1)]
                        k += n
                    tri = np.array(tri)
                tris.append((t.attrib.get("material", ""), tri))
        objs.append(dict(name=nd.attrib["name"], W=W, tris=tris))
    return objs


def slice_segments(objs, z0):
    """Intersect every triangle with the plane z = z0 -> list of (x1, y1, x2, y2, top_z, material)."""
    segs = []
    for o in objs:
        W = o["W"]
        for mat, tri in o["tris"]:
            for a, b, c in zip(W[tri[:, 0]], W[tri[:, 1]], W[tri[:, 2]]):
                zs = (a[2], b[2], c[2])
                if not min(zs) < z0 < max(zs):
                    continue
                pts = []
                for p, q in ((a, b), (b, c), (c, a)):
                    if (p[2] - z0) * (q[2] - z0) < 0:
                        s = (z0 - p[2]) / (q[2] - p[2])
                        pts.append(p[:2] + s * (q[:2] - p[:2]))
                if len(pts) == 2 and np.linalg.norm(pts[0] - pts[1]) > 1e-4:
                    segs.append((pts[0][0], pts[0][1], pts[1][0], pts[1][1], max(zs), mat))
    return segs


def cluster(vals, tol):
    """Single-linkage 1-D clustering: values closer than tol share a label."""
    order = np.argsort(vals)
    lab = np.zeros(len(vals), int)
    k = 0
    for i in range(1, len(order)):
        if vals[order[i]] - vals[order[i - 1]] > tol:
            k += 1
        lab[order[i]] = k
    return lab


def merge_segments(segs, join_gap=0.02, ang_tol=math.radians(0.3), off_tol=0.01):
    """Merge collinear, touching segments -> list of (theta, offset, t_min, t_max, top_z, material)."""
    S = np.array([s[:4] for s in segs], float)
    top = np.array([s[4] for s in segs])
    mat = np.array([s[5] for s in segs])
    d = S[:, 2:4] - S[:, 0:2]
    flip = (d[:, 0] < -1e-4) | ((np.abs(d[:, 0]) <= 1e-4) & (d[:, 1] < 0))     # canonical direction
    S[flip] = S[flip][:, [2, 3, 0, 1]]
    d = S[:, 2:4] - S[:, 0:2]
    d[np.abs(d[:, 0]) <= 1e-4, 0] = 0.0                                          # exactly vertical
    th = np.arctan2(d[:, 1], d[:, 0])
    out = []
    for la in np.unique(cluster(th, ang_tol)):
        ia = np.flatnonzero(cluster(th, ang_tol) == la)
        t_mean = th[ia].mean()
        u, n = np.array([math.cos(t_mean), math.sin(t_mean)]), np.array([-math.sin(t_mean), math.cos(t_mean)])
        off = S[ia, 0:2] @ n
        lo = cluster(off, off_tol)
        for lb in np.unique(lo):
            ib = ia[lo == lb]
            t1, t2 = S[ib, 0:2] @ u, S[ib, 2:4] @ u
            a, b = np.minimum(t1, t2), np.maximum(t1, t2)
            order = np.argsort(a)
            cur = None
            for k in order:
                if cur is not None and a[k] - cur[1] <= join_gap:
                    cur[1] = max(cur[1], b[k]); cur[2].append(ib[k])
                else:
                    if cur is not None:
                        out.append((t_mean, off[lo == lb].mean(), cur[0], cur[1], cur[2]))
                    cur = [a[k], b[k], [ib[k]]]
            out.append((t_mean, off[lo == lb].mean(), cur[0], cur[1], cur[2]))
    return [(t, o, a, b, top[ix].max(), Counter(mat[ix]).most_common(1)[0][0]) for t, o, a, b, ix in out]


def colour(material):
    m = material.lower()
    for key, name, rgba in (("cp_walls", "wall", "0.85 0.85 0.82 1"), ("metal", "metal", "0.50 0.52 0.55 1"),
                            ("wood", "wood", "0.45 0.30 0.18 1"), ("jackal_box", "cardboard", "0.72 0.55 0.35 1"),
                            ("tiles", "tiles", "0.55 0.60 0.65 1")):
        if key in m:
            return name, rgba
    return "other", "0.70 0.70 0.70 1"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dae", default=os.path.join(HERE, "..", "models", "office", "source", "office.dae"))
    ap.add_argument("--out", default=os.path.join(HERE, "..", "models", "office", "office_world.xml"))
    ap.add_argument("--z", type=float, default=0.15, help="slice height [m]; the lidar sits at 0.15")
    ap.add_argument("--wall-t", type=float, default=0.05, help="thickness given to every wall line [m]")
    ap.add_argument("--no-fence", action="store_true", help="do not add the boundary fence around the floor")
    ap.add_argument("--invisible", action="store_true",
                    help="draw no boxes (alpha 0) and let rays pass through them: collision only. Use with office_visuals.py")
    a = ap.parse_args()

    objs = load_dae(a.dae)
    floor = [o for o in objs if np.ptp(o["W"][:, 2]) < 1e-3 and np.ptp(o["W"][:, 0]) * np.ptp(o["W"][:, 1]) > 100]
    body = [o for o in objs if o not in floor]
    lo = np.min([o["W"].min(0) for o in objs], 0)
    hi = np.max([o["W"].max(0) for o in objs], 0)
    fl = max(floor, key=lambda o: np.ptp(o["W"][:, 0]) * np.ptp(o["W"][:, 1]))["W"] if floor else np.array([lo, hi])  # the largest flat part
    (x0, y0), (x1, y1) = fl[:, :2].min(0), fl[:, :2].max(0)

    segs = slice_segments(body, a.z)
    walls = [w for w in merge_segments(segs) if w[3] - w[2] >= 0.03]

    mats = {}
    geoms = []
    for i, (t, off, tmin, tmax, ztop, mat) in enumerate(walls):
        u, n = np.array([math.cos(t), math.sin(t)]), np.array([-math.sin(t), math.cos(t)])
        tc, half = (tmin + tmax) / 2, (tmax - tmin) / 2 + a.wall_t / 2
        c = off * n + tc * u
        cname, rgba = colour(mat)
        if a.invisible:
            cname, rgba = "collision", "0 0 0 0"
        mats[cname] = rgba
        geoms.append(f'    <geom name="office_wall_{i:03d}" type="box" pos="{c[0]:.4f} {c[1]:.4f} {ztop / 2:.4f}" '
                     f'size="{half:.4f} {a.wall_t / 2:.4f} {ztop / 2:.4f}" '
                     f'quat="{math.cos(t / 2):.6f} 0 0 {math.sin(t / 2):.6f}" material="office_{cname}"/>')

    fence = []
    if not a.no_fence:
        th, h = 0.10, 1.25
        for nm, (cx, cy, sx, sy) in dict(n=((x0 + x1) / 2, y1 + th / 2, (x1 - x0) / 2 + th, th / 2),
                                         s=((x0 + x1) / 2, y0 - th / 2, (x1 - x0) / 2 + th, th / 2),
                                         e=(x1 + th / 2, (y0 + y1) / 2, th / 2, (y1 - y0) / 2),
                                         w=(x0 - th / 2, (y0 + y1) / 2, th / 2, (y1 - y0) / 2)).items():
            fence.append(f'    <geom name="office_fence_{nm}" type="box" pos="{cx:.4f} {cy:.4f} {h:.4f}" '
                         f'size="{sx:.4f} {sy:.4f} {h:.4f}" material="office_wall"/>')
        mats.setdefault("wall", "0.85 0.85 0.82 1")

    floor_mat = "office_collision" if a.invisible else "office_floor"      # the visual floor mesh is drawn instead
    mats.setdefault("collision", "0 0 0 0") if a.invisible else None
    asset = ['    <material name="office_floor" rgba="0.38 0.25 0.16 1"/>']
    asset += [f'    <material name="office_{k}" rgba="{v}"/>' for k, v in sorted(mats.items())]
    if a.invisible:                  # ground under the holes of the visual floor mesh; no collision (the plane above does that)
        asset.append('    <material name="office_ground" rgba="0.55 0.54 0.51 1"/>')
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    xml = ['<mujoco model="office_world">',
           '  <!-- Generated by scripts/office_to_mjcf.py from source/office.dae (Clearpath Robotics, BSD, see',
           '       source/LICENSE.txt). World only: no robot, no <compiler>/<option>. Do not edit by hand;',
           f'       regenerate with the script. Walls are boxes cut from the mesh at z = {a.z} m. -->',
           '  <asset>', *asset, '  </asset>', '  <worldbody>',
           f'    <light name="office_top" pos="{cx:.2f} {cy:.2f} 12" dir="0 0 -1" directional="true" diffuse="0.9 0.9 0.9" specular="0.1 0.1 0.1"/>',
           f'    <light name="office_fill" pos="{cx - 8:.2f} {cy - 6:.2f} 8" dir="0.4 0.3 -1" diffuse="0.4 0.4 0.4"/>',
           f'    <geom name="office_floor" type="plane" pos="{cx:.4f} {cy:.4f} 0" size="{(x1 - x0) / 2:.4f} {(y1 - y0) / 2:.4f} 0.1" material="{floor_mat}"/>',
           *([f'    <geom name="office_ground" type="plane" pos="{cx:.4f} {cy:.4f} -0.003" size="{(x1 - x0) / 2:.4f} {(y1 - y0) / 2:.4f} 0.1" '
              'material="office_ground" contype="0" conaffinity="0"/>'] if a.invisible else []),
           *geoms, *fence, '  </worldbody>', '</mujoco>']
    with open(a.out, "w") as f:
        f.write("\n".join(xml) + "\n")
    dist = []
    for t, off, tmin, tmax, *_ in walls:                          # clearance from the origin to each wall line
        u, n = np.array([math.cos(t), math.sin(t)]), np.array([-math.sin(t), math.cos(t)])
        p = np.clip(0.0, tmin, tmax)
        dist.append(math.hypot(off * n[0] + p * u[0], off * n[1] + p * u[1]))
    print(f"parts: {len(objs)} ({len(floor)} floor); slice z = {a.z} m; segments cut: {len(segs)} -> merged wall boxes: {len(walls)}")
    print(f"floor x [{x0:.2f}, {x1:.2f}]  y [{y0:.2f}, {y1:.2f}] ({x1 - x0:.1f} x {y1 - y0:.1f} m); fence: {'no' if a.no_fence else 'yes'}")
    print(f"nearest wall to the origin (0, 0): {min(dist):.2f} m")
    print(f"wrote {a.out}")


if __name__ == "__main__":
    main()
