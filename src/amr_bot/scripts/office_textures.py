#!/usr/bin/env python3
"""Convert the office JPG textures to PNG (MuJoCo cannot read JPG) and shrink them to at most --max px.

usage (needs Pillow, so run it in the container): office_textures.py [--src DIR] [--dst DIR] [--max 1024]
"""
import argparse
import os

from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
# DAE image file -> short name used by office_visuals.py
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


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--src", default=os.path.join(HERE, "..", "models", "office", "source", "textures"))
    ap.add_argument("--dst", default=os.path.join(HERE, "..", "models", "office", "textures"))
    ap.add_argument("--max", type=int, default=1024, help="longest side after shrinking [px]")
    a = ap.parse_args()
    os.makedirs(a.dst, exist_ok=True)
    for fn, name in NAMES.items():
        path = os.path.join(a.src, fn)
        if not os.path.exists(path):
            print(f"missing: {path}")
            continue
        im = Image.open(path).convert("RGB")
        w, h = im.size
        if max(w, h) > a.max:
            k = a.max / max(w, h)
            im = im.resize((max(1, round(w * k)), max(1, round(h * k))), Image.LANCZOS)
        out = os.path.join(a.dst, name + ".png")
        im.save(out, optimize=True)
        print(f"{fn[:44]:44s} {w}x{h} -> {im.size[0]}x{im.size[1]}  {os.path.getsize(out) / 1024:7.0f} KB  {name}.png")


if __name__ == "__main__":
    main()
