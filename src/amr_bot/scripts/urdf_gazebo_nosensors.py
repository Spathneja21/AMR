#!/usr/bin/env python3
"""Strip the lidar/camera <gazebo reference="..."> sensor blocks out of the robot URDF, for spawning
into Gazebo Sim. Needed because of a confirmed environment bug (PROCESS.md section 30/31): the
gpu_lidar and camera sensors crash the gz-sim server's renderer on this machine (reproduced on both
Fortress and Harmonic, same root cause, not fixable from our side) - the diff-drive + joint-state
plugins (the top-level <gazebo> block, no "reference" attribute) are unaffected and work fine.

Regenerated from the real URDF every time, so it can't drift from the robot's actual geometry.
Usage: python3 urdf_gazebo_nosensors.py > /tmp/supermarketbot_gazebo.urdf
"""
import re
import sys
from pathlib import Path

URDF = Path(__file__).resolve().parent.parent / "urdf" / "supermarketbot.urdf"

s = URDF.read_text()
for link in ("lidar_link", "camera_link"):
    s = re.sub(rf'<gazebo reference="{link}">.*?</gazebo>\n', "", s, flags=re.S)
sys.stdout.write(s)
