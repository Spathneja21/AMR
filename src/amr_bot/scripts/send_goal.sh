#!/usr/bin/env bash
# usage: send_goal.sh X Y YAW_RAD        goal in the map frame
set -e
[ $# -eq 3 ] || { echo "usage: $0 x y yaw_rad"; exit 1; }
read -r QZ QW < <(python3 -c "import math,sys; h=float(sys.argv[1])/2; print(math.sin(h), math.cos(h))" "$3")
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: $1, y: $2, z: 0.0}, orientation: {z: $QZ, w: $QW}}}"
