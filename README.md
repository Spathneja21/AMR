# AMR

Differential-drive AMR simulated in MuJoCo, ROS 2 Humble, C++. Robot model
(meshes/URDF/world) reused from [`Ros-AMR-Mobile-Robot`](../Ros-AMR-Mobile-Robot),
rebuilt from scratch as a single C++ bridge node (`amr_bot`/`mujoco_bridge`)
instead of the original's Python + Gazebo stack.

## Prerequisites

- Docker, with an X server (`$DISPLAY` set) for RViz / the MuJoCo viewer.
- The `supermarketbot:humble` image (ROS 2 Humble + nav2 + slam_toolbox +
  the `mujoco` pip wheel), built from `../Ros-AMR-Mobile-Robot/dockerfile`.
  Host OS itself doesn't need to be 22.04 — everything runs in the container.

## One-time container setup

```bash
xhost +local:docker
```
X11 permission grant for the container to draw on your host display.
**Does not persist across host logins** — re-run this if RViz/the MuJoCo
window later fails with `No protocol specified`.

```bash
docker run -it --name amr \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ~/projects/AMR_Practice_project:/ros2_ws \
  -v ~/projects/Ros-AMR-Mobile-Robot:/reference:ro \
  --entrypoint bash \
  supermarketbot:humble
```
- `--net=host` — ROS 2 DDS discovery is multicast-based; bridge networks break it.
- `--entrypoint bash` — the image's default entrypoint sources
  `/ros2_ws/install/setup.bash`, which doesn't exist yet on a fresh mount
  and aborts the whole chain. Override it and source manually instead.
- No `--gpus`: everything (RViz, the MuJoCo viewer) runs via Mesa software
  rendering — see the `LIBGL_ALWAYS_SOFTWARE` note below.

Once inside, install the extra dev packages the base image doesn't ship
(**container-local only — repeat this after any `docker rm`/fresh `docker run`,
or bake it into a Dockerfile if that gets old**):
```bash
apt-get update && apt-get install -y libglfw3-dev libgl1-mesa-dev
```

## Recurring session commands

```bash
docker start -ai amr        # resume the container
docker exec -it amr bash    # open an ADDITIONAL shell in the running container
```

Every shell (each `exec`/`start`) needs sourcing before any `ros2`/`colcon` command:
```bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
```

Software rendering is required for the MuJoCo GLFW viewer (no `/dev/dri` GPU
passthrough is configured); export it once per shell that runs `mujoco_bridge`,
or prefix the command:
```bash
export LIBGL_ALWAYS_SOFTWARE=1
```

## Build

```bash
cd /ros2_ws
colcon build --packages-select amr_bot
source install/setup.bash
```
(`--packages-select`, not `--package-select` — easy typo.)

On the host, files created inside the container land owned by `root`; fix
once per session if VSCode/host tools can't edit them:
```bash
sudo chown -R $USER:$USER ~/projects/AMR_Practice_project
```

## Running the full pipeline

Each block below is its own terminal (`docker exec -it amr bash`, both
`source` lines first). Bring them up roughly in this order.

**1. Physics + odom/scan/joint_states/TF bridge** — always first, everything else depends on it:
```bash
ros2 run amr_bot mujoco_bridge
```

**2. URDF → TF** (`base_link` → wheels/lidar/camera):
```bash
ros2 launch amr_bot rsp.launch.py
```

**3. Drive it:**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**4a. Build a map (SLAM):**
```bash
ros2 run slam_toolbox async_slam_toolbox_node \
  --ros-args --params-file /ros2_ws/src/amr_bot/config/async_slam_toolbox_cfg.yaml
```
Save once the map looks complete in RViz:
```bash
mkdir -p /ros2_ws/src/amr_bot/maps
ros2 run nav2_map_server map_saver_cli -f /ros2_ws/src/amr_bot/maps/my_map
```

**4b. OR navigate autonomously on a saved map (nav2):**
```bash
ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=false \
  map:=/ros2_ws/src/amr_bot/maps/my_map.yaml \
  params_file:=/ros2_ws/src/amr_bot/config/nav2_params.yaml
```

**5. Visualize:**
```bash
rviz2
```
- `Fixed Frame`: `map` while SLAM/nav2 is running, `odom` if neither is up yet.
- Displays to add: `LaserScan` on `/scan`, `Map` on `/map`, `RobotModel` on
  `/robot_description` (Description Source: Topic).

**6. Native MuJoCo viewer** (optional — mirrors RViz's `RobotModel`, reads
the exact same live simulation `mujoco_bridge` is stepping): opens
automatically as part of `mujoco_bridge` itself once built with GLFW support
(see step 1) — no separate command. Controls: left-drag orbit, right-drag
pan, scroll zoom, hold shift for the horizontal drag variant.

## Debugging one link in the pipeline

```bash
ros2 node list
ros2 topic list
ros2 topic echo /odom --once
ros2 topic echo /scan --once
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo odom lidar_link
ros2 lifecycle list /controller_server   # nav2 node state, once bringup is up
```

## Notes

- Robot constants: wheel radius 0.035 m, wheel separation 0.205 m, mass 1.4 kg.
- Model path (`world.xml`) is currently hardcoded in `mujoco_bridge.cpp` as
  `/ros2_ws/src/amr_bot/models/world/world.xml` — not yet using
  `ament_index_cpp::get_package_share_directory`.
- See `PROCESS.md` (gitignored, local only) for the full build log, every
  bug hit, and why — useful background before touching this code again.
