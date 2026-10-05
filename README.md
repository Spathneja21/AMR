# AMR

Differential-drive AMR simulated in MuJoCo, ROS 2 Humble, C++. Robot model
(meshes/URDF/world) reused from [`Ros-AMR-Mobile-Robot`](../Ros-AMR-Mobile-Robot),
rebuilt from scratch in C++ instead of the original's Python + Gazebo stack.
The `amr_bot` package has three executables: `mujoco_bridge` (physics + ROS bridge),
`obstacle_avoider` (lidar emergency stop) and `path_logger` (planned-vs-actual path
logger, work in progress).

## Prerequisites

- Docker, with an X server (`$DISPLAY` set) for RViz / the MuJoCo viewer.
- The `supermarketbot:humble` image (ROS 2 Humble + nav2 + slam_toolbox +
  the `mujoco` pip wheel), built from `../Ros-AMR-Mobile-Robot/dockerfile`.
  Host OS itself doesn't need to be 22.04 — everything runs in the container.

## One-time container setup

```bash
xhost +local:docker
docker start -ai amr
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
docker exec -it amr bash
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
LIBGL_ALWAYS_SOFTWARE=1 ros2 run amr_bot mujoco_bridge
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
ros2 run nav2_map_server map_saver_cli -f /ros2_ws/src/amr_bot/maps/market
```

**4b. OR navigate autonomously on a saved map (nav2):**
```bash
ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=false \
  map:=/ros2_ws/src/amr_bot/maps/market.yaml \
  params_file:=/ros2_ws/src/amr_bot/config/nav2_params.yaml
```

**4c. Localize, then send a goal (nav2).** AMCL publishes no `map`→`odom` transform until it
is given an initial pose, so nothing works (RViz says `Frame [map] does not exist`) until you do
this. The robot spawns at the map origin. Either use RViz's **2D Pose Estimate** (Fixed Frame
must be `map` when you click, and click on the robot), or set it from the CLI:
```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}, covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.0685]}}"
```
Check it took (ground truth is `/odom`, which should agree with AMCL within a few cm):
```bash
ros2 topic echo /amcl_pose --once --field pose.pose.position
ros2 run tf2_ros tf2_echo map odom
```
Then send a goal with RViz's **2D Goal Pose** (Fixed Frame `map`), or from the CLI:
```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: -2.0, y: -2.4, z: 0.0}, orientation: {z: 0.7071, w: 0.7071}}}"
```

**5. Visualize:**
```bash
rviz2
```
- `Fixed Frame`: `map` while SLAM/nav2 is running, `odom` if neither is up yet.
- Displays to add: `LaserScan` on `/scan`, `Map` on `/map`, `RobotModel` on
  `/robot_description` (Description Source: Topic).
- On the `Map` display set `Topic` → `Durability Policy` = `Transient Local` and
  `Reliability Policy` = `Reliable`. `map_server` publishes the saved map once, and RViz's
  default (volatile) subscription never receives it.

**6. Native MuJoCo viewer** (optional — mirrors RViz's `RobotModel`, reads
the exact same live simulation `mujoco_bridge` is stepping): opens
automatically as part of `mujoco_bridge` itself once built with GLFW support
(see step 1) — no separate command. Controls: left-drag orbit, right-drag
pan, scroll zoom, hold shift for the horizontal drag variant.

## Other nodes in `amr_bot`

**`obstacle_avoider`** — lidar emergency stop. Reads `/scan`; if the nearest return in the
±20° forward cone (scan indices 160–200) is closer than 0.3 m, it publishes a zero `Twist`
on `/cmd_vel_safe`. It is standalone and not yet merged into `/cmd_vel` (that needs
`twist_mux`); silent when nothing is close, which is correct.
```bash
ros2 run amr_bot obstacle_avoider
ros2 topic echo /cmd_vel_safe
```

**`path_logger`** — *work in progress, built but not yet run or verified.* Writes one set of
CSVs per navigation goal into `runs/` (gitignored): the plans the controller received and
the robot's actual motion, for plotting planned vs actual path. Starts on `/goal_pose`, ends
on the navigation goal's terminal status. Parameters: `label`, `log_dir`, `rate_hz`,
`tail_s`. The analysis script and a goal-sending helper are not written yet.
```bash
ros2 run amr_bot path_logger --ros-args -p label:=baseline
```
Set a correct AMCL initial pose (step 4c) before recording anything.

## Debugging one link in the pipeline

```bash
ros2 node list
ros2 topic list
ros2 topic echo /odom --once
ros2 topic echo /scan --once
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo odom lidar_link
ros2 lifecycle list /controller_server   # nav2 node state, once bringup is up
ros2 lifecycle get /amcl                 # expect: active [3]
```

Nav2 prints nothing after `process started`, or nodes won't configure: a previous nav2 is
probably still running (closed terminals and Ctrl+C can leave it alive).
```bash
pgrep -fa nav2_bringup                        # should show exactly one launch
ros2 node list | sort | uniq -d               # must print nothing (duplicate node names)
pkill -9 -f component_container_isolated     # the container ignores SIGINT/SIGTERM, so -9 every time
pkill -f "ros2 launch nav2_bringup"
```
Killing only the launch process leaves the container alive and orphaned (PPID 1), and a new
bringup then collides with its node names and stays silent after `process started`.

## Notes

- Robot constants: wheel radius 0.035 m, wheel separation 0.205 m, mass 1.4 kg.
- Model path is currently hardcoded in `mujoco_bridge.cpp` as
  `/ros2_ws/src/amr_bot/models/world/supermarket_scene.xml` — not yet using
  `ament_index_cpp::get_package_share_directory`.
- `supermarket_scene.xml` = the robot (from `world.xml`) + `<include file="supermarket.xml"/>`
  for the store. `supermarket.xml` is world-only (no robot). The include must stay
  the first child of `<mujoco>` so the robot's `<compiler>`/`<option>` override the
  world file's `angle="degree"` / `timestep="0.005"`.
- A saved map only matches the world it was built in — re-run SLAM and re-save
  whenever the world file changes.
- Costmap tuning in `config/nav2_params.yaml` (both costmaps): `robot_radius` 0.15,
  `inflation_radius` 0.30, `cost_scaling_factor` 5.0. Nav2 reads params only at startup, so
  relaunch bringup after editing. Goal tolerances are still 0.25 m / 0.25 rad, so nav2 reports
  success within about 25 cm and 14° of the goal.

## Known issues

- RViz prints `[ERROR] ... indexed_8bit_image ... active samplers with a different type refer
  to the same texture image unit` when the `Map` display is created, and the map can render
  blank. The container has no GPU device (`/dev/dri`), so RViz uses software rendering.
  Cosmetic: nav2 and the map data are unaffected, so set the pose and goals from the CLI.
  Optional, untested: recreate the container with `--device /dev/dri`.
- A 2D Pose Estimate clicked on a blank viewport can land far outside the map and put AMCL
  kilometres off. Re-set the pose (step 4c) and verify with `/amcl_pose`.
- The suspected odom twist frame bug and the goal alignment / path-following analysis are
  open. The plan and current status are in `PROCESS.md` sections 15–17.
- See `PROCESS.md` (gitignored, local only) for the full build log, every
  bug hit, and why — useful background before touching this code again.
