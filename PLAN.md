# AMR C++ / MuJoCo — 7-Day Interview Prep Plan

## Context

You have an Addverb interview in ~1 week. You already own a **working** ROS 2 AMR
stack at `Ros-AMR-Mobile-Robot/supermarketbot` — but it is Python, and you built
it by wiring things together rather than deriving them. The interview will ask
*why*, not *what command*.

So the goal of this project is **not a new robot**. It is: re-derive the same
stack in C++, from concepts, fast enough that you can whiteboard any of your 11
questions cold. The existing repo becomes the answer key you check yourself
against — not the thing you copy.

**My role from here: guide only.** I explain approaches, sketch data flow, point
at the right API, ask you questions back. I do not write your nodes unless you
say "write it".

### What you already have (reuse as data, do not rebuild)
| Asset | Path | Note |
|---|---|---|
| Robot MJCF | `supermarketbot/world/world.xml` | full world: robot + 360 rangefinder rays + supermarket |
| Robot-only MJCF | `supermarketbot/urdf/supermarketbot.xml` | 1 ray only — use `world.xml` |
| Meshes | `supermarketbot/meshes/*.stl` | chassis L1-3, wheel, lidar |
| URDF/xacro | `supermarketbot/urdf/supermarketbot.{urdf,xacro}` | for `robot_state_publisher` + RViz |
| Nav2 params | `supermarketbot/config/nav2_params.yaml` | DWB + NavFn, robot_radius 0.22 |
| SLAM params | `supermarketbot/config/*slam_toolbox_cfg.yaml` | async/sync variants |
| Docker | `dockerfile` (ROS 2 Humble + nav2 + slam_toolbox + mujoco) | add `ros-humble-nav2-*` dev headers only if you write plugins |

Robot constants, memorise these: **wheel radius 0.035 m, wheel separation
0.205 m, mass 1.4 kg, caster at rear, lidar at z=0.15**.

### Target workspace
```
~/projects/amr_cpp_ws/src/amr_bot/     # ament_cmake, C++
  ├── CMakeLists.txt  package.xml
  ├── src/            # your nodes
  ├── models/         # copied world.xml + meshes (relative paths need fixing)
  ├── urdf/  config/  launch/
```

---

## The lazy scope decision (read this before day 1)

Do **not** rewrite the whole stack in C++. In one week you write **three C++
nodes**, and everything else is off-the-shelf:

1. `mujoco_bridge` — sim ↔ ROS. The one node that must be C++ (and it's the one
   an interviewer will dig into).
2. `diff_drive_controller` — your PID wheel-velocity loop.
3. `reactive_avoider` — lidar → steering (VFH/gap-following).

`slam_toolbox`, `nav2`, `robot_state_publisher`, `teleop_twist_keyboard` stay as
installed packages. Writing your own SLAM in week 1 is how you arrive at the
interview with nothing working. MPC is day 6 *only if* days 1-5 are green.

---

## Day-by-day

### Day 1 — Docker + MuJoCo + workspace (covers Q11, Q7, Q6)

**Q11 — existing ROS 2 Humble Docker.** Approach, in order:
- `docker build -t amr:humble -f dockerfile .` from the old repo, then extend it
  with a new `dockerfile` for the C++ package (`ros-humble-ament-cmake`,
  `libmujoco` or the pip mujoco's headers — see below).
- GUI from a container needs: `--net=host -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix`
  plus `xhost +local:docker`, and `--gpus all` if you want the MuJoCo viewer fast.
- Mount your workspace instead of `COPY`ing it: `-v ~/projects/amr_cpp_ws:/ros2_ws`.
  You edit on the host, build in the container. Rebuilding an image per code
  change is the classic time sink.
- Key concept to be able to explain: ROS 2 uses DDS multicast, so two containers
  on the same `--net=host` see each other's topics; separate bridge networks do
  not without `ROS_DOMAIN_ID` + discovery config.

**Q7 — MuJoCo setup for C++.** This is the piece the old repo does *not* have
(it uses the Python bindings). Approach:
- Download the official MuJoCo release tarball (it ships prebuilt `libmujoco.so`
  + headers). No build from source needed.
- In `CMakeLists.txt`: `find_library(MUJOCO_LIB mujoco HINTS ...)`,
  `target_include_directories`, `target_link_libraries`. Set `LD_LIBRARY_PATH`
  or use `install(RPATH)`.
- Core API you'll live in: `mj_loadXML` → `mjModel*`, `mj_makeData` → `mjData*`,
  `mj_step(m,d)`, `d->ctrl[]` (actuators in), `d->qpos/qvel/sensordata` (state
  out), `mj_name2id(m, mjOBJ_JOINT, "left_wheel_joint")` for index lookup.
- Viewer: the C++ path is `simulate` from the MuJoCo samples (GLFW). Lazy
  option: run headless and visualise in RViz instead. Recommended for week 1.

**Q6 — C++ working directory.** The real question behind it, and what to say:
- A process's cwd is *where you launched it from*, not where the binary lives.
  `ros2 run` inherits your shell's cwd, `ros2 launch` too — so **never** use
  relative paths to models/configs.
- The ROS 2 answer: `ament_index_cpp::get_package_share_directory("amr_bot")`
  then `/ "models" / "world.xml"` (use `std::filesystem::path`).
- That only works if `CMakeLists.txt` has
  `install(DIRECTORY models urdf config launch DESTINATION share/${PROJECT_NAME})`.
  Forgetting that install rule is the #1 "it works in source but not installed"
  bug. Be ready to say this.
- Gotcha specific to you: `world.xml` has `meshdir="."` and `file="../meshes/*.stl"`.
  Once installed to `share/`, that relative chain breaks. Fix the MJCF's
  `meshdir` when you copy it, or load via `mj_loadXML` with the correct cwd.

**End-of-day checkpoint:** a C++ node that loads `world.xml`, steps 1000 times,
and prints `d->qpos[0..2]`. Nothing ROS yet.

---

### Day 2 — The bridge: odom, TF, joint states (covers Q8-part-1, Q2-setup)

Build `mujoco_bridge` as a C++ `rclcpp::Node`. What it must do, and the concepts
to have ready:

**Inbound:** subscribe `/cmd_vel` (`geometry_msgs::msg::Twist`). Convert (v, ω)
to wheel speeds — **derive this on paper, it is a guaranteed interview question**:
```
ω_L = (v − ω·L/2) / r      ω_R = (v + ω·L/2) / r      L=0.205, r=0.035
```
Write those into `d->ctrl[0]`, `d->ctrl[1]` (the MJCF has `velocity` actuators
with `kv=5`, i.e. MuJoCo already runs a P velocity loop for you — know that,
it's why "where's your PID?" has a nuanced answer).

**Sim loop:** `mj_step` at 500 Hz, publish at 20 Hz. Two timers, or one timer
that steps N times. Discuss: sim time vs wall time, and `use_sim_time`.

**Outbound:**
- `/joint_states` — wheel positions/velocities from `qpos`/`qvel` via
  `jnt_qposadr` / `jnt_dofadr` (not the joint id directly — explain why: a
  freejoint eats 7 qpos slots).
- `/odom` + TF `odom → base_link`. Here's the honest bit: the base has a
  `freejoint`, so `qpos[0:7]` is **ground truth**, not odometry. For an interview
  you should *also* implement wheel-encoder dead reckoning
  (`x += v·cos(θ)·dt`, `θ += ω·dt`) and be able to contrast the two — drift,
  wheel slip, why SLAM needs odom to be *smooth* not *accurate*.
- TF tree you are building toward: `map → odom → base_link → lidar_link/wheels`.
  `odom→base_link` is yours; `map→odom` is SLAM's; `base_link→*` is
  `robot_state_publisher` from the URDF. Know who publishes what — classic question.

**Checkpoint:** `ros2 topic echo /odom`, `ros2 run tf2_tools view_frames`.

---

### Day 3 — Lidar + teleop (covers Q2, Q5)

**Q2 — lidar data processing.** `sensor_msgs::msg::LaserScan` anatomy:
`angle_min`, `angle_max`, `angle_increment`, `ranges[]`, `range_min/max`.
Ray *i* is at angle `angle_min + i·angle_increment` **in `frame_id`'s frame**.
Concepts to nail:
- Converting a scan to points: `x = r·cos(θ), y = r·sin(θ)` — then TF into
  `base_link` before using it for anything.
- Invalid returns: MuJoCo rangefinder gives **−1 for no-hit**; LaserScan wants
  `inf` (or NaN). Filtering these correctly is where most bugs live.
- Downsampling, median filter for spikes, and why you *don't* smooth across the
  angle_max/angle_min wraparound.
- Be ready to explain the difference between a scan, a PointCloud2, and a
  costmap — three representations of the same rays.

Your `world.xml` has 360 `rangefinder` sensors named `lidar_0..359` with
`cutoff=10`. Read them with `d->sensordata` + `m->sensor_adr`.

**Q5 — teleop.** Three levels, do all three:
1. `ros2 run teleop_twist_keyboard teleop_twist_keyboard` → remap `/cmd_vel`.
   Free. Do this first to validate day 2.
2. Joystick: `joy_node` → `teleop_twist_joy`. Know the deadman-switch concept.
3. Your own C++ teleop with `termios` raw mode — good only if you want to say
   "I've written one". Low priority.
- Concept: `twist_mux` for priority between teleop / nav2 / avoider on `/cmd_vel`.
  This one impresses; it's the real answer to "how do you stop a robot mid-nav".

**Checkpoint:** drive the robot around the supermarket world with arrow keys,
watch `/scan` in RViz.

---

### Day 4 — SLAM (covers Q4)

**Setup is 20 minutes, understanding is the day.** Run `slam_toolbox` async mode
with the old repo's `async_slam_toolbox_cfg.yaml` (copy it). Requirements:
`/scan`, TF `odom→base_link`, `base_link→lidar_link`. It publishes `map→odom`
and `/map`.

**Q4 — how SLAM actually works.** Build this mental model:
- The problem: pose and map are mutually dependent (chicken/egg). SLAM solves
  both jointly.
- **Front end — scan matching.** New scan vs. previous scan/submap. ICP
  (iterative closest point) or correlative scan matching. Output: a relative
  pose constraint + covariance. This is what "odom is a prior" means.
- **Back end — pose graph optimisation.** Nodes = robot poses, edges =
  constraints. **Loop closure** adds an edge between a new pose and an old one
  when the scans match → optimiser (Ceres, in slam_toolbox) redistributes the
  accumulated drift over the whole graph. Be able to draw this.
- **Occupancy grid**: log-odds update per cell, ray-casting free space along each
  beam, marking the endpoint occupied. Know the three cell states (−1 unknown,
  0 free, 100 occupied) — this exact encoding shows up in Q10.
- Contrast: **particle-filter SLAM (gmapping)** vs **graph SLAM (slam_toolbox,
  cartographer)**. Why graph won: loop closure corrects the past, particles can't.
- Localisation ≠ SLAM: **AMCL** is Monte Carlo localisation on a *known* map.
  Nav2 uses it once you save the map. Know the difference cold.

Then: teleop the store, `ros2 run nav2_map_server map_saver_cli -f my_map`.

---

### Day 5 — Nav2 (covers Q1, Q10, Q3)

**Q1 — Nav2 setup.** The mental model, in the order it matters:
- Nav2 is **lifecycle nodes** managed by `lifecycle_manager` (unconfigured →
  inactive → active). "Nav2 doesn't do anything" is almost always a node that
  never activated. Know `ros2 lifecycle list`.
- Servers: `planner_server` (global), `controller_server` (local),
  `behavior_server` (recoveries: spin/backup/wait), `bt_navigator` (the
  orchestrator), `smoother_server`, `velocity_smoother`, plus `map_server` +
  `amcl` for localisation.
- The **behaviour tree** is the brain: `navigate_to_pose_w_replanning_and_recovery.xml`.
  Ticks the planner at 1 Hz, the controller at 20 Hz, and falls into recovery
  branches on failure. Being able to describe this tree is a strong answer.
- Your inputs: `/scan`, `/odom`, TF `map→odom→base_link`, `/map`, and a goal.
- Start from the repo's `nav2_params.yaml`. Tune `robot_radius` (0.22) and the
  velocity limits to your robot — most nav2 failures are geometry lies.

**Q10 — local vs global planners.**
- **Global**: searches the *static* costmap for a full path, start→goal, run
  infrequently (1 Hz). `NavFn` = Dijkstra/A* over the grid; `Smac` = hybrid-A*
  with kinematic feasibility. Output: a `nav_msgs/Path`. Doesn't know about
  moving people.
- **Local**: the *controller*. Takes a window of the global path plus the rolling
  local costmap (which sees live lidar) and emits a `Twist` at 20 Hz. `DWB` =
  dynamic window: sample feasible (v, ω) pairs, roll each one forward, score
  each trajectory with critics (path alignment, goal dist, obstacle cost) — pick
  the best. `MPPI` and `RPP` (pure pursuit) are the modern alternatives.
- **Costmap layers** — know all four: `static_layer` (the SLAM map),
  `obstacle_layer`/`voxel_layer` (live scan), `inflation_layer` (cost gradient
  around obstacles; `inflation_radius` + `cost_scaling_factor` is *the* tuning
  knob), `static/other`. Layered costmap = each layer writes into a master grid.
- Be ready for: "global says the path is clear, local refuses to move" → answer
  is inflation radius vs robot radius, or a stale TF.

**Q3 — the feedback loop for autonomous driving.** Draw the full loop:
```
lidar+odom → SLAM/AMCL → pose in map
pose + goal → global planner → path
path + local costmap → local planner (DWB) → cmd_vel
cmd_vel → your diff-drive controller → wheel velocities → plant (MuJoCo)
plant → encoders + lidar → back to the top
```
Every box is a rate. Say the rates. Then say where it breaks: TF timeout, stale
scans, control rate lower than the plant's dynamics.

---

### Day 6 — Your own controller + obstacle avoidance (covers Q8, Q9)

**Q8 — PID, as a standalone C++ node.** Where the PID actually goes matters more
than the formula:
- Loop 1 (inner, fast): wheel velocity. setpoint = ω_target from kinematics,
  measurement = `qvel` (your "encoder"), output = torque. Note: the MJCF's
  `velocity` actuator (`kv=5`) is already doing this. To write your own PID you
  swap it for a `motor` actuator and close the loop yourself — do this, it's the
  demonstrable piece.
- Loop 2 (outer): pose → cmd_vel (heading PID toward a waypoint). This is a
  "go-to-goal" controller and is a classic live-coding ask.
- Implementation details that mark experience: anti-windup (clamp the integral),
  derivative on *measurement* not error (avoids setpoint kick), output
  saturation, and a real `dt` from the clock, not an assumed constant.
- Leave the gains as ROS parameters so you can tune live with `ros2 param set` —
  a real robot always needs a calibration knob.

**Q9 — obstacle avoidance from lidar.** Three tiers, know all, implement one:
- Tier 1 — **emergency stop / bubble**: min range in a forward cone < threshold
  → zero the Twist. 20 lines. Do this first; it's what actually ships.
- Tier 2 — **gap following / VFH**: bin the scan into angular sectors, build a
  polar obstacle-density histogram, pick the widest gap that admits the robot
  width, steer toward its centre. This is your day-6 build.
- Tier 3 — **potential fields**: attractive to goal + repulsive from each
  return. Know its failure mode (local minima, oscillation in corridors) — the
  fact that you can name the failure mode is the point.
- Where it sits: as a `twist_mux` input with higher priority than nav2, or
  inline between nav2 and the bridge. Discuss both.

**Q8b — MPC. Only if days 1-5 are green.** Don't implement. Be able to explain:
predict N steps forward with the unicycle model, minimise a cost over the
control sequence subject to (v, ω) limits, apply only the first control,
re-solve next tick (receding horizon). Why it beats PID: handles constraints and
anticipates the path's curvature. Why you'd skip it: needs a solver (OSQP/acados)
and a good model, and it's ~10x the compute. Nav2's `MPPI` controller is the
sampling-based cousin — mentioning that you'd just enable MPPI rather than write
an MPC is the *senior* answer.

---

### Day 7 — Integrate, then rehearse

- Full run: bridge → SLAM → save map → nav2 with saved map → send a goal in
  RViz → robot drives an aisle with your avoider armed.
- **Then diff against the Python repo.** Open `mujoco_bridge.py` and
  `nav2_params.yaml` and find every place your version differs. Each difference
  is either your bug or their bug — resolving them is the highest-value hour of
  the week.
- Write a one-page README with the architecture diagram and the rate of every
  box. That page *is* your interview script.
- Rehearse out loud, 3 minutes each, on all 11 topics.

---

## Deliberate omissions

| Skipped | Add when |
|---|---|
| C++ MuJoCo GUI viewer (GLFW) | RViz insufficient; after day 5 |
| Custom SLAM implementation | You have a month, not a week |
| MPC solver integration | Days 1-5 green and you want the flex |
| nav2 pluginlib wrappers for your controllers | After standalone nodes work |
| Rewriting vision.py / product_locator.py | Not in your 11 questions |
| C++ URDF/xacro changes | Existing URDF is fine as-is |

## Verification

Each day ends with one runnable check, in order:
1. `./mujoco_test` prints changing `qpos` — model loads.
2. `ros2 topic echo /odom` moves when you publish `/cmd_vel`; `view_frames` shows `odom→base_link`.
3. RViz shows a 360° scan that matches the walls; arrow keys drive.
4. `/map` grows in RViz while you teleop; `map_saver_cli` writes a `.pgm`.
5. RViz "Nav2 Goal" → robot plans and reaches a pose in another aisle.
6. Drive at a shelf with the avoider on → it stops or steers around.
7. All of the above in one launch file, from a cold container.

## Open question for you (answer before day 1)

The MJCF's `velocity` actuators mean MuJoCo closes the wheel loop internally.
Do you want to swap them for `motor` actuators on day 6 so your PID has
something to actually control? I'd say yes — otherwise "I wrote a PID" is a
layer of cake on top of a PID that was already there, and an interviewer will
find that.
