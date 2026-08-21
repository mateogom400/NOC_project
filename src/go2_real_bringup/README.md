# go2_real — Real-Robot Deployment Stack

Bringing the A\*+MPC navigation framework to the **physical Unitree Go2**, driving the robot through its **built-in Sport-mode controller** (no CHAMP). The MPC publishes body-frame `/cmd_vel`; `go2_bringup/go2_hw_bridge` forwards those commands to the Unitree Sport API at `/api/sport/request`.

This is a thin orchestration layer on top of:
- [`a_star_mpc_planner`](../a_star_mpc_planner/) — the actual planner + MPC
- [`go2_bringup`](../go2_bringup/) — the Unitree Sport-API hardware bridge

## Packages

| Package | Role |
|---|---|
| [`go2_real_bringup`](../go2_real_bringup/) | Top-level launch — wires everything together |
| [`robot_real_lidar`](../robot_real_lidar/) | `/utlidar/cloud` → `/lidar/points_filtered` adapter |
| [`go2_real_planner`](../go2_real_planner/) | Wraps `a_star_mpc_planner` with BO-YAML selector |
| [`robot_real_goal_manager`](../robot_real_goal_manager/) | RViz `/goal_pose` relay + waypoint mission runner |

## Pipeline

```
  /utlidar/cloud  ──►  robot_real_lidar  ──►  /lidar/points_filtered
                                                    │
  RViz /goal_pose ──►  goal_relay   ──►  /global_goal
  (or)  mission.yaml ──►  mission_runner ─►  /global_goal
                                                    │
                                                    ▼
                          a_star_mpc_planner (A* + MPC + setpoint controller)
                                                    │
                                                    ▼  /cmd_vel  (or /cmd_vel_raw → safety → /cmd_vel)
                                          go2_bringup/go2_hw_bridge
                                                    │
                                                    ▼  /api/sport/request
                                          Unitree Go2 built-in controller
```

## Network & DDS Setup (do this first)

Every message to and from the Go2 travels over **DDS**. The robot's onboard ROS 2 bridge ([`unitree_ros2`](https://github.com/unitreerobotics/unitree_ros2)) is built against **CycloneDDS** and discovers peers on the network interface bridged to the robot's internal `192.168.123.0/24` network. You must use the **same RMW** and **pin discovery to the right interface**, otherwise topics like `/sportmodestate` and `/utlidar/cloud` will never appear.

### 1. Use CycloneDDS

```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### 2. Identify the Go2 network interface

```bash
ip -br addr | grep 192.168.123
# example output:
# enx00e04c360b9d   UP   192.168.123.51/24
```

The interface name (`enx00e04c360b9d`, `eth0`, ...) is the one you must pin DDS to.

### 3. Edit the bundled CycloneDDS profile

```bash
# Edit the line  <NetworkInterface name="eth0" .../>  to match step 2
$EDITOR src/go2_real_bringup/config/cyclonedds_profile.xml
```

### 4. Export the profile path

```bash
export CYCLONEDDS_URI=file://$(ros2 pkg prefix go2_real_bringup)/share/go2_real_bringup/config/cyclonedds_profile.xml
```

Put the three `export` lines in your `~/.bashrc` (or the Jetson's `~/.profile`) so every shell that talks to the Go2 picks them up automatically.

### 5. Verify discovery

```bash
ros2 topic list | grep -E "sportmodestate|utlidar"
# Expect: /sportmodestate /lowstate /utlidar/cloud  (among others)
```

If nothing appears, the most common culprits are:
- Wrong interface name in `cyclonedds_profile.xml`
- The Go2 onboard `unitree_ros2` bridge is not running
- The host firewall is blocking UDP multicast on the interface
- `RMW_IMPLEMENTATION` was set in only one shell — the bridge and your stack must agree

## Quick Start

### 1. Build

```bash
cd ~/Go2_navigation
colcon build --symlink-install --packages-select \
  go2_real_bringup robot_real_lidar go2_real_planner robot_real_goal_manager
source install/setup.bash
```

### 2. Launch (BO-tuned parameters, RViz on)

```bash
ros2 launch go2_real_bringup go2_real_full.launch.py
```

This brings up: `robot_state_publisher`, `go2_hw_bridge` (Unitree Sport API), the LiDAR adapter, the A\*+MPC planner, the safety gate, the goal manager, and RViz2.

### 3. Send a goal

In RViz, click **"2D Goal Pose"** and drop a goal. The `goal_relay_node` forwards it onto `/global_goal` (with the frame re-stamped to `odom`).

## Choosing Parameters

The planner can run with either the **hand-tuned baseline** (Phase 0 of the paper) or the **BO-tuned** weights (Phase 1).

```bash
# BO-tuned (default)
ros2 launch go2_real_bringup go2_real_full.launch.py use_bo_params:=true

# Hand-tuned baseline
ros2 launch go2_real_bringup go2_real_full.launch.py use_bo_params:=false

# Custom file (e.g. a fresh tuning output)
ros2 launch go2_real_bringup go2_real_full.launch.py \
  params_file:=$HOME/Go2_navigation/tuning/tuning_results/best_planner_params.yaml
```

Bundled files live in [`go2_real_planner/config/`](../go2_real_planner/config/):

| File | Used when |
|---|---|
| `planner_params_default.yaml`  | `use_bo_params:=false` |
| `planner_params_bo_tuned.yaml` | `use_bo_params:=true` (default) |
| *any path*                     | `params_file:=<path>` (overrides both) |

Drop a fresh `best_planner_params.yaml` from `tuning/tuning_results/` into `go2_real_planner/config/planner_params_bo_tuned.yaml` to ship a new BO result with the deployment package.

## Running a Waypoint Mission

For reproducible benchmark trials, the mission runner sequences goals from a YAML file:

```bash
ros2 launch go2_real_bringup go2_real_full.launch.py \
  mission_file:=$(ros2 pkg prefix robot_real_goal_manager)/share/robot_real_goal_manager/config/example_mission.yaml
```

Edit [`robot_real_goal_manager/config/example_mission.yaml`](../robot_real_goal_manager/config/example_mission.yaml) to define your own waypoints. Pass `repeat:=true` to loop forever.

## Launch Arguments

```bash
ros2 launch go2_real_bringup go2_real_full.launch.py [args...]
```

| Argument | Default | Description |
|---|---|---|
| `use_bo_params` | `true` | BO-tuned vs hand-tuned planner YAML |
| `params_file`   | `""`   | Override planner YAML path (wins over `use_bo_params`) |
| `mission_file`  | `""`   | Waypoint YAML; empty = wait for RViz goals only |
| `use_safety`    | `true` | Insert `robot_safety` velocity_limiter before hw_bridge |
| `use_rviz`      | `true` | Launch RViz2 with the bundled config |
| `use_sim_time`  | `false`| Keep false on hardware |

## Planner-Only Restart

Useful for swapping parameter sets without restarting the hardware bridge:

```bash
# Terminal 1 — hardware only (leave running)
ros2 launch go2_bringup go2_hardware.launch.py

# Terminal 2 — planner only, restart freely
ros2 launch go2_real_bringup go2_real_planner_only.launch.py use_bo_params:=false
```

## Diagnostics

```bash
ros2 topic echo /mpc/diagnostics        # success, cost, solve_ms, avg_ms, fails
ros2 topic hz   /cmd_vel                # should be ~20 Hz
ros2 topic hz   /lidar/points_filtered  # should be ~10 Hz
ros2 topic echo /api/sport/request      # raw Unitree Sport API commands
```

## Why no CHAMP

CHAMP is a hierarchical locomotion controller that converts `/cmd_vel` into per-leg joint targets — useful in simulation where the Go2 leg controller is unavailable. On real hardware the Go2 ships with Unitree's own production locomotion controller, accessible via the Sport API. `go2_bringup/go2_hw_bridge` already speaks that protocol, so the entire CHAMP layer is unnecessary on the real robot.
