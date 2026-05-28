# SUPER Compatibility in YOPO_lidar_V3

Python port of **SUPER/super_planner** + **SUPER/mission_planner** for this repo's ROS2 + LiDAR stack.

## Architecture mapping

| SUPER (C++) | policy/super (Python) | Status |
|-------------|-------------------------|--------|
| `rog_map` rolling occupancy | `super_map.py` | Ported (2D slice @ cruise height) |
| `path_search/Astar` | `grid_astar.py` (`super_astar.py` wrapper) | Ported (2D grid + horizon fallback) |
| `SuperPlanner::PlanFromRest` | `super_planner_core.plan_from_rest` | Ported |
| `SuperPlanner::ReplanOnce` | `super_planner_core.replan_once` | Ported (lite) |
| `CorridorGenerator` | `corridor_builder.py` | Adapted (2D safe corridor abstraction) |
| `ExpTrajOpt` (MINCO-like stage) | `exp_traj_optimizer.py` + `committed_trajectory.py` | Adapted (time-parameterized committed trajectory) |
| `BackupTrajOpt` | `backup_traj_optimizer.py` | Adapted (fallback stop trajectory generation) |
| `YawTrajOpt` | `yaw_traj_optimizer.py` + sampler in `super_compat_planner.py` | Adapted (yaw trajectory synchronized with committed trajectory) |
| `fsm/Fsm` | `super_fsm.py` + node integration | Ported |
| `mission_planner/WaypointPlanner` | `super_mission.py` | Ported |
| ROS command output | `super_compat_planner.py` | Uses `/so3_control/pos_cmd` |

## Module layout

```text
policy/super/
  super_compat_planner.py   # ROS2 node entry
  super_planner_core.py     # PlanFromRest / ReplanOnce
  super_map.py              # Rolling occupancy map
  grid_astar.py             # Grid A* + horizon fallback
  path_optimizer.py         # simplify / shortcut / lookahead / path delta
  trajectory_optimizer.py   # min-jerk style path interpolation
  corridor_builder.py       # corridor stage (Python adaptation)
  exp_traj_optimizer.py     # expected trajectory optimizer
  backup_traj_optimizer.py  # backup trajectory optimizer
  yaw_traj_optimizer.py     # yaw trajectory optimizer
  committed_trajectory.py   # committed trajectory sampling model
  super_astar.py            # compatibility wrapper
  super_path_utils.py       # compatibility wrapper
  super_fsm.py              # INIT → WAIT_GOAL → GENERATE_TRAJ → FOLLOW_TRAJ → EMER_STOP
  super_mission.py          # Waypoint mission sequencer
  data/waypoints_example.txt
  script/run_super_ros2.sh
  script/test_super.py
```

## Quick start (full stack)

```bash
source /opt/ros/humble/setup.bash
source Controller/src/install_ros2/setup.bash
source Simulator/src/install_ros2/setup.bash
ros2 launch yopo_bringup system.launch.py planner_method:=super_compat
```

Send goals in RViz (`/move_base_simple/goal`, frame `world`).

## Mission mode (SUPER mission_planner)

Load waypoint file (`x y z switch_dis` per line):

```bash
ros2 launch yopo_bringup system.launch.py planner_method:=super_compat \
  --ros-args -p mission_file:=waypoints_example.txt
```

Or via direct node:

```bash
bash policy/super/script/run_super_ros2.sh --ros-args \
  -p mission_file:=waypoints_example.txt
```

## Coordinate frames

| Topic | Frame | Pair with |
|-------|-------|-----------|
| `/sim/odom` | world | always |
| `/move_base_simple/goal` | world | always |
| `/lidar_points` | body | `world_frame_cloud:=false` (default) |
| `/lidar_points_world` | world | `world_frame_cloud:=true` |
| `/super_compat/planned_path` | world | visualization |

## Key parameters

| Parameter | Default | SUPER equivalent |
|-----------|---------|------------------|
| `planning_horizon` | 16.0 | local map / search horizon |
| `obstacle_inflation` | 0.7 | inflated occupancy |
| `goal_tolerance` | 0.8 | `closeToGoal` threshold |
| `mission_file` | "" | waypoint file path |
| `mission_switch_dis` | 3.0 | waypoint switch distance |
| `super_param_file` (launch arg) | `policy/super/config/super_compat.yaml` | fixed regression baseline parameter bundle |

## Diagnostics and regression

- Runtime diagnostics topics:
  - `/super_compat/diag/fsm_state`
  - `/super_compat/diag/plan_ret`
  - `/super_compat/diag/metrics` (`distance_to_goal`, `distance_to_path`, `speed_xy`, `unsafe_hold`, `emer_count`, `connected`, `path_ready`, `goal_latched`, `z`, `clearance_proxy`)
- End-to-end smoke + reach test:
  - `bash Controller/src/utils/yopo_bringup/scripts/test_super_compat.sh`
- Quantitative parity regression:
  - `python3 Controller/src/utils/yopo_bringup/scripts/super_parity_regression.py --output super_parity_report.json`
  - `bash Controller/src/utils/yopo_bringup/scripts/run_super_parity_regression.sh`

## Remaining future work

- 3D ROG map + probabilistic fusion
- Full 3D IRIS corridor generation (`CorridorGenerator`)
- Full MINCO backend parity (current implementation is 2D adapted)
- Full yaw polynomial optimization parity (`YawTrajOpt`)

These require C++ libs or heavy numerical code; current port focuses on **map → A* → FSM → safe command** compatible with YOPO_lidar_V3 controller.
