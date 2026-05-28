# YOPO LiDAR + ROS2 — Build and run

This repository is **LiDAR-only** and **ROS2-only**: depth-image modes, `sensor_mode` toggles, and ROS1 entry points were removed.
`YOPO_lidar_V3` keeps the planner pipeline but uses a **Point Transformer V3-style point-cloud backbone** (no range-image backbone in runtime/train path under `policy/YOPO/`).

## 1. Requirements

- Ubuntu 22.04 (or compatible) with **ROS2 Humble**
- NVIDIA driver + CUDA (for `sensor_simulator`)
- Conda with **Python 3.10** for `rclpy` compatibility

## 2. Clone layout

Assume the workspace root is `YOPO_lidar/` with subfolders `policy/YOPO/`, `policy/super/`, `Controller/`, `Simulator/`.

## 3. Python environment

```bash
conda create -n yopo python=3.10 -y
conda activate yopo
cd YOPO_lidar
pip install -r policy/YOPO/requirements.txt
```

## 4. Build ROS2 packages

```bash
cd YOPO_lidar
source /opt/ros/humble/setup.bash
bash scripts/build_controller_ros2.sh
bash scripts/build_simulator_ros2.sh
```

Artifacts:

- `Controller/src/install_ros2/`
- `Simulator/src/install_ros2/`

Optional: `export YOPO_CUDA_ARCH=86` (or your SM) if `nvcc` arch detection fails.

## 5. Full stack (single launch)

```bash
cd YOPO_lidar
source /opt/ros/humble/setup.bash
source Controller/src/install_ros2/setup.bash
source Simulator/src/install_ros2/setup.bash
ros2 launch yopo_bringup system.launch.py weight:=saved/YOPO_1/epoch50.pth
```

(`conda run -n yopo` is wired inside this launch file for the planner.)

## 6. Manual three-terminal flow

### Terminal A — quadrotor + attitude control

```bash
cd YOPO_lidar
source /opt/ros/humble/setup.bash
source Controller/src/install_ros2/setup.bash
ros2 launch so3_quadrotor_simulator simulator_attitude_control.launch.py
```

### Terminal B — LiDAR simulator

```bash
cd YOPO_lidar
source /opt/ros/humble/setup.bash
source Controller/src/install_ros2/setup.bash
source Simulator/src/install_ros2/setup.bash
ros2 run sensor_simulator sensor_simulator
```

### Terminal C — YOPO planner

```bash
cd YOPO_lidar
source /opt/ros/humble/setup.bash
source Controller/src/install_ros2/setup.bash
conda activate yopo
python policy/YOPO/script/test_yopo_ros.py --ros_version ros2 --weight policy/YOPO/saved/YOPO_2/epoch1.pth
```

### Terminal D — RViz (optional)

```bash
cd YOPO_lidar
rviz2 -d visualization/yopo_ros2.rviz
```

`yopo_ros2.rviz` is aligned with the `YOPO_lidar` layout, with V3-specific topic adaptation (`/mock_map`, `/lidar_points_world`) and no range-image panel.

## 7. Health checks (topics)

```bash
source /opt/ros/humble/setup.bash
source YOPO_lidar/Controller/src/install_ros2/setup.bash
source YOPO_lidar/Simulator/src/install_ros2/setup.bash
ros2 topic echo /sim/odom --once
ros2 topic echo /lidar_points --once
ros2 topic echo /lidar_points_world --once
ros2 topic echo /so3_control/pos_cmd --once
```

## 8. Dataset collection (`lidar_*.bin` only)

```bash
source /opt/ros/humble/setup.bash
source YOPO_lidar/Simulator/src/install_ros2/setup.bash
ros2 run sensor_simulator dataset_generator
```

Outputs under `Simulator/src/config/config.yaml` → `save_path` (default `../dataset/`): folders `0/`, `1/`, … with `lidar_i.bin` plus `pose-*.csv` at dataset root.

Keep **`Simulator` LiDAR layout** aligned with **`policy/YOPO/config/traj_opt.yaml`** (`pointcloud_num_points` / `pointcloud_feature_dim` / `lidar_vertical_fov` / `lidar_sensing_horizon`).

## 9. Training

```bash
conda activate yopo
cd YOPO_lidar/policy/YOPO
python train_yopo.py
```

Training loads only `*.bin` + `pose-*.csv` (see `policy/yopo_dataset.py`).

#### TensorBoard监控:

```bash
# 启动TensorBoard
tensorboard --logdir=./saved --port=6006

# 在浏览器打开 http://localhost:6006
```


## 10. Inference CLI

```bash
python policy/YOPO/script/test_yopo_ros.py --ros_version ros2 --weight policy/YOPO/saved/YOPO_1/epoch50.pth \
  --odom_topic /sim/odom --lidar_topic /lidar_points --ctrl_topic /so3_control/pos_cmd
```

## 11. Configuration reference

| File | Role |
|------|------|
| `Simulator/src/config/config.yaml` | Map generation, `lidar:` ray grid, `lidar_fps`, dataset paths |
| `policy/YOPO/config/traj_opt.yaml` | Point-cloud token shape, lattice / primitives, dataset_path, speeds |

`policy/YOPO/config/config.py` always applies the **360° LiDAR lattice** derivations (no `sensor_mode`).

## 12. Launch arguments (`yopo_bringup`)

- `yopo_root` — absolute path to the `policy/YOPO/` directory (defaults beside `Controller/` in this repo)
- `super_root` — absolute path to the `policy/super/` directory (used by `planner_method:=super_compat`)
- `weight` — `.pth` file relative to `yopo_root` unless absolute
- `flight_height` — fixed planner flight height, default `2.0` and clamped to the forest map range `[0.5, 4.0]` m
- `max_speed` — planner speed limit for compatible planner branches, default `10.0` m/s
- `super_param_file` — ROS2 parameter baseline for SUPER-compatible planner (default `policy/super/config/super_compat.yaml`)

Example:

```bash
ros2 launch yopo_bringup full_yopo_ros2.launch.py \
  yopo_root:=/home/you/YOPO_lidar/policy/YOPO \
  weight:=saved/YOPO_1/epoch50.pth
```

## 13. Troubleshooting

- **`ModuleNotFoundError: rclpy`** — use Python **3.10** in the environment that runs `policy/YOPO/script/test_yopo_ros.py`.
- **`quadrotor_msgs` not found** — source `Controller/src/install_ros2/setup.bash` before Python.
- **Empty dataset / wrong path** — ensure `dataset_path` in `traj_opt.yaml` points to the folder that contains `pose-0.csv` and map subfolders (paths are resolved from `policy/YOPO/`).

## 14. Migration note

Old commands (`rosrun`, `catkin_make`, `--trial/--epoch`, `--sensor_mode depth`, `/depth_image`) are intentionally unsupported; use the ROS2 + LiDAR flow above.

## 15. Planner method branches

`yopo_bringup` now supports planner-level branching:

- `planner_method:=yopo` (default, existing YOPO method)
- `planner_method:=super_compat` (SUPER-style compatible branch inside this repo)

Run SUPER-compatible branch:

```bash
source /opt/ros/humble/setup.bash
source Controller/src/install_ros2/setup.bash
source Simulator/src/install_ros2/setup.bash
ros2 launch yopo_bringup system.launch.py planner_method:=super_compat
```

Defaults: `/lidar_points` + `world_frame_cloud:=false` (body-frame LiDAR, same as YOPO). See `SUPER_COMPAT.md` for world-frame pairing.

Smoke test script:

```bash
bash Controller/src/utils/yopo_bringup/scripts/test_super_compat.sh
```

Parity regression script (planner should already be running):

```bash
python3 Controller/src/utils/yopo_bringup/scripts/super_parity_regression.py \
  --output super_parity_report.json
```

Or one-shot full-stack regression:

```bash
bash Controller/src/utils/yopo_bringup/scripts/run_super_parity_regression.sh
```

See `SUPER_COMPAT.md` for adaptation details.

## 16. AirSim adaptation guide

For integrating this project with AirSim, see:

- `AIRSIM_ADAPTATION_TUTORIAL.md`
