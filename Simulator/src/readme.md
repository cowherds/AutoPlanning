# CUDA LiDAR simulator (ROS2)

ROS2 / Humble + CUDA. Publishes `sensor_msgs/PointCloud2` on `lidar_topic` (default `/lidar_points`, body frame) and `lidar_world_topic` (default `/lidar_points_world`, world frame for RViz).  
Depth images and ROS1 nodes were removed in this fork.

## Dependencies

- CUDA, ROS2 Humble, PCL, **yaml-cpp**  
  `sudo apt-get install ros-humble-desktop libyaml-cpp-dev`

## Build (colcon)

From repository root:

```bash
source /opt/ros/humble/setup.bash
bash scripts/build_simulator_ros2.sh
```

Or manually:

```bash
cd Simulator/src
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select sensor_simulator
```

If `nvcc` complains about GPU arch, set e.g. `export YOPO_CUDA_ARCH=86` before building. See `Simulator/src/CMakeLists.txt`.

Overlay:

```bash
source Simulator/src/install_ros2/setup.bash
```

## Run

```bash
ros2 run sensor_simulator sensor_simulator
```

LiDAR rate is controlled by `lidar_fps` in [`config/config.yaml`](config/config.yaml).

### Dataset generation (no PNG, only `lidar_*.bin`)

```bash
source /opt/ros/humble/setup.bash
source Simulator/src/install_ros2/setup.bash
ros2 run sensor_simulator dataset_generator
```

### Optional test odometry (ROS2)

```bash
python3 sim_odom.py
```

(Prefer the full stack `quadrotor_simulator_so3` + control for closed-loop tests.)

## Configuration

Edit [`config/config.yaml`](config/config.yaml): `odom_topic`, `lidar_topic`, `lidar_world_topic`, `lidar_fps`, map randomization, `lidar:` ray pattern, and `save_path` / `image_num` for dataset generation.

## Scenes

See `img/` for example environments (forest, maze, perlin, etc.). Map generation follows HKUST mockamap-style utilities in `maps.cpp`.
