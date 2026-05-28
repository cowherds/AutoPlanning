# Visualization Assets

This directory is the centralized visualization entry for `YOPO_lidar_V3`.

## Recommended RViz profiles

- `visualization/yopo_ros2.rviz`: default profile for ROS2 runtime.
- `visualization/yopo.rviz`: legacy-compatible profile.

## Quick start

```bash
cd /home/duckcity/YOPO_lidar_V3
rviz2 -d visualization/yopo_ros2.rviz
```

## Related package-local RViz files

Some package-local RViz configs are still kept in package folders for build/runtime compatibility:

- `Simulator/src/rviz.rviz`
- `Controller/src/so3_quadrotor_simulator/config/rviz.rviz`
