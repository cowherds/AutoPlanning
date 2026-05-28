# 项目 ROS2 Topic 解读

## 0. 扫描范围与说明

- 扫描方式：基于仓库源码静态分析（`create_publisher` / `create_subscription` / launch remap / 默认参数配置）。
- 主要覆盖模块：`Simulator`、`Controller`、`policy/YOPO`、`policy/super`。

---

## 1. ROS2 Topic 全量清单

> 说明：下表是本项目 ROS2 代码路径中可见的 topic。重点感知 topic（激光雷达/点云/里程计/IMU）已明确标注。

| Topic | 消息类型 | 备注 | 发布者（Publisher） | 订阅者（Subscriber） | 数据格式（关键字段） | 字段类型（核心） | 样例（简版） |
|---|---|---|---|---|---|---|---|
| `/lidar_points` | `sensor_msgs/msg/PointCloud2` | 激光雷达点云（机体系） | `sensor_simulator_node_ros2` | `yopo_net`、`super_compat_planner`（按参数） | `PointCloud2{header, fields[x,y,z], data}` | `height/width:uint32, fields[].name:string, fields[].datatype:uint8, data:uint8[], is_dense:bool` | `frame_id:"body", width:3, fields:[x,y,z], data:"<bytes>"` |
| `/lidar_points_world` | `sensor_msgs/msg/PointCloud2` | 激光雷达点云（世界系） | `sensor_simulator_node_ros2` | `yopo_net`、`super_compat_planner`（切换到 world-frame 模式时） | `PointCloud2{header, fields[x,y,z], data}` | `height/width:uint32, fields[].name:string, fields[].datatype:uint8, data:uint8[], is_dense:bool` | `frame_id:"world", width:3, fields:[x,y,z], data:"<bytes>"` |
| `/mock_map` | `sensor_msgs/msg/PointCloud2` | 地图点云（可视化） | `sensor_simulator_node_ros2` | 可视化/调试端（按需） | `PointCloud2{header, fields[x,y,z], data}` | `height/width:uint32, fields[].name:string, fields[].datatype:uint8, data:uint8[]` | `frame_id:"world", width:N, fields:[x,y,z]` |
| `/sim/odom` | `nav_msgs/msg/Odometry` | 里程计（仿真） | `quadrotor_simulator_so3`（或测试脚本 `sim_odom.py`） | `sensor_simulator_node_ros2`、`yopo_net`、`super_compat_planner`、`network_control_node`（经 remap） | `Odometry{header, child_frame_id, pose.pose, twist.twist}` | `child_frame_id:string, pose.position/quat:float64, twist.linear/angular:float64, covariance:float64[36]` | `frame_id:"world", child_frame_id:"quadrotor", position:{x:1.2,y:-0.4,z:2.8}` |
| `/sim/imu` | `sensor_msgs/msg/Imu` | 惯导 IMU（仿真） | `quadrotor_simulator_so3` | `network_control_node`（经 remap） | `Imu{header, orientation, angular_velocity, linear_acceleration}` | `orientation/ang_vel/lin_acc:float64, covariance:float64[9]` | `orientation:{z:0.38,w:0.92}, angular_velocity:{z:0.12}` |
| `/vins_estimator/imu_propagate` | `nav_msgs/msg/Odometry` | 里程计（实机 VINS） | 外部 VINS 节点 | `network_control_node`（实机 launch remap） | `Odometry{header, child_frame_id, pose.pose, twist.twist}` | `child_frame_id:string, pose.position/quat:float64, twist.linear/angular:float64, covariance:float64[36]` | `frame_id:"world", child_frame_id:"vins_body", velocity:{x:2.1,y:0.0,z:0.0}` |
| `/mavros/imu/data_raw` | `sensor_msgs/msg/Imu` | 惯导 IMU（实机） | 外部 MAVROS | `network_control_node`（实机 launch remap） | `Imu{header, orientation, angular_velocity, linear_acceleration}` | `orientation/ang_vel/lin_acc:float64, covariance:float64[9]` | `orientation:{x:0,y:0,z:0,w:1}, linear_acceleration:{z:9.8}` |
| `/move_base_simple/goal` | `geometry_msgs/msg/PoseStamped` | 目标点（RViz） | RViz/上位机 | `yopo_net`、`super_compat_planner` | `PoseStamped{header, pose.position, pose.orientation}` | `position:float64, orientation(quat):float64, frame_id:string` | `frame_id:"world", position:{x:20.0,y:5.0,z:0.0}` |
| `/so3_control/pos_cmd` | `quadrotor_msgs/msg/PositionCommand` | 位置控制指令 | `yopo_net` 或 `super_compat_planner` | `network_control_node`（remap 到 `position_cmd`） | `PositionCommand{position, velocity, acceleration, yaw, yaw_dot, trajectory_flag}` | `position/velocity/acc:float64, yaw/yaw_dot:float64, kx/kv:float64[3], trajectory_id:uint32, trajectory_flag:uint8` | `position:{x:8,y:1.5,z:3}, velocity:{x:3,y:0.2,z:0}, trajectory_flag:1` |
| `so3_cmd` | `quadrotor_msgs/msg/SO3Command` | 姿态与力控制指令 | `network_control_node` | `quadrotor_simulator_so3`（remap 到 `cmd`） | `SO3Command{force, orientation, kR, kOm, aux}` | `force/quat:float64, kR/kOm:float64[3], aux.current_yaw:float64, aux.enable_motors:bool` | `force:{z:9.8}, orientation:{z:0.17,w:0.98}, aux:{enable_motors:true}` |
| `force_disturbance` | `geometry_msgs/msg/Vector3` | 外部扰动力输入 | 外部注入节点（可选） | `quadrotor_simulator_so3` | `Vector3{x,y,z}` | `x/y/z:float64` | `{x:0.0,y:0.0,z:0.0}` |
| `moment_disturbance` | `geometry_msgs/msg/Vector3` | 外部扰动力矩输入 | 外部注入节点（可选） | `quadrotor_simulator_so3` | `Vector3{x,y,z}` | `x/y/z:float64` | `{x:0.0,y:0.0,z:0.0}` |
| `uav` | `visualization_msgs/msg/Marker` | 无人机模型可视化 | `quadrotor_simulator_so3` | RViz（可选） | `Marker{header, ns,id,type,pose,scale,color,mesh_resource}` | `ns:string, id/type/action:int32, pose/scale:float64, color:float32, mesh_resource:string, bool` | `type:MESH_RESOURCE, pose.position:{x:1.2,y:-0.4,z:2.8}` |
| `/super_compat/planned_path` | `nav_msgs/msg/Path` | 规划路径可视化 | `super_compat_planner` | RViz/调试端（可选） | `Path{header, poses[]}` | `poses:PoseStamped[] (position/orientation:float64), frame_id:string` | `frame_id:"world", poses:[{position:{x:1,y:1,z:3}}, ...]` |
| `/super_compat/diag/fsm_state` | `std_msgs/msg/String` | 诊断：状态机状态 | `super_compat_planner` | 调试端（可选） | `String{data}` | `data:string` | `data:"FOLLOW_TRAJ"` |
| `/super_compat/diag/plan_ret` | `std_msgs/msg/String` | 诊断：规划返回码 | `super_compat_planner` | 调试端（可选） | `String{data}` | `data:string` | `data:"replan_once:SUCCESS"` |
| `/super_compat/diag/metrics` | `std_msgs/msg/Float32MultiArray` | 诊断：数值指标数组 | `super_compat_planner` | 调试端（可选） | `Float32MultiArray{layout,data[]}` | `layout.dim[].label:string, size/stride:uint32, data_offset:uint32, data:float32[]` | `data:[d_goal,d_path,speed,...]` |
| `/yopo_net/lattice_trajs_visual` | `sensor_msgs/msg/PointCloud2` | 轨迹可视化（lattice） | `yopo_net` | RViz/调试端（可选） | `PointCloud2{header, fields[x,y,z], data}` | `height/width:uint32, fields[].name:string, fields[].datatype:uint8, data:uint8[]` | `frame_id:"world", fields:[x,y,z]` |
| `/yopo_net/best_traj_visual` | `sensor_msgs/msg/PointCloud2` | 轨迹可视化（best） | `yopo_net` | RViz/调试端（可选） | `PointCloud2{header, fields[x,y,z], data}` | `height/width:uint32, fields[].name:string, fields[].datatype:uint8, data:uint8[]` | `frame_id:"world", fields:[x,y,z]` |
| `/yopo_net/trajs_visual` | `sensor_msgs/msg/PointCloud2` | 轨迹可视化（all+score） | `yopo_net` | RViz/调试端（可选） | `PointCloud2{header, fields[x,y,z,intensity], data}` | `height/width:uint32, fields[].name:string, intensity:float32, data:uint8[]` | `frame_id:"world", fields:[x,y,z,intensity]` |

### 1.1 Launch remap 前的节点内部 topic 名

以下名称也在源码中直接出现，但在默认 launch 中通常会被 remap 到上表的绝对路径：

| 节点内部名 | 默认 remap 后 |
|---|---|
| `odom` | `/sim/odom`（仿真）或 `/vins_estimator/imu_propagate`（实机） |
| `imu` | `/sim/imu`（仿真）或 `/mavros/imu/data_raw`（实机） |
| `position_cmd` | `/so3_control/pos_cmd` |
| `cmd` | `so3_cmd` |

---

## 2. 重点问题直答：深度相机、激光雷达等感知 Topic

### 2.1 激光雷达（LiDAR）

- `/lidar_points`：**使用**（默认感知输入）。
- `/lidar_points_world`：**使用**（可选 world-frame 配置）。
- 项目当前感知主链路是 LiDAR 点云（非深度图）。

### 2.2 深度相机（Depth Camera）

- **未发现 ROS2 深度相机 topic 的发布/订阅**（如 `/camera/depth/image_raw`、`/camera/depth/points` 等）。
- YOPO 脚本中存在 `sensor_mode` 参数说明，但当前实现为 LiDAR-only 路径。

### 2.3 其他感知输入

- `/sim/odom`、`/vins_estimator/imu_propagate`（里程计）和 `/sim/imu`、`/mavros/imu/data_raw`（IMU）均被控制/规划链路使用，属于状态估计类感知输入。

---

## 3. 每个 Topic 的数据格式说明（并标注发布/订阅角色）

> 下面按“感知优先”列出格式重点；同一消息类型在多个 topic 复用时，格式与字段类型一致。

### 3.1 `/lidar_points`（`PointCloud2`）
- 角色：`sensor_simulator_node_ros2` 发布；`yopo_net` / `super_compat_planner` 订阅。
- 关键字段：`header`、`height`、`width`、`fields(x,y,z)`、`point_step`、`row_step`、`data`、`is_dense`。
- 字段类型（核心）：`height/width:uint32`，`fields[].name:string`，`fields[].offset:uint32`，`fields[].datatype:uint8`，`data:uint8[]`，`is_dense:bool`。

### 3.2 `/lidar_points_world`（`PointCloud2`）
- 角色：`sensor_simulator_node_ros2` 发布；规划节点按配置订阅。
- 与 `/lidar_points` 格式一致，仅语义上 `frame_id=world`。
- 字段类型（核心）：同 `3.1`（`PointCloud2` 标准类型）。

### 3.3 `/mock_map`（`PointCloud2`）
- 角色：`sensor_simulator_node_ros2` 发布。
- 格式同 `PointCloud2`，用于地图点云可视化。
- 字段类型（核心）：同 `3.1`（`PointCloud2` 标准类型）。

### 3.4 `/sim/odom`、`/vins_estimator/imu_propagate`（`Odometry`）
- 角色：
  - `/sim/odom`：仿真器发布，多节点订阅；
  - `/vins_estimator/imu_propagate`：外部 VINS 发布，控制器订阅（实机）。
- 关键字段：`header`、`child_frame_id`、`pose.pose`、`twist.twist`（及可选协方差）。
- 字段类型（核心）：`child_frame_id:string`，`pose.position/pose.orientation:float64`，`twist.linear/angular:float64`，`pose.covariance/twist.covariance:float64[36]`。

### 3.5 `/sim/imu`、`/mavros/imu/data_raw`（`Imu`）
- 角色：
  - `/sim/imu`：仿真器发布，控制器订阅；
  - `/mavros/imu/data_raw`：外部 MAVROS 发布，控制器订阅（实机）。
- 关键字段：`orientation`、`angular_velocity`、`linear_acceleration`（及协方差）。
- 字段类型（核心）：`orientation/ang_vel/lin_acc:float64`，`orientation_covariance/angular_velocity_covariance/linear_acceleration_covariance:float64[9]`。

### 3.6 `/move_base_simple/goal`（`PoseStamped`）
- 角色：RViz/上位机发布；规划器订阅。
- 关键字段：`header.frame_id`、`pose.position`、`pose.orientation`。
- 字段类型（核心）：`frame_id:string`，`position.x/y/z:float64`，`orientation.x/y/z/w:float64`。

### 3.7 `/so3_control/pos_cmd`（`quadrotor_msgs/PositionCommand`）
- 角色：`yopo_net` 或 `super_compat_planner` 发布；`network_control_node` 订阅。
- 关键字段：`position`、`velocity`、`acceleration`、`yaw`、`yaw_dot`、`trajectory_id`、`trajectory_flag`、`kx`、`kv`。
- 字段类型（核心）：`position/velocity/acceleration:float64`，`yaw/yaw_dot:float64`，`kx/kv:float64[3]`，`trajectory_id:uint32`，`trajectory_flag:uint8`。

### 3.8 `so3_cmd`（`quadrotor_msgs/SO3Command`）
- 角色：`network_control_node` 发布；`quadrotor_simulator_so3` 订阅。
- 关键字段：`force`、`orientation`、`kR`、`kOm`、`aux(current_yaw, kf_correction, angle_corrections, enable_motors, use_external_yaw)`。
- 字段类型（核心）：`force/orientation:float64`，`kR/kOm:float64[3]`，`aux.current_yaw/kf_correction:float64`，`aux.angle_corrections:float64[2]`，`aux.enable_motors/use_external_yaw:bool`。

### 3.9 `force_disturbance` / `moment_disturbance`（`Vector3`）
- 角色：外部扰动源发布（可选）；仿真器订阅。
- 关键字段：`x`、`y`、`z`。
- 字段类型（核心）：`x/y/z:float64`。

### 3.10 `uav`（`Marker`）
- 角色：仿真器发布，RViz 订阅。
- 关键字段：`header`、`ns`、`id`、`type`、`action`、`pose`、`scale`、`color`、`mesh_resource`。
- 字段类型（核心）：`ns:string`，`id/type/action:int32`，`pose/scale:float64`，`color.rgba:float32`，`mesh_resource:string`，`mesh_use_embedded_materials:bool`。

### 3.11 `/super_compat/planned_path`（`Path`）
- 角色：`super_compat_planner` 发布。
- 关键字段：`header`、`poses[]`（每个元素为 `PoseStamped`）。
- 字段类型（核心）：`poses:PoseStamped[]`（数组结构），元素中 `position/orientation` 为 `float64`，`frame_id:string`。

### 3.12 `/super_compat/diag/fsm_state`、`/super_compat/diag/plan_ret`（`String`）
- 角色：`super_compat_planner` 发布。
- 关键字段：`data`。
- 字段类型（核心）：`data:string`。

### 3.13 `/super_compat/diag/metrics`（`Float32MultiArray`）
- 角色：`super_compat_planner` 发布。
- 关键字段：`layout`、`data[]`。
- 字段类型（核心）：`layout.dim[].label:string`，`layout.dim[].size/stride:uint32`，`layout.data_offset:uint32`，`data:float32[]`。

### 3.14 `/yopo_net/lattice_trajs_visual`、`/yopo_net/best_traj_visual`、`/yopo_net/trajs_visual`（`PointCloud2`）
- 角色：`yopo_net` 发布（可视化）。
- 关键字段：同 `PointCloud2`；`/yopo_net/trajs_visual` 常包含 `intensity` 字段用于评分展示。
- 字段类型（核心）：同 `3.1`，其中 `/yopo_net/trajs_visual` 常见 `intensity:float32` 字段。

---

## 4. 每个 Topic 的数据样例（可直接对照）

> 以下不再使用 YAML 展示，而是直接使用项目中的真实代码结构：C++ 节点用 C++，Python 节点用 Python。

### 4.1 `sensor_msgs/msg/PointCloud2`（用于 `/lidar_points`、`/lidar_points_world`、`/mock_map`、YOPO 可视化点云）

```cpp
// C++ 发布（Simulator/src/src/sensor_simulator_ros2.cpp）
sensor_msgs::msg::PointCloud2 output;
pcl::toROSMsg(lidar_points, output);
output.header.stamp = stamp;            // builtin_interfaces::msg::Time
output.header.frame_id = lidar_frame_id_; // std::string
point_cloud_pub_->publish(output);      // /lidar_points

sensor_msgs::msg::PointCloud2 output_world;
pcl::toROSMsg(lidar_points_world, output_world);
output_world.header.stamp = stamp;
output_world.header.frame_id = "world";
point_cloud_world_pub_->publish(output_world); // /lidar_points_world
```

```python
# Python 订阅解析（policy/YOPO/script/test_yopo_ros.py）
n_points = msg.width * msg.height             # uint32 * uint32
fields = {f.name: f.offset for f in msg.fields}  # PointField[]
cloud_dtype = np.dtype({
    "names": ["x", "y", "z"],
    "formats": ["<f4", "<f4", "<f4"],        # float32
    "offsets": [fields["x"], fields["y"], fields["z"]],
    "itemsize": msg.point_step,               # uint32
})
structured = np.frombuffer(msg.data, dtype=cloud_dtype, count=n_points)  # data: uint8[]
```

### 4.2 `nav_msgs/msg/Odometry`（用于 `/sim/odom`、`/vins_estimator/imu_propagate`）

```cpp
// C++ 发布（Controller/src/so3_quadrotor_simulator/src/quadrotor_simulator_ros2.cpp）
nav_msgs::msg::Odometry odom_msg;
odom_msg.header.stamp = tnow;            // Time
odom_msg.header.frame_id = "world";      // string
odom_msg.child_frame_id = quad_name_;    // string
odom_msg.pose.pose.position.x = state.x(0);      // float64
odom_msg.pose.pose.orientation.w = q.w();        // float64
odom_msg.twist.twist.linear.x = state.v(0);      // float64
odom_msg.twist.twist.angular.z = state.omega(2); // float64
odom_pub_->publish(odom_msg);
```

```python
# Python 订阅读取（policy/super/super_compat_planner.py）
def _on_odom(self, msg: Odometry) -> None:
    self.odom = msg
    p = msg.pose.pose.position      # Point(float64 x/y/z)
    v = msg.twist.twist.linear      # Vector3(float64 x/y/z)
    q = msg.pose.pose.orientation   # Quaternion(float64 x/y/z/w)
```

### 4.3 `sensor_msgs/msg/Imu`（用于 `/sim/imu`、`/mavros/imu/data_raw`）

```cpp
// C++ 发布（Controller/src/so3_quadrotor_simulator/src/quadrotor_simulator_ros2.cpp）
sensor_msgs::msg::Imu imu;
imu.header.stamp = tnow;                // Time
imu.header.frame_id = "world";          // string
imu.orientation.w = q.w();              // float64
imu.angular_velocity.z = state.omega(2);// float64
imu.linear_acceleration.x = quad.getAcc()[0]; // float64
imu_pub_->publish(imu);
```

```cpp
// C++ 订阅读取（Controller/src/so3_control/src/network_control_ros2.cpp）
void imuCallback(const sensor_msgs::msg::Imu::SharedPtr imu) {
  Eigen::Vector3d acc(
      imu->linear_acceleration.x,  // float64
      imu->linear_acceleration.y,  // float64
      imu->linear_acceleration.z); // float64
}
```

### 4.4 `geometry_msgs/msg/PoseStamped`（用于 `/move_base_simple/goal`）

```python
# Python 订阅读取（policy/super/super_compat_planner.py）
def _on_goal(self, msg: PoseStamped) -> None:
    frame = (msg.header.frame_id or "").strip().lstrip("/")  # string
    gx = msg.pose.position.x  # float64
    gy = msg.pose.position.y  # float64
    self._apply_goal_xy(gx, gy, "rviz")
```

### 4.5 `quadrotor_msgs/msg/PositionCommand`（用于 `/so3_control/pos_cmd`）

```python
# Python 发布（policy/YOPO/script/test_yopo_ros.py）
control_msg = self.PositionCommand()
control_msg.header.stamp = self.ros.now()   # Time
control_msg.trajectory_flag = control_msg.TRAJECTORY_STATUS_READY  # uint8
control_msg.position.x = px       # float64
control_msg.position.y = py       # float64
control_msg.position.z = self.flight_height # float64
control_msg.velocity.x = vx       # float64
control_msg.acceleration.x = ax   # float64
control_msg.yaw = yaw             # float64
control_msg.yaw_dot = yaw_dot     # float64
self.ctrl_pub.publish(control_msg)
```

### 4.6 `quadrotor_msgs/msg/SO3Command`（用于 `so3_cmd`）

```cpp
// C++ 发布（Controller/src/so3_control/src/network_control_ros2.cpp）
quadrotor_msgs::msg::SO3Command cmd;
cmd.header.stamp = this->now();   // Time
cmd.force.x = force(0);           // float64
cmd.orientation.w = quat.w();     // float64
cmd.k_r = {1.5, 1.5, 1.0};        // float64[3]
cmd.k_om = {0.13, 0.13, 0.1};     // float64[3]
cmd.aux.current_yaw = cur_yaw;    // float64
cmd.aux.enable_motors = true;     // bool
cmd.aux.use_external_yaw = false; // bool
so3_command_pub_->publish(cmd);
```

### 4.7 `geometry_msgs/msg/Vector3`（用于 `force_disturbance`、`moment_disturbance`）

```cpp
// C++ 订阅读取（Controller/src/so3_quadrotor_simulator/src/quadrotor_simulator_ros2.cpp）
void forceDisturbanceCallback(const geometry_msgs::msg::Vector3::SharedPtr f) {
  disturbance_.f = Eigen::Vector3d(f->x, f->y, f->z); // x/y/z: float64
}

void momentDisturbanceCallback(const geometry_msgs::msg::Vector3::SharedPtr m) {
  disturbance_.m = Eigen::Vector3d(m->x, m->y, m->z); // x/y/z: float64
}
```

### 4.8 `visualization_msgs/msg/Marker`（用于 `uav`）

```cpp
// C++ 发布（Controller/src/so3_quadrotor_simulator/src/quadrotor_simulator_ros2.cpp）
visualization_msgs::msg::Marker mesh;
mesh.header = odom_msg.header;                    // Header
mesh.header.frame_id = "world";                   // string
mesh.ns = "mesh";                                 // string
mesh.id = 0;                                      // int32
mesh.type = visualization_msgs::msg::Marker::MESH_RESOURCE; // int32 enum
mesh.action = visualization_msgs::msg::Marker::ADD;         // int32 enum
mesh.pose = odom_msg.pose.pose;                   // Pose(float64)
mesh.scale.x = 2.0;                               // float64
mesh.color.a = 1.0f;                              // float32
mesh.mesh_resource = mesh_resource_;              // string
mesh.mesh_use_embedded_materials = true;          // bool
mesh_pub_->publish(mesh);
```

### 4.9 `nav_msgs/msg/Path`（用于 `/super_compat/planned_path`）

```python
# Python 发布（policy/super/super_compat_planner.py）
path = Path()
path.header.frame_id = "world"                    # string
path.header.stamp = self.get_clock().now().to_msg()  # Time

ps = PoseStamped()
ps.header = path.header
ps.pose.position.x = x     # float64
ps.pose.position.y = y     # float64
ps.pose.position.z = self.cfg.flight_height  # float64
ps.pose.orientation.w = 1.0  # float64
path.poses.append(ps)      # PoseStamped[]
self.path_pub.publish(path)
```

### 4.10 `std_msgs/msg/String`（用于 `/super_compat/diag/fsm_state`、`/super_compat/diag/plan_ret`）

```python
# Python 发布（policy/super/super_compat_planner.py）
state_msg = String()
state_msg.data = self.fsm.label()    # string
self.diag_state_pub.publish(state_msg)

plan_msg = String()
plan_msg.data = f"{self._last_plan_mode}:{self._last_plan_ret.name}"  # string
self.diag_plan_pub.publish(plan_msg)
```

### 4.11 `std_msgs/msg/Float32MultiArray`（用于 `/super_compat/diag/metrics`）

```python
# Python 发布（policy/super/super_compat_planner.py）
metrics = Float32MultiArray()
metrics.data = [
    float(d_goal),        # float32[] 元素（Python 侧用 float，序列化到 float32）
    float(d_path),
    float(math.hypot(vx, vy)),
    float(self._unsafe_hold_count),
    float(self._emer_count),
    float(1.0 if self.core.state.connected_to_goal else 0.0),
]
self.diag_metrics_pub.publish(metrics)
```

