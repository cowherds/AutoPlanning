import std_msgs.msg
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from threading import Lock
from sensor_msgs.msg import PointCloud2, PointField

import time
import torch
import numpy as np
import argparse
import sys
from pathlib import Path
from scipy.spatial.transform import Rotation as R

# Make algorithm root importable when launcher lives under script/.
_ALGO_ROOT = Path(__file__).resolve().parents[1]
_POLICY_ROOT = _ALGO_ROOT.parent
for _p in (_ALGO_ROOT, _POLICY_ROOT):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from config.config import cfg
from utils.flight_constraints import (
    DEFAULT_FLIGHT_HEIGHT,
    MAX_FLIGHT_HEIGHT,
    MIN_FLIGHT_HEIGHT,
    clamp_position_xyz,
    clamp_xy,
    cruise_height,
    flat_z_poly_solver,
    make_3d_goal,
)
from policy.yopo_network import YopoNetwork
from policy.poly_solver import *
from policy.state_transform import *
from ros_compat import make_ros_adapter, import_point_cloud2, import_position_command

try:
    from torch2trt import TRTModule
except ImportError:
    print("tensorrt not found.")


def resolve_weight_path(weight_arg):
    """Resolve weight path from common working directories."""
    repo_root = _ALGO_ROOT.parents[1]
    if not weight_arg:
        auto_candidates = sorted((_ALGO_ROOT / "saved").glob("**/*.pth"))
        if auto_candidates:
            return str(auto_candidates[-1].resolve())
        raise FileNotFoundError(
            "Model weight not found: no --weight was provided and no .pth files were found under "
            f"{_ALGO_ROOT / 'saved'}.\n"
            "Put a trained checkpoint under policy/YOPO/saved/<run_name>/, or run with "
            "--weight /absolute/path/to/model.pth"
        )

    input_path = Path(weight_arg).expanduser()
    candidates = []

    if input_path.is_absolute():
        candidates.append(input_path)
    else:
        candidates.extend(
            [
                Path.cwd() / input_path,
                _ALGO_ROOT / input_path,
                repo_root / input_path,
            ]
        )
        parts = input_path.parts
        if len(parts) >= 2 and parts[0] == "policy" and parts[1] == "YOPO":
            candidates.append(_ALGO_ROOT / Path(*parts[2:]))
        if len(parts) >= 1 and parts[0] == "YOPO":
            candidates.append(_ALGO_ROOT / Path(*parts[1:]))

    uniq = []
    seen = set()
    for cand in candidates:
        normalized = cand.resolve(strict=False)
        key = str(normalized)
        if key in seen:
            continue
        seen.add(key)
        uniq.append(normalized)
        if normalized.is_file():
            return str(normalized)

    searched = "\n".join(f"  - {p}" for p in uniq)
    raise FileNotFoundError(f"Model weight not found: {weight_arg}\nSearched paths:\n{searched}")


def str2bool(value):
    if isinstance(value, bool):
        return value
    lower = value.lower()
    if lower in {"true", "1", "yes", "y", "on"}:
        return True
    if lower in {"false", "0", "no", "n", "off"}:
        return False
    raise argparse.ArgumentTypeError(f"Invalid bool value: {value}")


class YopoNet:
    def __init__(self, config, weight):
        self.config = config
        self.ros_version, self.ros = make_ros_adapter("yopo_net", force_version=self.config.get("ros_version"))
        print(f"YOPO using {self.ros_version.upper()} backend.")
        self._apply_default_topics()
        self.point_cloud2 = import_point_cloud2(self.ros_version)
        self.PositionCommand = import_position_command(self.ros_version)

        cfg["train"] = False
        self.device = "cuda" if torch.cuda.is_available() else "cpu"

        self.num_points = int(cfg.get('pointcloud_num_points', 4096))
        self.point_feature_dim = int(cfg.get('pointcloud_feature_dim', 4))
        self.lidar_vertical_fov = cfg['lidar_vertical_fov']
        self.lidar_sensing_horizon = cfg['lidar_sensing_horizon']
        self.min_dis = cfg.get('lidar_sensing_blind', 0.1)

        self.flight_height = cruise_height(
            float(self.config.get("flight_height", DEFAULT_FLIGHT_HEIGHT))
        )
        self.max_speed = float(cfg.get("velocity", 10.0))
        self.goal = self._make_2d_goal(self.config['goal'])
        self.plan_from_reference = self.config['plan_from_reference']
        self.use_trt = self.config['use_tensorrt']
        self.verbose = self.config['verbose']
        self.visualize = self.config['visualize']

        self.odom = Odometry()
        self.odom_init = False
        self.last_yaw = 0.0
        self.ctrl_dt = 0.02
        self.ctrl_time = None
        self.desire_init = False
        self.arrive = False
        self.desire_pos = None
        self.desire_vel = None
        self.desire_acc = None
        self.optimal_poly_x = None
        self.optimal_poly_y = None
        self.optimal_poly_z = None
        self.lock = Lock()
        self.last_control_msg = None
        self.state_transform = StateTransform()
        self.lattice_primitive = LatticePrimitive.get_instance()
        self.traj_time = self.lattice_primitive.segment_time

        self.time_forward = 0.0
        self.time_process = 0.0
        self.time_prepare = 0.0
        self.time_interpolation = 0.0
        self.time_visualize = 0.0
        self.count = 0
        self.sensor_fps = max(float(self.config.get("sensor_fps", 10.0)), 1.0)

        if self.use_trt:
            self.policy = TRTModule()
            self.policy.load_state_dict(torch.load(weight))
        else:
            state_dict = torch.load(weight, weights_only=True)
            self.policy = YopoNetwork()
            self.policy.load_state_dict(state_dict)
            self.policy = self.policy.to(self.device)
            self.policy.eval()
        self.warm_up()

        self.lattice_traj_pub = self.ros.create_publisher(PointCloud2, "/yopo_net/lattice_trajs_visual", queue_size=1)
        self.best_traj_pub = self.ros.create_publisher(PointCloud2, "/yopo_net/best_traj_visual", queue_size=1)
        self.all_trajs_pub = self.ros.create_publisher(PointCloud2, "/yopo_net/trajs_visual", queue_size=1)
        self.ctrl_pub = self.ros.create_publisher(self.PositionCommand, self.config["ctrl_topic"], queue_size=1)

        self.odom_sub = self.ros.create_subscription(
            Odometry, self.config["odom_topic"], self.callback_odometry, queue_size=1)

        lidar_topic = self.config.get("lidar_topic", "/lidar_points")
        self.lidar_sub = self.ros.create_subscription(
            PointCloud2, lidar_topic, self.callback_lidar, queue_size=1)
        print(f"Subscribing to LiDAR topic: {lidar_topic}")

        self.goal_sub = self.ros.create_subscription(
            PoseStamped, "/move_base_simple/goal", self.callback_set_goal, queue_size=1)

        self.ros.sleep(1.0)
        self.timer_ctrl = self.ros.create_timer(self.ctrl_dt, self.control_pub)
        print(
            f"Topics | odom: {self.config['odom_topic']} | lidar: {self.config['lidar_topic']} | "
            f"ctrl: {self.config['ctrl_topic']}"
        )
        print("YOPO Net Node Ready! Sensor: LiDAR point cloud (PTv3-style)")
        self.ros.spin()

    def _apply_default_topics(self):
        default_topics = {
            "odom_topic": "/sim/odom",
            "lidar_topic": "/lidar_points",
            "ctrl_topic": "/so3_control/pos_cmd",
        }
        for key, value in default_topics.items():
            if not self.config.get(key):
                self.config[key] = value

    def _make_2d_goal(self, goal_xy):
        goal = np.asarray(goal_xy, dtype=float)
        if goal.shape[0] < 2:
            raise ValueError("goal must contain at least x and y")
        return make_3d_goal(goal[:2], self.flight_height)

    def callback_set_goal(self, data):
        self.goal = self._make_2d_goal([data.pose.position.x, data.pose.position.y])
        self.arrive = False
        print(f"New 2D Goal: ({data.pose.position.x:.1f}, {data.pose.position.y:.1f}), height={self.flight_height:.1f}m")

    def callback_odometry(self, data):
        self.odom = data
        if not self.desire_init:
            self.desire_pos = clamp_position_xyz(
                (
                    self.odom.pose.pose.position.x,
                    self.odom.pose.pose.position.y,
                    self.odom.pose.pose.position.z,
                ),
                flight_height=self.flight_height,
            )
            self.desire_vel = np.array(
                (
                    self.odom.twist.twist.linear.x,
                    self.odom.twist.twist.linear.y,
                    0.0,
                )
            )
            self.desire_acc = np.array((0.0, 0.0, 0.0))
            ypr = R.from_quat([self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y,
                               self.odom.pose.pose.orientation.z, self.odom.pose.pose.orientation.w]).as_euler('ZYX', degrees=False)
            self.last_yaw = ypr[0]
        self.odom_init = True

        pos_xy = np.array((self.odom.pose.pose.position.x, self.odom.pose.pose.position.y))
        if np.linalg.norm(pos_xy - self.goal[:2]) < 5 and not self.arrive:
            print("Arrive!")
            self.arrive = True

    def process_odom(self):
        Rotation_wb = R.from_quat([self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y,
                                   self.odom.pose.pose.orientation.z, self.odom.pose.pose.orientation.w]).as_matrix()
        Rotation_wc = Rotation_wb
        Rotation_cw = Rotation_wc.T

        vel_w = self.desire_vel if self.plan_from_reference else np.array(
            [self.odom.twist.twist.linear.x, self.odom.twist.twist.linear.y, self.odom.twist.twist.linear.z])
        vel_c = np.dot(Rotation_cw, vel_w)
        acc_w = self.desire_acc
        acc_c = np.dot(Rotation_cw, acc_w)

        goal_w = self.goal - self.desire_pos
        goal_c = np.dot(Rotation_cw, goal_w)

        obs = np.concatenate((vel_c, acc_c, goal_c), axis=0).astype(np.float32)
        obs_norm = self.state_transform.normalize_obs(torch.from_numpy(obs[None, :]))
        return obs_norm, Rotation_wc

    def _pointcloud2_to_numpy(self, msg):
        n_points = msg.width * msg.height
        if n_points == 0:
            return np.zeros((0, 3), dtype=np.float32)

        fields = {f.name: f.offset for f in msg.fields}
        if not {"x", "y", "z"}.issubset(fields):
            raise ValueError("PointCloud2 message does not contain x/y/z fields")

        cloud_dtype = np.dtype({
            "names": ["x", "y", "z"],
            "formats": ["<f4", "<f4", "<f4"],
            "offsets": [fields["x"], fields["y"], fields["z"]],
            "itemsize": msg.point_step,
        })
        structured = np.frombuffer(msg.data, dtype=cloud_dtype, count=n_points)
        return np.stack((structured["x"], structured["y"], structured["z"]), axis=-1).astype(np.float32, copy=False)

    def _prepare_point_features(self, points):
        valid = np.isfinite(points).all(axis=1)
        points = points[valid]
        if points.shape[0] == 0:
            points = np.zeros((1, 3), dtype=np.float32)

        depth = np.linalg.norm(points, axis=1)
        valid_depth = (depth > self.min_dis) & (depth < self.lidar_sensing_horizon)
        points = points[valid_depth]
        depth = depth[valid_depth]
        if points.shape[0] == 0:
            points = np.zeros((1, 3), dtype=np.float32)
            depth = np.zeros((1,), dtype=np.float32)

        if points.shape[0] >= self.num_points:
            sample_idx = np.random.choice(points.shape[0], self.num_points, replace=False)
        else:
            pad_idx = np.random.choice(points.shape[0], self.num_points - points.shape[0], replace=True)
            sample_idx = np.concatenate([np.arange(points.shape[0]), pad_idx], axis=0)

        points = points[sample_idx]
        depth = depth[sample_idx]
        points_norm = points / max(self.lidar_sensing_horizon, 1e-6)
        depth_norm = (depth / max(self.lidar_sensing_horizon, 1e-6))[:, None]
        return np.concatenate([points_norm, depth_norm], axis=1).astype(np.float32)

    @torch.inference_mode()
    def callback_lidar(self, data):
        if not self.odom_init:
            return

        time0 = time.time()

        points = self._pointcloud2_to_numpy(data)
        point_features = self._prepare_point_features(points)

        time1 = time.time()
        sensor_input = torch.from_numpy(point_features[None, :, :]).to(self.device, non_blocking=True)
        obs_norm, Rotation_wc = self.process_odom()
        obs_input = self.state_transform.prepare_input(obs_norm.to(self.device, non_blocking=True))
        self.Rotation_wc = Rotation_wc

        time2 = time.time()
        endstate_pred, score_pred = self.policy(sensor_input, obs_input)
        endstate_pred, score_pred = endstate_pred.cpu().numpy(), score_pred.cpu().numpy()
        time3 = time.time()

        endstate, score = self.process_output(endstate_pred, score_pred, return_all_preds=self.visualize)
        endstate_c = endstate.reshape(-1, 3, 3).transpose(0, 2, 1)
        endstate_w = np.matmul(self.Rotation_wc, endstate_c)

        action_id = np.argmin(score) if self.visualize else 0
        with self.lock:
            raw_pos = self.desire_pos if self.plan_from_reference else np.array(
                (
                    self.odom.pose.pose.position.x,
                    self.odom.pose.pose.position.y,
                    self.odom.pose.pose.position.z,
                )
            )
            start_pos = clamp_position_xyz(raw_pos, flight_height=self.flight_height)
            start_vel = self.desire_vel if self.plan_from_reference else np.array(
                (
                    self.odom.twist.twist.linear.x,
                    self.odom.twist.twist.linear.y,
                    0.0,
                )
            )
            end_x, end_y = clamp_xy(
                endstate_w[action_id, 0, 0] + start_pos[0],
                endstate_w[action_id, 1, 0] + start_pos[1],
            )
            self.optimal_poly_x = Poly5Solver(
                start_pos[0], start_vel[0], self.desire_acc[0],
                end_x, endstate_w[action_id, 0, 1], endstate_w[action_id, 0, 2], self.traj_time,
            )
            self.optimal_poly_y = Poly5Solver(
                start_pos[1], start_vel[1], self.desire_acc[1],
                end_y, endstate_w[action_id, 1, 1], endstate_w[action_id, 1, 2], self.traj_time,
            )
            self.optimal_poly_z = flat_z_poly_solver(Poly5Solver, self.flight_height, self.traj_time)
            self.ctrl_time = 0.0
        time4 = time.time()
        self.visualize_trajectory(score_pred, endstate_w)
        time5 = time.time()

        self.print_time(time0, time1, time2, time3, time4, time5)

    def control_pub(self, _timer):
        if self.ctrl_time is None or self.ctrl_time > self.traj_time:
            return
        if self.arrive and self.last_control_msg is not None:
            self.desire_init = False
            self.last_control_msg.trajectory_flag = self.last_control_msg.TRAJECTORY_STATUS_EMPTY
            self.ctrl_pub.publish(self.last_control_msg)
            return

        with self.lock:
            self.ctrl_time += self.ctrl_dt
            control_msg = self.PositionCommand()
            control_msg.header.stamp = self.ros.now()
            control_msg.trajectory_flag = control_msg.TRAJECTORY_STATUS_READY
            px = self.optimal_poly_x.get_position(self.ctrl_time)
            py = self.optimal_poly_y.get_position(self.ctrl_time)
            control_msg.position.x, control_msg.position.y = clamp_xy(px, py)
            control_msg.position.z = self.flight_height
            control_msg.velocity.x = self.optimal_poly_x.get_velocity(self.ctrl_time)
            control_msg.velocity.y = self.optimal_poly_y.get_velocity(self.ctrl_time)
            control_msg.velocity.z = 0.0
            speed_xy = np.linalg.norm([control_msg.velocity.x, control_msg.velocity.y])
            if speed_xy > self.max_speed:
                scale = self.max_speed / max(speed_xy, 1e-6)
                control_msg.velocity.x *= scale
                control_msg.velocity.y *= scale
            control_msg.acceleration.x = self.optimal_poly_x.get_acceleration(self.ctrl_time)
            control_msg.acceleration.y = self.optimal_poly_y.get_acceleration(self.ctrl_time)
            control_msg.acceleration.z = 0.0
            self.desire_pos = np.array(
                [control_msg.position.x, control_msg.position.y, self.flight_height]
            )
            self.desire_vel = np.array(
                [control_msg.velocity.x, control_msg.velocity.y, 0.0]
            )
            self.desire_acc = np.array(
                [control_msg.acceleration.x, control_msg.acceleration.y, 0.0]
            )
            goal_dir = self.goal - self.desire_pos
            yaw, yaw_dot = calculate_yaw(self.desire_vel, goal_dir, self.last_yaw, self.ctrl_dt)
            self.last_yaw = yaw
            control_msg.yaw = yaw
            control_msg.yaw_dot = yaw_dot
            self.desire_init = True
            self.last_control_msg = control_msg
            self.ctrl_pub.publish(control_msg)

    def process_output(self, endstate_pred, score_pred, return_all_preds=False):
        endstate_pred = endstate_pred.reshape(9, self.lattice_primitive.traj_num).T
        score_pred = score_pred.reshape(self.lattice_primitive.traj_num)

        if not return_all_preds:
            action_id = np.argmin(score_pred)
            lattice_id = self.lattice_primitive.traj_num - 1 - action_id
            endstate = self.state_transform.pred_to_endstate_cpu(endstate_pred[action_id, :][np.newaxis, :], lattice_id)
            score = score_pred[action_id]
        else:
            score = score_pred
            endstate = self.state_transform.pred_to_endstate_cpu(endstate_pred, torch.arange(self.lattice_primitive.traj_num-1, -1, -1))

        return endstate, score

    def visualize_trajectory(self, pred_score, pred_endstate):
        dt = self.traj_time / 20.0
        start_pos = self.desire_pos if self.plan_from_reference else np.array(
            (self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, self.odom.pose.pose.position.z))
        start_vel = self.desire_vel if self.plan_from_reference else np.array(
            (self.odom.twist.twist.linear.x, self.odom.twist.twist.linear.y, self.odom.twist.twist.linear.z))
        if self.ros.has_connections(self.best_traj_pub):
            t_values = np.arange(0, self.traj_time, dt)
            points_array = np.stack((
                self.optimal_poly_x.get_position(t_values),
                self.optimal_poly_y.get_position(t_values),
                self.optimal_poly_z.get_position(t_values)
            ), axis=-1)
            header = std_msgs.msg.Header()
            header.stamp = self.ros.now()
            header.frame_id = 'world'
            point_cloud_msg = self.point_cloud2.create_cloud_xyz32(header, points_array)
            self.best_traj_pub.publish(point_cloud_msg)
        if self.visualize and self.ros.has_connections(self.lattice_traj_pub):
            lattice_endstate = self.lattice_primitive.lattice_pos_node.cpu().numpy()
            lattice_endstate = np.dot(lattice_endstate, self.Rotation_wc.T)
            zero_state = np.zeros_like(lattice_endstate)
            lattice_poly_x = Polys5Solver(start_pos[0], start_vel[0], self.desire_acc[0],
                                          lattice_endstate[:, 0] + start_pos[0], zero_state[:, 0], zero_state[:, 0], self.traj_time)
            lattice_poly_y = Polys5Solver(start_pos[1], start_vel[1], self.desire_acc[1],
                                          lattice_endstate[:, 1] + start_pos[1], zero_state[:, 1], zero_state[:, 1], self.traj_time)
            lattice_poly_z = Polys5Solver(start_pos[2], start_vel[2], self.desire_acc[2],
                                          lattice_endstate[:, 2] + start_pos[2], zero_state[:, 2], zero_state[:, 2], self.traj_time)
            t_values = np.arange(0, self.traj_time, dt)
            points_array = np.stack((
                lattice_poly_x.get_position(t_values),
                lattice_poly_y.get_position(t_values),
                lattice_poly_z.get_position(t_values)
            ), axis=-1)
            header = std_msgs.msg.Header()
            header.stamp = self.ros.now()
            header.frame_id = 'world'
            point_cloud_msg = self.point_cloud2.create_cloud_xyz32(header, points_array)
            self.lattice_traj_pub.publish(point_cloud_msg)
        if self.visualize and self.ros.has_connections(self.all_trajs_pub):
            all_poly_x = Polys5Solver(start_pos[0], start_vel[0], self.desire_acc[0],
                                      pred_endstate[:, 0, 0] + start_pos[0], pred_endstate[:, 0, 1], pred_endstate[:, 0, 2], self.traj_time)
            all_poly_y = Polys5Solver(start_pos[1], start_vel[1], self.desire_acc[1],
                                      pred_endstate[:, 1, 0] + start_pos[1], pred_endstate[:, 1, 1], pred_endstate[:, 1, 2], self.traj_time)
            all_poly_z = Polys5Solver(start_pos[2], start_vel[2], self.desire_acc[2],
                                      pred_endstate[:, 2, 0] + start_pos[2], pred_endstate[:, 2, 1], pred_endstate[:, 2, 2], self.traj_time)
            t_values = np.arange(0, self.traj_time, dt)
            points_array = np.stack((
                all_poly_x.get_position(t_values),
                all_poly_y.get_position(t_values),
                all_poly_z.get_position(t_values)
            ), axis=-1)
            scores = np.repeat(pred_score, t_values.size)
            points_array = np.column_stack((points_array, scores))
            header = std_msgs.msg.Header()
            header.stamp = self.ros.now()
            header.frame_id = 'world'
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            ]
            point_cloud_msg = self.point_cloud2.create_cloud(header, fields, points_array)
            self.all_trajs_pub.publish(point_cloud_msg)

    def print_time(self, time0, time1, time2, time3, time4, time5):
        self.time_interpolation = self.time_interpolation + (time1 - time0)
        self.time_prepare = self.time_prepare + (time2 - time1)
        self.time_forward = self.time_forward + (time3 - time2)
        self.time_process = self.time_process + (time4 - time3)
        self.time_visualize = self.time_visualize + (time5 - time4)
        self.count = self.count + 1

        total_time = (time5 - time0) * 1000
        tolerance = 1000.0 / self.sensor_fps
        if total_time > tolerance:
            self.ros.logwarn(f"Warn: Processing time {(time5 - time0) * 1000:.2f} ms exceeds {tolerance:.2f} ms, may cause message lag!")
            print(f"\033[34mCurrent Time Consuming:\033[0m "
                  f"lidar-preprocess: \033[32m{1000 * (time1 - time0):.2f} ms\033[0m; "
                  f"data-prepare: \033[32m{1000 * (time2 - time1):.2f} ms\033[0m; "
                  f"network-inference: \033[32m{1000 * (time3 - time2):.2f} ms\033[0m; "
                  f"post-process: \033[32m{1000 * (time4 - time3):.2f} ms\033[0m; "
                  f"visualize-trajectory: \033[32m{1000 * (time5 - time4):.2f} ms\033[0m")
        if self.verbose or (total_time > tolerance):
            print(f"\033[34mAverage Time Consuming:\033[0m "
                  f"lidar-preprocess: \033[32m{1000 * self.time_interpolation / self.count:.2f} ms\033[0m; "
                  f"data-prepare: \033[32m{1000 * self.time_prepare / self.count:.2f} ms\033[0m; "
                  f"network-inference: \033[32m{1000 * self.time_forward / self.count:.2f} ms\033[0m; "
                  f"post-process: \033[32m{1000 * self.time_process / self.count:.2f} ms\033[0m; "
                  f"visualize-trajectory: \033[32m{1000 * self.time_visualize / self.count:.2f} ms\033[0m")

    def warm_up(self):
        dummy_input = torch.randn(1, self.num_points, self.point_feature_dim).to(self.device)
        dummy_obs = torch.randn(1, 9, self.lattice_primitive.vertical_num, self.lattice_primitive.horizon_num).to(self.device)
        for _ in range(10):
            self.policy(dummy_input, dummy_obs)
        print("Warm-up done!")


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--weight',
        type=str,
        default=None,
        help='model weight path; if omitted, auto-detects policy/YOPO/saved/**/*.pth',
    )
    parser.add_argument('--use_tensorrt', type=str2bool, nargs='?', const=True, default=False)
    parser.add_argument('--verbose', type=str2bool, nargs='?', const=True, default=False)
    parser.add_argument('--visualize', type=str2bool, nargs='?', const=True, default=True)
    parser.add_argument('--plan_from_reference', type=str2bool, nargs='?', const=True, default=True)
    parser.add_argument('--odom_topic', type=str, default=None)
    parser.add_argument('--lidar_topic', type=str, default=None)
    parser.add_argument('--ctrl_topic', type=str, default=None)
    parser.add_argument('--sensor_fps', type=float, default=10.0)
    parser.add_argument('--ros_version', type=str, default='ros2', help='must be ros2 (default)')
    parser.add_argument(
        '--sensor_mode',
        type=str,
        default='lidar',
        choices=['lidar', 'auto'],
        help='Deprecated (LiDAR-only build): use lidar or auto; depth mode was removed.',
    )
    parser.add_argument('--goal', type=float, nargs=2, default=[10.0, 0.0], help='2D goal: x y')
    parser.add_argument(
        '--flight_height',
        type=float,
        default=DEFAULT_FLIGHT_HEIGHT,
        help='fixed flight height, clamped to forest map range [0.5, 4.0]m',
    )
    args = parser.parse_args()

    settings = vars(args)
    weight = resolve_weight_path(settings.pop('weight'))
    print(f"Using weight: {weight}")
    settings.pop('sensor_mode', None)
    YopoNet(settings, weight)
