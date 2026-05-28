#!/usr/bin/env python3
"""SUPER-compatible ROS2 planner node for YOPO_lidar_V3."""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Optional, Tuple

_SUPER_DIR = Path(__file__).resolve().parent
_POLICY_ROOT = _SUPER_DIR.parent
for _p in (_SUPER_DIR, _POLICY_ROOT):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from utils.flight_constraints import (
    DEFAULT_FLIGHT_HEIGHT,
    clamp_xy,
    cruise_height,
    make_3d_goal,
)

from super_fsm import MachineState, SuperFSM
from super_map import MapConfig
from super_mission import MissionConfig, WaypointMission
from path_optimizer import lookahead_target
from super_planner_core import PlanRetCode, PlannerCoreConfig, SuperPlannerCore
from committed_trajectory import CommittedTrajectory

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from quadrotor_msgs.msg import PositionCommand
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32MultiArray, String

# Softer than network_control_node to avoid double-aggressive tracking.
KX_XY = 3.2
KV_XY = 2.2
KX_Z = 4.5
KV_Z = 2.8
MAX_ACC_XY = 4.5
MAX_ACC_Z = 2.5
MAX_JERK_XY = 12.0
MAX_YAW_RATE = 1.0
APPROACH_DECEL = 2.0
HOVER_LOCK_SPEED = 0.2
HOVER_SETTLE_FRAMES = 15
EXPECTED_GOAL_FRAME = "world"
HOVER_KP_XY = 1.8
HOVER_KD_XY = 1.4
HOVER_KP_Z = 2.2
HOVER_KD_Z = 1.6
HOVER_MAX_ACC_XY = 1.6
HOVER_MAX_ACC_Z = 1.2
GOAL_APPROACH_RADIUS = 2.0
GOAL_ARRIVE_POS_TOL = 0.25
GOAL_ARRIVE_VEL_TOL = 0.25
GOAL_ARRIVE_HOLD_FRAMES = 10
PATH_PROGRESS_RESET_DIST = 4.0
EMER_BACKOFF_SEC = 0.8
STUCK_PROGRESS_TIMEOUT_SEC = 6.0
STUCK_PROGRESS_EPS_M = 0.6
STUCK_RECOVERY_COOLDOWN_SEC = 3.0
SAFE_TARGET_HOLD_FRAMES = 12


@dataclass
class PlannerConfig:
    flight_height: float
    max_speed: float
    lookahead_distance: float
    planning_rate_hz: float
    control_rate_hz: float
    goal_tolerance: float
    world_frame_cloud: bool
    use_default_goal: bool
    target_smooth_tau: float
    acc_smooth_tau: float
    yaw_smooth_tau: float


class SuperCompatPlanner(Node):
    """
    ROS2 adapter for SUPER-style planning in YOPO_lidar_V3.

    Architecture (mirrors SUPER/super_planner + SUPER/mission_planner):
    - mission_planner -> optional waypoint file / RViz goals
    - FSM -> INIT / WAIT_GOAL / GENERATE_TRAJ / FOLLOW_TRAJ / EMER_STOP
    - SuperPlannerCore -> map + A* + replan (PlanFromRest / ReplanOnce)
    - Command layer -> PositionCommand for existing SO3 controller
    """

    def __init__(self) -> None:
        super().__init__("super_compat_planner")
        self._declare_parameters()
        self.cfg = self._load_planner_config()
        self.core = self._build_core()
        self.fsm = SuperFSM()
        self.fsm.on_system_start()
        self.fsm.change(MachineState.WAIT_GOAL)

        self.mission: Optional[WaypointMission] = self._load_mission()
        self.goal = self._make_3d_goal(self.get_parameter("default_goal").value)
        self.goal_active = self.cfg.use_default_goal
        self._new_goal_flag = self.cfg.use_default_goal
        if self.goal_active:
            self.core.set_goal(self.goal[0], self.goal[1])
            self.fsm.on_new_goal()

        self.odom: Optional[Odometry] = None
        self.last_cmd_id = 0
        self.last_warn_time = 0.0
        self.last_wait_goal_info_time = 0.0
        self.last_diag_pub_time = 0.0
        self._dt_ctrl = 1.0 / max(self.cfg.control_rate_hz, 1e-3)
        self._smooth_target: Optional[List[float]] = None
        self._smooth_acc = [0.0, 0.0, 0.0]
        self._smooth_yaw: Optional[float] = None
        self._hover_anchor: Optional[Tuple[float, float, float]] = None
        self._hover_settled_count = 0
        self._path_progress_idx = 0
        self._last_safe_target: Optional[Tuple[float, float]] = None
        self._unsafe_hold_count = 0
        self._goal_arrive_count = 0
        self._goal_reached_latched = False
        self._emer_count = 0
        self._last_plan_ret = PlanRetCode.NO_NEED
        self._last_plan_mode = "idle"
        self._emer_backoff_until = 0.0
        self._best_goal_dist = float("inf")
        self._last_progress_time = 0.0
        self._stuck_recover_until = 0.0
        self._active_safe_target: Optional[Tuple[float, float]] = None
        self._safe_target_hold_count = 0

        odom_topic = str(self.get_parameter("odom_topic").value)
        lidar_topic = str(self.get_parameter("lidar_topic").value)
        ctrl_topic = str(self.get_parameter("ctrl_topic").value)
        goal_topic = str(self.get_parameter("goal_topic").value)
        self.expected_goal_frame = str(self.get_parameter("expected_goal_frame").value)
        path_topic = str(self.get_parameter("path_topic").value)

        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self.create_subscription(PointCloud2, lidar_topic, self._on_lidar, 10)
        self.create_subscription(PoseStamped, goal_topic, self._on_goal, 10)
        self.cmd_pub = self.create_publisher(PositionCommand, ctrl_topic, 10)
        self.path_pub = self.create_publisher(Path, path_topic, 10)
        self.diag_state_pub = self.create_publisher(String, "/super_compat/diag/fsm_state", 10)
        self.diag_plan_pub = self.create_publisher(String, "/super_compat/diag/plan_ret", 10)
        self.diag_metrics_pub = self.create_publisher(
            Float32MultiArray, "/super_compat/diag/metrics", 10
        )

        self.create_timer(1.0 / max(self.cfg.planning_rate_hz, 1e-3), self._plan_once)
        self.create_timer(1.0 / max(self.cfg.control_rate_hz, 1e-3), self._publish_cmd)
        self.create_timer(0.25, self._publish_diag)
        if self.mission is not None:
            self.create_timer(0.1, self._mission_tick)

        self._check_lidar_config(lidar_topic)
        self.get_logger().info(
            f"SUPER-compat ready | state={self.fsm.label()} | odom={odom_topic} | "
            f"lidar={lidar_topic} | ctrl={ctrl_topic} | goal={goal_topic} | "
            f"world_frame_cloud={self.cfg.world_frame_cloud} | h={self.cfg.flight_height:.2f}m"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("odom_topic", "/sim/odom")
        self.declare_parameter("lidar_topic", "/lidar_points")
        self.declare_parameter("ctrl_topic", "/so3_control/pos_cmd")
        self.declare_parameter("goal_topic", "/move_base_simple/goal")
        self.declare_parameter("path_topic", "/super_compat/planned_path")
        self.declare_parameter("expected_goal_frame", EXPECTED_GOAL_FRAME)
        self.declare_parameter("mission_file", "")
        self.declare_parameter("map_resolution", 0.35)
        self.declare_parameter("planning_horizon", 16.0)
        self.declare_parameter("obstacle_inflation", 0.7)
        self.declare_parameter("map_decay_sec", 2.0)
        self.declare_parameter("flight_height", DEFAULT_FLIGHT_HEIGHT)
        self.declare_parameter("max_speed", 10.0)
        self.declare_parameter("lookahead_distance", 2.2)
        self.declare_parameter("planning_rate_hz", 8.0)
        self.declare_parameter("control_rate_hz", 50.0)
        self.declare_parameter("goal_tolerance", 0.8)
        self.declare_parameter("world_frame_cloud", False)
        self.declare_parameter("start_clearance", 1.0)
        self.declare_parameter("use_default_goal", False)
        self.declare_parameter("default_goal", [10.0, 0.0])
        self.declare_parameter("target_smooth_tau", 0.45)
        self.declare_parameter("acc_smooth_tau", 0.20)
        self.declare_parameter("yaw_smooth_tau", 0.35)
        self.declare_parameter("mission_switch_dis", 3.0)

    def _load_planner_config(self) -> PlannerConfig:
        return PlannerConfig(
            flight_height=cruise_height(float(self.get_parameter("flight_height").value)),
            max_speed=min(float(self.get_parameter("max_speed").value), 10.0),
            lookahead_distance=float(self.get_parameter("lookahead_distance").value),
            planning_rate_hz=float(self.get_parameter("planning_rate_hz").value),
            control_rate_hz=float(self.get_parameter("control_rate_hz").value),
            goal_tolerance=float(self.get_parameter("goal_tolerance").value),
            world_frame_cloud=bool(self.get_parameter("world_frame_cloud").value),
            use_default_goal=bool(self.get_parameter("use_default_goal").value),
            target_smooth_tau=float(self.get_parameter("target_smooth_tau").value),
            acc_smooth_tau=float(self.get_parameter("acc_smooth_tau").value),
            yaw_smooth_tau=float(self.get_parameter("yaw_smooth_tau").value),
        )

    def _build_core(self) -> SuperPlannerCore:
        map_cfg = MapConfig(
            resolution=float(self.get_parameter("map_resolution").value),
            planning_horizon=float(self.get_parameter("planning_horizon").value),
            obstacle_inflation=float(self.get_parameter("obstacle_inflation").value),
            map_decay_sec=float(self.get_parameter("map_decay_sec").value),
            start_clearance=float(self.get_parameter("start_clearance").value),
            flight_height=self.cfg.flight_height,
        )
        core_cfg = PlannerCoreConfig(map=map_cfg, goal_tolerance=self.cfg.goal_tolerance)
        return SuperPlannerCore(core_cfg)

    def _load_mission(self) -> Optional[WaypointMission]:
        mission_file = str(self.get_parameter("mission_file").value).strip()
        if not mission_file or mission_file.lower() in {"__none__", "none", "null"}:
            return None
        path = Path(mission_file)
        if not path.is_absolute():
            path = _SUPER_DIR / "data" / mission_file
        if not path.exists():
            self.get_logger().error(f"mission_file not found: {path}")
            return None
        mcfg = MissionConfig(switch_dis_default=float(self.get_parameter("mission_switch_dis").value))
        mission = WaypointMission.from_file(path, mcfg)
        mission.trigger()
        self.get_logger().info(f"Loaded mission with {len(mission.waypoints)} waypoints from {path}")
        return mission

    @property
    def current_path(self) -> List[Tuple[float, float]]:
        return self.core.state.current_path

    @property
    def path_ready(self) -> bool:
        return self.core.state.path_ready

    @property
    def committed_traj(self) -> Optional[CommittedTrajectory]:
        return self.core.state.committed_traj

    def _check_lidar_config(self, lidar_topic: str) -> None:
        looks_world = lidar_topic.endswith("_world") or "world" in lidar_topic
        if looks_world and not self.cfg.world_frame_cloud:
            self.get_logger().warn(
                f"lidar_topic='{lidar_topic}' vs world_frame_cloud=false may corrupt the map."
            )
        if (not looks_world) and self.cfg.world_frame_cloud:
            self.get_logger().warn(
                f"lidar_topic='{lidar_topic}' vs world_frame_cloud=true may corrupt the map."
            )

    def _make_3d_goal(self, goal_xy) -> List[float]:
        if len(goal_xy) < 2:
            raise ValueError("goal must contain x and y")
        g = make_3d_goal(goal_xy[:2], self.cfg.flight_height)
        return [float(g[0]), float(g[1]), float(g[2])]

    def _apply_goal_xy(self, gx: float, gy: float, source: str) -> None:
        gx, gy = clamp_xy(gx, gy)
        if not self.core.set_goal(gx, gy):
            self.get_logger().warn(f"Goal ({gx:.2f}, {gy:.2f}) deeply occupied; skipped.")
            return
        snapped = self.core.state.goal_xy
        self.goal = self._make_3d_goal(snapped)
        self.goal_active = True
        self._new_goal_flag = True
        self.fsm.on_new_goal()
        self.core.reset_path_on_new_goal()
        self._hover_anchor = None
        self._hover_settled_count = 0
        self._path_progress_idx = 0
        self._last_safe_target = None
        self._unsafe_hold_count = 0
        self._goal_arrive_count = 0
        self._goal_reached_latched = False
        self._best_goal_dist = float("inf")
        self._last_progress_time = self.get_clock().now().nanoseconds * 1e-9
        self._stuck_recover_until = 0.0
        self._active_safe_target = None
        self._safe_target_hold_count = 0
        self._reset_tracking_state()
        if self.odom is not None:
            sx, sy, _ = self._odom_pos()
            dist = math.hypot(self.goal[0] - sx, self.goal[1] - sy)
            self.get_logger().info(
                f"[{source}] goal=({self.goal[0]:.2f}, {self.goal[1]:.2f}, {self.goal[2]:.2f}), dist={dist:.1f}m"
            )
        else:
            self.get_logger().info(
                f"[{source}] goal=({self.goal[0]:.2f}, {self.goal[1]:.2f}, {self.goal[2]:.2f})"
            )

    def _on_odom(self, msg: Odometry) -> None:
        self.odom = msg
        if self.mission is not None:
            p = msg.pose.pose.position
            self.mission.update_odom(p.x, p.y, p.z, self.get_clock().now().nanoseconds * 1e-9)

    def _on_goal(self, msg: PoseStamped) -> None:
        frame = (msg.header.frame_id or "").strip().lstrip("/")
        expected = self.expected_goal_frame.strip().lstrip("/")
        if frame and expected and frame != expected:
            self.get_logger().warn(
                f"Goal frame_id='{msg.header.frame_id}' != '{expected}'; using position as world xy."
            )
        self._apply_goal_xy(msg.pose.position.x, msg.pose.position.y, "rviz")

    def _mission_tick(self) -> None:
        if self.mission is None or self.odom is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        nxt = self.mission.tick(now)
        if nxt is not None:
            self._apply_goal_xy(nxt[0], nxt[1], "mission")

    def _on_lidar(self, msg: PointCloud2) -> None:
        if self.odom is None:
            return
        now_t = self.get_clock().now().nanoseconds * 1e-9
        world_pts = []
        for px, py, pz in self._iter_xyz(msg):
            if self.cfg.world_frame_cloud:
                world_pts.append((px, py, pz))
            else:
                world_pts.append(self._body_to_world(px, py, pz))
        self.core.map.ingest_world_points(world_pts, now_t)

    def _plan_once(self) -> None:
        if self.odom is None:
            return

        sx, sy, _ = self._odom_pos()
        vx, vy, _ = self._odom_vel()
        now_t = self.get_clock().now().nanoseconds * 1e-9

        if now_t < self._emer_backoff_until:
            return

        if not self.goal_active:
            self.core.reset_path_on_new_goal()
            if now_t - self.last_wait_goal_info_time > 5.0:
                self.get_logger().info(
                    f"Waiting for goal on {self.get_parameter('goal_topic').value}"
                )
                self.last_wait_goal_info_time = now_t
            self.fsm.change(MachineState.WAIT_GOAL)
            return

        gx, gy = clamp_xy(self.goal[0], self.goal[1])
        self.goal[0], self.goal[1] = gx, gy
        self.core.state.goal_xy = (gx, gy)

        if self.fsm.state == MachineState.WAIT_GOAL:
            self.fsm.step_wait_goal()

        if self.fsm.state == MachineState.GENERATE_TRAJ or self._new_goal_flag:
            ret = self.core.plan_from_rest(sx, sy, now_sec=now_t, vx=vx, vy=vy)
            self._last_plan_ret = ret
            self._last_plan_mode = "plan_from_rest"
            self._handle_plan_ret(ret, sx, sy, plan_from_rest=True)
            self._new_goal_flag = False
            return

        if self.fsm.should_replan():
            ret = self.core.replan_once(sx, sy, new_goal=False, now_sec=now_t, vx=vx, vy=vy)
            self._last_plan_ret = ret
            self._last_plan_mode = "replan_once"
            self._handle_plan_ret(ret, sx, sy, plan_from_rest=False)

        self._publish_path_viz()

    def _handle_plan_ret(
        self, ret: PlanRetCode, sx: float, sy: float, plan_from_rest: bool
    ) -> None:
        if ret == PlanRetCode.FINISH:
            self.fsm.step_generate_traj(True, close_to_goal=True)
            return
        if ret == PlanRetCode.SUCCESS:
            if plan_from_rest:
                self.fsm.step_generate_traj(True, close_to_goal=False)
            else:
                self.fsm.change(MachineState.FOLLOW_TRAJ)
            return
        if ret == PlanRetCode.NO_NEED:
            return
        if ret == PlanRetCode.EMER:
            self.get_logger().warn("Planner EMER: path blocked, attempting auto-recovery replan.")
            self._emer_count += 1
            self._emer_backoff_until = self.get_clock().now().nanoseconds * 1e-9 + EMER_BACKOFF_SEC
            self.fsm.step_follow_traj(replan_emer=True)
            self.fsm.step_emer_stop()
            self.core.reset_path_on_new_goal()
            self._new_goal_flag = True
            self.fsm.on_new_goal()
            return
        if ret == PlanRetCode.FAILED:
            now_t = self.get_clock().now().nanoseconds * 1e-9
            if now_t - self.last_warn_time > 2.0:
                self.get_logger().warn("Plan failed; waiting next replan tick.")
                self.last_warn_time = now_t

    def _publish_diag(self) -> None:
        now_t = self.get_clock().now().nanoseconds * 1e-9
        if now_t - self.last_diag_pub_time < 0.2:
            return
        self.last_diag_pub_time = now_t

        state_msg = String()
        state_msg.data = self.fsm.label()
        self.diag_state_pub.publish(state_msg)

        plan_msg = String()
        plan_msg.data = f"{self._last_plan_mode}:{self._last_plan_ret.name}"
        self.diag_plan_pub.publish(plan_msg)

        metrics = Float32MultiArray()
        if self.odom is None:
            metrics.data = [0.0] * 10
            self.diag_metrics_pub.publish(metrics)
            return

        x, y, z = self._odom_pos()
        vx, vy, vz = self._odom_vel()
        gx, gy = self.goal[0], self.goal[1]
        d_goal = math.hypot(gx - x, gy - y)
        d_path = self._distance_to_current_path((x, y))
        clearance_proxy = self._clearance_proxy((x, y))
        metrics.data = [
            float(d_goal),
            float(d_path),
            float(math.hypot(vx, vy)),
            float(self._unsafe_hold_count),
            float(self._emer_count),
            float(1.0 if self.core.state.connected_to_goal else 0.0),
            float(1.0 if self.path_ready else 0.0),
            float(1.0 if self._goal_reached_latched else 0.0),
            float(z),
            float(clearance_proxy),
        ]
        self.diag_metrics_pub.publish(metrics)

    def _publish_path_viz(self) -> None:
        if not self.current_path:
            return
        path = Path()
        path.header.frame_id = EXPECTED_GOAL_FRAME
        path.header.stamp = self.get_clock().now().to_msg()
        for x, y in self.current_path:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = self.cfg.flight_height
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.path_pub.publish(path)

    def _publish_cmd(self) -> None:
        if self.odom is None:
            return

        x, y, z = self._odom_pos()
        yaw = self._odom_yaw()
        vx, vy, vz = self._odom_vel()

        cmd = PositionCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        # Use controller-side position/velocity closed-loop for robustness.
        cmd.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_EMPTY
        cmd.trajectory_id = self.last_cmd_id
        self.last_cmd_id += 1

        if not self.goal_active or self.fsm.state == MachineState.EMER_STOP:
            # Idle/EMER mode should hold current altitude instead of forcing cruise height.
            self._try_lock_hover_anchor(x, y, vx, vy, prefer=(x, y, z))
            self._fill_hover_cmd(cmd, x, y, z, yaw, vx, vy, vz)
            self.cmd_pub.publish(cmd)
            return

        if self._goal_reached_latched:
            self._try_lock_hover_anchor(self.goal[0], self.goal[1], vx, vy, prefer=self.goal)
            self._fill_hover_cmd(cmd, x, y, z, yaw, vx, vy, vz)
            self.cmd_pub.publish(cmd)
            return

        if self._goal_reached_and_stable(x, y, vx, vy):
            self._goal_reached_latched = True
            self._try_lock_hover_anchor(self.goal[0], self.goal[1], vx, vy, prefer=self.goal)
            self._fill_hover_cmd(cmd, x, y, z, yaw, vx, vy, vz)
            self.cmd_pub.publish(cmd)
            return

        traj = self.committed_traj
        if traj is not None and not traj.empty():
            now_t = self.get_clock().now().nanoseconds * 1e-9
            t = max(0.0, now_t - self.core.state.traj_stamp)
            st = traj.sample(t)
            yaw_ref = self._sample_yaw_ref(t, yaw)
            self._fill_traj_sample_cmd(
                cmd,
                x=x,
                y=y,
                vx=vx,
                vy=vy,
                z=z,
                vz=vz,
                yaw=yaw,
                target_x=st.x,
                target_y=st.y,
                target_vx=st.vx,
                target_vy=st.vy,
                target_ax=st.ax,
                target_ay=st.ay,
                yaw_ref=yaw_ref,
            )
            self.cmd_pub.publish(cmd)
            return

        gx, gy = self.goal[0], self.goal[1]
        dis_to_goal = math.hypot(gx - x, gy - y)
        self._maybe_trigger_stuck_recovery(dis_to_goal)

        # Final approach: force exact goal tracking (instead of lookahead waypoints).
        if dis_to_goal < GOAL_APPROACH_RADIUS:
            target_x, target_y = gx, gy
            target_x, target_y = clamp_xy(target_x, target_y)
            target_x, target_y = self._smooth_target_xy(target_x, target_y)
            self._fill_tracking_cmd(cmd, x, y, z, vx, vy, vz, yaw, target_x, target_y)
            self.cmd_pub.publish(cmd)
            return

        if self.path_ready and self.current_path:
            target_x, target_y = self._path_target_with_progress((x, y))
            safe_target = self._select_safe_target((x, y), (target_x, target_y), self._path_progress_idx)
            if safe_target is None:
                safe_target = self._local_escape_target((x, y), (self.goal[0], self.goal[1]))
            if safe_target is None:
                self._unsafe_hold_count += 1
                target_x, target_y = self._continue_toward_goal((x, y))
            else:
                self._unsafe_hold_count = 0
                target_x, target_y = safe_target
        else:
            target_x, target_y = self.goal[0], self.goal[1]
            safe_target = self._select_safe_target((x, y), (target_x, target_y), 0)
            if safe_target is None:
                safe_target = self._local_escape_target((x, y), (self.goal[0], self.goal[1]))
            if safe_target is None:
                self._unsafe_hold_count += 1
                target_x, target_y = self._continue_toward_goal((x, y))
            else:
                self._unsafe_hold_count = 0
                target_x, target_y = safe_target

        target_x, target_y = clamp_xy(target_x, target_y)
        target_x, target_y = self._smooth_target_xy(target_x, target_y)
        self._fill_tracking_cmd(cmd, x, y, z, vx, vy, vz, yaw, target_x, target_y)
        self.cmd_pub.publish(cmd)

    def _alpha_from_tau(self, tau: float) -> float:
        tau = max(tau, 1e-3)
        return 1.0 - math.exp(-self._dt_ctrl / tau)

    def _reset_tracking_state(self) -> None:
        self._smooth_target = None
        self._smooth_acc = [0.0, 0.0, 0.0]

    def _try_lock_hover_anchor(
        self,
        x: float,
        y: float,
        vx: float,
        vy: float,
        prefer: Optional[Tuple[float, float, float]] = None,
    ) -> None:
        if self._hover_anchor is not None:
            return
        if math.hypot(vx, vy) > HOVER_LOCK_SPEED:
            self._hover_settled_count = 0
            return
        self._hover_settled_count += 1
        if self._hover_settled_count < HOVER_SETTLE_FRAMES:
            return
        if prefer is not None:
            hx, hy = clamp_xy(prefer[0], prefer[1])
            hz = prefer[2]
        else:
            hx, hy = clamp_xy(x, y)
            hz = self.cfg.flight_height
        self._hover_anchor = (hx, hy, hz)
        self._smooth_target = [hx, hy]
        self._smooth_acc = [0.0, 0.0, 0.0]
        self.get_logger().info(
            f"Hover anchor locked at ({hx:.2f}, {hy:.2f}, {hz:.2f})"
        )

    def _select_safe_target(
        self,
        start_xy: Tuple[float, float],
        target_xy: Tuple[float, float],
        min_idx: int,
    ) -> Optional[Tuple[float, float]]:
        blocked = self.core._blocked_set(start_xy)
        if (
            self._active_safe_target is not None
            and self._safe_target_hold_count < SAFE_TARGET_HOLD_FRAMES
            and self.core.map.segment_is_free(start_xy, self._active_safe_target, blocked)
        ):
            self._safe_target_hold_count += 1
            return self._active_safe_target
        if self.core.map.segment_is_free(start_xy, target_xy, blocked):
            self._last_safe_target = target_xy
            self._active_safe_target = target_xy
            self._safe_target_hold_count = 0
            return target_xy

        if self.current_path:
            sx, sy = start_xy
            gx, gy = self.goal[0], self.goal[1]
            dgx, dgy = gx - sx, gy - sy
            dgn = math.hypot(dgx, dgy)
            if dgn < 1e-6:
                dgn = 1.0
            ugx, ugy = dgx / dgn, dgy / dgn

            best: Optional[Tuple[float, float]] = None
            best_score = 1e18
            fallback: Optional[Tuple[float, float]] = None
            fallback_score = 1e18

            for i in range(max(min_idx, 0), len(self.current_path)):
                p = self.current_path[i]
                if not self.core.map.segment_is_free(start_xy, p, blocked):
                    continue
                px, py = p
                # prefer candidates that reduce distance to final goal
                d_goal = math.hypot(gx - px, gy - py)
                dsx, dsy = px - sx, py - sy
                dsn = math.hypot(dsx, dsy)
                if dsn < 1e-6:
                    continue
                ux, uy = dsx / dsn, dsy / dsn
                align = ux * ugx + uy * ugy  # +1 same direction, -1 opposite

                # primary: forward-ish + closer to goal
                if align > -0.15:
                    score = d_goal - 0.3 * dsn
                    if score < best_score:
                        best_score = score
                        best = p

                # fallback: any safe point, but penalize opposite direction
                fb_score = d_goal + (1.0 - align) * 2.0
                if fb_score < fallback_score:
                    fallback_score = fb_score
                    fallback = p

            if best is not None:
                self._last_safe_target = best
                self._active_safe_target = best
                self._safe_target_hold_count = 0
                return best
            if fallback is not None:
                self._last_safe_target = fallback
                self._active_safe_target = fallback
                self._safe_target_hold_count = 0
                return fallback
        if self._last_safe_target is not None and self.core.map.segment_is_free(
            start_xy, self._last_safe_target, blocked
        ):
            self._active_safe_target = self._last_safe_target
            self._safe_target_hold_count = 0
            return self._last_safe_target
        return None

    def _local_escape_target(
        self,
        start_xy: Tuple[float, float],
        goal_xy: Tuple[float, float],
    ) -> Optional[Tuple[float, float]]:
        blocked = self.core._blocked_set(start_xy)
        sx, sy = start_xy
        gx, gy = goal_xy
        dx = gx - sx
        dy = gy - sy
        norm = math.hypot(dx, dy)
        if norm < 1e-6:
            return None
        ux, uy = dx / norm, dy / norm
        step = max(self.core.cfg.map.resolution * 2.0, 0.6)
        # first try along goal direction
        for k in (1.0, 0.75, 0.5):
            tx = sx + ux * step * k
            ty = sy + uy * step * k
            if self.core.map.segment_is_free(start_xy, (tx, ty), blocked):
                self._last_safe_target = (tx, ty)
                return tx, ty
        # then try small angular fan around heading
        base = math.atan2(uy, ux)
        for da in (0.35, -0.35, 0.7, -0.7, 1.05, -1.05):
            th = base + da
            tx = sx + math.cos(th) * step
            ty = sy + math.sin(th) * step
            if self.core.map.segment_is_free(start_xy, (tx, ty), blocked):
                self._last_safe_target = (tx, ty)
                return tx, ty
        return None

    def _path_target_with_progress(self, pos_xy: Tuple[float, float]) -> Tuple[float, float]:
        if not self.current_path:
            return self.goal[0], self.goal[1]
        sx, sy = pos_xy
        start_i = max(0, min(self._path_progress_idx, len(self.current_path) - 1))

        nearest_i_fwd = start_i
        best_d_fwd = 1e18
        for i in range(start_i, len(self.current_path)):
            px, py = self.current_path[i]
            d = (px - sx) * (px - sx) + (py - sy) * (py - sy)
            if d < best_d_fwd:
                best_d_fwd = d
                nearest_i_fwd = i

        # If far away from the forward corridor, reset progress to global nearest.
        if math.sqrt(best_d_fwd) > PATH_PROGRESS_RESET_DIST:
            nearest_i_all = 0
            best_d_all = 1e18
            for i, (px, py) in enumerate(self.current_path):
                d = (px - sx) * (px - sx) + (py - sy) * (py - sy)
                if d < best_d_all:
                    best_d_all = d
                    nearest_i_all = i
            self._path_progress_idx = nearest_i_all
        else:
            self._path_progress_idx = max(self._path_progress_idx, nearest_i_fwd)

        sub_path = self.current_path[self._path_progress_idx :]
        if len(sub_path) <= 1:
            return self.current_path[-1]
        return lookahead_target(pos_xy, sub_path, self.cfg.lookahead_distance)

    def _continue_toward_goal(self, pos_xy: Tuple[float, float]) -> Tuple[float, float]:
        # Keep the vehicle in tracking mode until it truly reaches goal.
        if self._last_safe_target is not None:
            x, y = pos_xy
            gx, gy = self.goal[0], self.goal[1]
            d_cur = math.hypot(gx - x, gy - y)
            d_last = math.hypot(gx - self._last_safe_target[0], gy - self._last_safe_target[1])
            # If last target clearly drives away from goal, discard it.
            if d_last <= d_cur + 0.8:
                return self._last_safe_target
        x, y = pos_xy
        gx, gy = self.goal[0], self.goal[1]
        dx, dy = gx - x, gy - y
        dist = math.hypot(dx, dy)
        if dist < 1e-3:
            return gx, gy
        step = min(max(self.core.cfg.map.resolution, 0.4), dist)
        return x + dx / dist * step, y + dy / dist * step

    def _maybe_trigger_stuck_recovery(self, dis_to_goal: float) -> None:
        now_t = self.get_clock().now().nanoseconds * 1e-9
        if self._last_progress_time <= 0.0:
            self._last_progress_time = now_t
            self._best_goal_dist = dis_to_goal
            return
        if dis_to_goal + STUCK_PROGRESS_EPS_M < self._best_goal_dist:
            self._best_goal_dist = dis_to_goal
            self._last_progress_time = now_t
            return
        if now_t < self._stuck_recover_until:
            return
        if now_t - self._last_progress_time < STUCK_PROGRESS_TIMEOUT_SEC:
            return
        if dis_to_goal < max(self.cfg.goal_tolerance * 2.0, 2.0):
            return
        self.get_logger().warn(
            "Detected stuck around obstacle: force reset and plan-from-rest recovery."
        )
        self.core.reset_path_on_new_goal()
        self._new_goal_flag = True
        self.fsm.on_new_goal()
        self._path_progress_idx = 0
        self._last_safe_target = None
        self._active_safe_target = None
        self._safe_target_hold_count = 0
        self._stuck_recover_until = now_t + STUCK_RECOVERY_COOLDOWN_SEC
        self._last_progress_time = now_t
        self._best_goal_dist = dis_to_goal

    def _distance_to_current_path(self, pos_xy: Tuple[float, float]) -> float:
        if not self.current_path:
            return 1e9
        px, py = pos_xy
        best = 1e18
        for x, y in self.current_path:
            d = (x - px) * (x - px) + (y - py) * (y - py)
            if d < best:
                best = d
        return math.sqrt(best)

    def _clearance_proxy(self, pos_xy: Tuple[float, float]) -> float:
        blocked = self.core._blocked_set(pos_xy)
        if not blocked:
            return self.core.cfg.map.planning_horizon
        px, py = pos_xy
        best = 1e18
        for gx, gy in blocked:
            wx, wy = self.core.map.grid_to_world(gx, gy)
            d = math.hypot(wx - px, wy - py)
            if d < best:
                best = d
        return best if best < 1e17 else self.core.cfg.map.planning_horizon

    def _smooth_target_xy(self, raw_x: float, raw_y: float) -> Tuple[float, float]:
        if self._smooth_target is None:
            self._smooth_target = [raw_x, raw_y]
            return raw_x, raw_y
        a = self._alpha_from_tau(self.cfg.target_smooth_tau)
        self._smooth_target[0] += a * (raw_x - self._smooth_target[0])
        self._smooth_target[1] += a * (raw_y - self._smooth_target[1])
        return self._smooth_target[0], self._smooth_target[1]

    def _comfort_speed(self, dist: float) -> float:
        v_lim = math.sqrt(max(0.0, 2.0 * APPROACH_DECEL * dist))
        speed_cap = self.cfg.max_speed
        # When local planner is not yet connected to the final goal,
        # keep a conservative speed to avoid large detours and overshoot.
        if not self.core.state.connected_to_goal:
            speed_cap = min(speed_cap, 4.0)
        return min(speed_cap, v_lim)

    def _limit_jerk(self, ax: float, ay: float) -> Tuple[float, float]:
        max_da = MAX_JERK_XY * self._dt_ctrl
        dax = ax - self._smooth_acc[0]
        day = ay - self._smooth_acc[1]
        if abs(dax) > max_da:
            ax = self._smooth_acc[0] + math.copysign(max_da, dax)
        if abs(day) > max_da:
            ay = self._smooth_acc[1] + math.copysign(max_da, day)
        return ax, ay

    def _filter_acc(self, ax: float, ay: float, az: float) -> Tuple[float, float, float]:
        ax, ay = self._limit_jerk(ax, ay)
        a = self._alpha_from_tau(self.cfg.acc_smooth_tau)
        self._smooth_acc[0] += a * (ax - self._smooth_acc[0])
        self._smooth_acc[1] += a * (ay - self._smooth_acc[1])
        self._smooth_acc[2] += a * (az - self._smooth_acc[2])
        return self._smooth_acc[0], self._smooth_acc[1], self._smooth_acc[2]

    def _smooth_yaw_cmd(self, desired_yaw: float, current_yaw: float) -> Tuple[float, float]:
        if self._smooth_yaw is None:
            self._smooth_yaw = current_yaw
        err = math.atan2(
            math.sin(desired_yaw - self._smooth_yaw),
            math.cos(desired_yaw - self._smooth_yaw),
        )
        a = self._alpha_from_tau(self.cfg.yaw_smooth_tau)
        yaw_dot = max(-MAX_YAW_RATE, min(MAX_YAW_RATE, err * a / max(self._dt_ctrl, 1e-3)))
        self._smooth_yaw += yaw_dot * self._dt_ctrl
        self._smooth_yaw = math.atan2(math.sin(self._smooth_yaw), math.cos(self._smooth_yaw))
        return self._smooth_yaw, yaw_dot

    def _sample_yaw_ref(self, t: float, fallback: float) -> float:
        yaw_pts = self.core.state.yaw_traj
        if not yaw_pts:
            return fallback
        if t <= yaw_pts[0].t:
            return yaw_pts[0].yaw
        if t >= yaw_pts[-1].t:
            return yaw_pts[-1].yaw
        i = 0
        while i + 1 < len(yaw_pts) and yaw_pts[i + 1].t < t:
            i += 1
        a = yaw_pts[i]
        b = yaw_pts[i + 1]
        dt = max(b.t - a.t, 1e-6)
        r = (t - a.t) / dt
        dy = math.atan2(math.sin(b.yaw - a.yaw), math.cos(b.yaw - a.yaw))
        return math.atan2(math.sin(a.yaw + r * dy), math.cos(a.yaw + r * dy))

    def _fill_tracking_cmd(
        self, cmd, x, y, z, vx, vy, vz, yaw, target_x, target_y
    ) -> None:
        dx = target_x - x
        dy = target_y - y
        dist = math.hypot(dx, dy)
        if dist < 1e-3:
            self._fill_hover_cmd(cmd, x, y, z, yaw, vx, vy, vz)
            return
        ux, uy = dx / dist, dy / dist
        speed = self._comfort_speed(dist)
        vdes_x, vdes_y = ux * speed, uy * speed
        ax = KX_XY * dx + KV_XY * (vdes_x - vx)
        ay = KX_XY * dy + KV_XY * (vdes_y - vy)
        acc_norm = math.hypot(ax, ay)
        if acc_norm > MAX_ACC_XY:
            s = MAX_ACC_XY / acc_norm
            ax, ay = ax * s, ay * s
        dz = self.cfg.flight_height - z
        az = max(-MAX_ACC_Z, min(MAX_ACC_Z, KX_Z * dz + KV_Z * (0.0 - vz)))
        ax, ay, az = self._filter_acc(ax, ay, az)
        cmd.position.x = target_x
        cmd.position.y = target_y
        cmd.position.z = self.cfg.flight_height
        cmd.velocity.x = vdes_x
        cmd.velocity.y = vdes_y
        cmd.velocity.z = 0.0
        cmd.acceleration.x = ax
        cmd.acceleration.y = ay
        cmd.acceleration.z = az
        cmd.yaw, cmd.yaw_dot = self._smooth_yaw_cmd(math.atan2(uy, ux), yaw)

    def _fill_traj_sample_cmd(
        self,
        cmd,
        x: float,
        y: float,
        vx: float,
        vy: float,
        z: float,
        vz: float,
        yaw: float,
        target_x: float,
        target_y: float,
        target_vx: float,
        target_vy: float,
        target_ax: float,
        target_ay: float,
        yaw_ref: float,
    ) -> None:
        # For TRAJECTORY_STATUS_READY, controller uses acceleration directly.
        # Add tracking feedback on top of feedforward trajectory acceleration.
        ex = target_x - x
        ey = target_y - y
        evx = target_vx - vx
        evy = target_vy - vy
        ax_cmd = target_ax + KX_XY * ex + KV_XY * evx
        ay_cmd = target_ay + KX_XY * ey + KV_XY * evy
        axy = math.hypot(ax_cmd, ay_cmd)
        if axy > MAX_ACC_XY:
            s = MAX_ACC_XY / axy
            ax_cmd *= s
            ay_cmd *= s

        dz = self.cfg.flight_height - z
        az = max(-MAX_ACC_Z, min(MAX_ACC_Z, KX_Z * dz + KV_Z * (0.0 - vz)))
        ax, ay, az = self._filter_acc(ax_cmd, ay_cmd, az)
        cmd.position.x = target_x
        cmd.position.y = target_y
        cmd.position.z = self.cfg.flight_height
        cmd.velocity.x = target_vx
        cmd.velocity.y = target_vy
        cmd.velocity.z = 0.0
        cmd.acceleration.x = ax
        cmd.acceleration.y = ay
        cmd.acceleration.z = az
        cmd.yaw, cmd.yaw_dot = self._smooth_yaw_cmd(yaw_ref, yaw)

    def _fill_hover_cmd(self, cmd, x, y, z, yaw, vx, vy, vz) -> None:
        if self._hover_anchor is not None:
            hx, hy, hz = self._hover_anchor
        else:
            # Keep current altitude by default to avoid unintended climb/descent pulses.
            hx, hy, hz = x, y, z
        cmd.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_EMPTY
        cmd.position.x = hx
        cmd.position.y = hy
        cmd.position.z = hz
        cmd.velocity.x = cmd.velocity.y = cmd.velocity.z = 0.0

        # Precise hover hold: lock at anchor with damped acceleration.
        ex = hx - x
        ey = hy - y
        ez = hz - z
        ax = HOVER_KP_XY * ex - HOVER_KD_XY * vx
        ay = HOVER_KP_XY * ey - HOVER_KD_XY * vy
        xy_norm = math.hypot(ax, ay)
        if xy_norm > HOVER_MAX_ACC_XY:
            scale = HOVER_MAX_ACC_XY / xy_norm
            ax *= scale
            ay *= scale
        az = HOVER_KP_Z * ez - HOVER_KD_Z * vz
        az = max(-HOVER_MAX_ACC_Z, min(HOVER_MAX_ACC_Z, az))
        cmd.acceleration.x = ax
        cmd.acceleration.y = ay
        cmd.acceleration.z = az
        cmd.yaw = yaw
        cmd.yaw_dot = 0.0

    def _goal_reached_and_stable(self, x: float, y: float, vx: float, vy: float) -> bool:
        gx, gy = self.goal[0], self.goal[1]
        pos_ok = math.hypot(gx - x, gy - y) < GOAL_ARRIVE_POS_TOL
        vel_ok = math.hypot(vx, vy) < GOAL_ARRIVE_VEL_TOL
        if pos_ok and vel_ok:
            self._goal_arrive_count += 1
        else:
            self._goal_arrive_count = 0
        return self._goal_arrive_count >= GOAL_ARRIVE_HOLD_FRAMES

    def _odom_pos(self) -> Tuple[float, float, float]:
        assert self.odom is not None
        p = self.odom.pose.pose.position
        return p.x, p.y, p.z

    def _odom_vel(self) -> Tuple[float, float, float]:
        assert self.odom is not None
        v = self.odom.twist.twist.linear
        return v.x, v.y, v.z

    def _odom_yaw(self) -> float:
        assert self.odom is not None
        q = self.odom.pose.pose.orientation
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _body_to_world(self, x: float, y: float, z: float) -> Tuple[float, float, float]:
        assert self.odom is not None
        p = self.odom.pose.pose.position
        q = self.odom.pose.pose.orientation
        qw, qx, qy, qz = q.w, q.x, q.y, q.z
        r00 = 1.0 - 2.0 * (qy * qy + qz * qz)
        r01 = 2.0 * (qx * qy - qz * qw)
        r02 = 2.0 * (qx * qz + qy * qw)
        r10 = 2.0 * (qx * qy + qz * qw)
        r11 = 1.0 - 2.0 * (qx * qx + qz * qz)
        r12 = 2.0 * (qy * qz - qx * qw)
        r20 = 2.0 * (qx * qz - qy * qw)
        r21 = 2.0 * (qy * qz + qx * qw)
        r22 = 1.0 - 2.0 * (qx * qx + qy * qy)
        return (
            r00 * x + r01 * y + r02 * z + p.x,
            r10 * x + r11 * y + r12 * z + p.y,
            r20 * x + r21 * y + r22 * z + p.z,
        )

    @staticmethod
    def _iter_xyz(msg: PointCloud2) -> Iterable[Tuple[float, float, float]]:
        return point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SuperCompatPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
