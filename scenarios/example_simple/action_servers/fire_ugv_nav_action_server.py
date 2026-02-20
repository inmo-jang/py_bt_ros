#!/usr/bin/env python3
"""
Fire UGV NavigateToPose Action Server (Refactored + Robust Scan Normalization)
- Rescue UGV의 Scan 정규화/클리핑/INF 반영 로직을 동일하게 적용
- 기능 유지: '/world/fire/.../pose' 자동 탐색 + Fire 무시 주행(장애물로 인식하지 않음)
- angle_increment < 0 인 scan 정규화 (좌/우 뒤집힘 방지)
- 거리 클리핑(<=0.5 -> 0.0, >=12 -> inf)
- angular.z 양수 => 왼쪽 회전 (ROS 표준)
"""

import math
import time
from typing import Optional, Tuple, List, Dict

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import TwistStamped, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from nav2_msgs.action import NavigateToPose
from builtin_interfaces.msg import Duration as DurationMsg


def clamp(value: float, min_val: float, max_val: float) -> float:
    return max(min_val, min(max_val, value))


def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi]"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def quaternion_to_yaw(q) -> float:
    """Extract yaw from quaternion"""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class FireUGVNavigateServer(Node):
    """
    NavigateToPose Action Server for Fire UGV
    - Goal tracking + Gap-based obstacle avoidance
    - Fire ignore behavior (critical):
        * Fire zone angles are ALWAYS considered passable
        * Fire zone is excluded from obstacle distance(front_min) computation
    - Robust scan preprocessing:
        * normalize reversed scan (angle_increment < 0)
        * <= clip_close_dist -> 0.0 (blocked)
        * >= clip_open_dist -> inf (open)
        * inf included as passable candidate (treated as far distance)
    """

    def __init__(self, ns: str = ""):
        ns = (ns or "").strip("/")
        if ns and not ns.startswith("/"):
            ns = "/" + ns
        super().__init__("fire_ugv_navigate_server", namespace=ns)

        # Declare parameters
        self._declare_parameters()
        self._load_parameters()

        # State
        self._current_pose: Optional[PoseStamped] = None
        self._raw_scan: Optional[LaserScan] = None

        # Processed scan cache (normalized + clipped)
        self._scan_ranges: Optional[List[float]] = None
        self._scan_angle_min: float = 0.0
        self._scan_angle_inc: float = 0.0

        # Fire data store
        self._fire_poses: Dict[str, PoseStamped] = {}
        self._fire_radii: Dict[str, float] = {}

        self._discovered_fires = set()
        self._fire_subs = []

        # Goal tracking to prevent parallel execution
        self._current_goal_handle = None

        # Callback group for async
        self._cb_group = ReentrantCallbackGroup()

        # Publishers & Subscribers
        self._cmd_pub = self.create_publisher(TwistStamped, "cmd_vel", 10)

        self._pose_sub = self.create_subscription(
            PoseStamped, "pose_world", self._pose_callback, 10,
            callback_group=self._cb_group
        )
        self._scan_sub = self.create_subscription(
            LaserScan, "scan", self._scan_callback, 10,
            callback_group=self._cb_group
        )

        # Fire auto-discovery timer
        self._discovery_timer = self.create_timer(
            1.5, self._discovery_callback, callback_group=self._cb_group
        )

        # Action Server
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            "navigate_to_pose",
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._cb_group
        )

        self.get_logger().info(
            f"FireUGVNavigateServer started. Namespace: {self.get_namespace()}, Action: navigate_to_pose. "
            f"Scanning for fires..."
        )

    # -------------------------
    # Params
    # -------------------------
    def _declare_parameters(self):
        # Navigation params
        self.declare_parameter("control_rate", 30.0)  # 20.0
        self.declare_parameter("max_linear_vel", 1.5)  # 속도 증가 (1.0 -> 1.5)
        self.declare_parameter("max_angular_vel", 1.0)  # 1.0
        self.declare_parameter("k_linear", 0.8)  # 0.5
        self.declare_parameter("k_angular", 1.0)  # 1.0
        self.declare_parameter("min_linear_scale", 0.15)
        self.declare_parameter("heading_tolerance", 0.1) # 0.05
        self.declare_parameter("goal_tolerance", 0.3)

        # Obstacle avoidance params
        self.declare_parameter("min_passable_dist", 2.0)        # 통과 가능 최소 거리
        self.declare_parameter("robot_width", 1.0)              # 로봇 폭 + 여유  1.0
        self.declare_parameter("obstacle_distance_slow", 3.5)   # 감속 시작 거리 5.0
        self.declare_parameter("obstacle_distance_stop", 1.2)   # 긴급 정지 거리
        self.declare_parameter("front_sector_angle", 60.0)      # 전방 감시 각도 (deg) 45.0
        self.declare_parameter("avoidance_gain", 2.5)           # 회피 강도 4.0

        # ✅ Scan preprocessing 요구사항 반영
        self.declare_parameter("clip_close_dist", 0.5)          # <= 이하면 0.0 처리  0.5
        self.declare_parameter("clip_open_dist", 12.0)          # >= 이면 inf 처리

        # Gap scoring weights (goal 정렬 반영 강화)
        self.declare_parameter("w_goal_align", 1.0)             # 목표 정렬 가중치 1.0
        self.declare_parameter("w_gap_width", 0.25)             # gap 폭 가중치 0.25
        self.declare_parameter("w_gap_dist", 0.10)              # gap 거리 가중치

        # Fire params
        self.declare_parameter("default_fire_radius", 1.0)
        self.declare_parameter("fire_ignore_margin", 1.0)

    def _load_parameters(self):
        self.control_rate = float(self.get_parameter("control_rate").value)
        self.max_linear_vel = float(self.get_parameter("max_linear_vel").value)
        self.max_angular_vel = float(self.get_parameter("max_angular_vel").value)
        self.k_linear = float(self.get_parameter("k_linear").value)
        self.k_angular = float(self.get_parameter("k_angular").value)
        self.min_linear_scale = float(self.get_parameter("min_linear_scale").value)
        self.heading_tolerance = float(self.get_parameter("heading_tolerance").value)
        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)

        self.min_passable_dist = float(self.get_parameter("min_passable_dist").value)
        self.robot_width = float(self.get_parameter("robot_width").value)
        self.obstacle_distance_slow = float(self.get_parameter("obstacle_distance_slow").value)
        self.obstacle_distance_stop = float(self.get_parameter("obstacle_distance_stop").value)
        self.front_sector_angle = math.radians(float(self.get_parameter("front_sector_angle").value))
        self.avoidance_gain = float(self.get_parameter("avoidance_gain").value)

        self.clip_close_dist = float(self.get_parameter("clip_close_dist").value)
        self.clip_open_dist = float(self.get_parameter("clip_open_dist").value)

        self.w_goal_align = float(self.get_parameter("w_goal_align").value)
        self.w_gap_width = float(self.get_parameter("w_gap_width").value)
        self.w_gap_dist = float(self.get_parameter("w_gap_dist").value)

        self.default_fire_radius = float(self.get_parameter("default_fire_radius").value)
        self.fire_ignore_margin = float(self.get_parameter("fire_ignore_margin").value)

    # -------------------------
    # Callbacks
    # -------------------------
    def _pose_callback(self, msg: PoseStamped):
        self._current_pose = msg

    def _scan_callback(self, msg: LaserScan):
        """
        1) angle_increment < 0 이면 scan을 뒤집어 정규화
        2) 거리 클리핑:
           - <= clip_close_dist -> 0.0
           - >= clip_open_dist  -> inf
        """
        self._raw_scan = msg

        ranges = list(msg.ranges)
        angle_min = float(msg.angle_min)
        angle_inc = float(msg.angle_increment)

        # Normalize reversed scan
        if angle_inc < 0.0:
            ranges.reverse()
            angle_min = float(msg.angle_max)
            angle_inc = -angle_inc

        proc = []
        for r in ranges:
            if not math.isfinite(r):
                proc.append(float("inf"))
                continue

            rr = float(r)
            if rr <= self.clip_close_dist:
                proc.append(0.0)
            elif rr >= self.clip_open_dist:
                proc.append(float("inf"))
            else:
                proc.append(rr)

        self._scan_ranges = proc
        self._scan_angle_min = angle_min
        self._scan_angle_inc = angle_inc

    def _fire_pose_callback(self, msg: PoseStamped, fire_id: str):
        self._fire_poses[fire_id] = msg

    def _fire_radius_callback(self, msg: Float64, fire_id: str):
        self._fire_radii[fire_id] = float(msg.data)

    def _goal_callback(self, goal_request) -> GoalResponse:
        self.get_logger().info(
            f"Received goal: x={goal_request.pose.pose.position.x:.2f}, "
            f"y={goal_request.pose.pose.position.y:.2f}"
        )
        # Cancel previous goal if exists
        if self._current_goal_handle is not None:
            self.get_logger().info("Canceling previous goal for new goal")
            try:
                self._current_goal_handle.abort()
            except Exception:
                pass
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        self.get_logger().info("Goal cancellation requested")
        return CancelResponse.ACCEPT

    # -------------------------
    # Auto Discovery (Fire)
    # -------------------------
    def _discovery_callback(self):
        topic_names_and_types = self.get_topic_names_and_types()
        for topic_name, _topic_types in topic_names_and_types:
            if topic_name.startswith("/world/fire/") and topic_name.endswith("/pose"):
                parts = topic_name.split("/")
                if len(parts) >= 5:
                    fire_id = parts[3]
                    if fire_id not in self._discovered_fires:
                        self.get_logger().info(f"New Fire Detected: {fire_id}")
                        self._subscribe_to_fire(fire_id)
                        self._discovered_fires.add(fire_id)

    def _subscribe_to_fire(self, fire_id: str):
        pose_topic = f"/world/fire/{fire_id}/pose"
        sub_pose = self.create_subscription(
            PoseStamped,
            pose_topic,
            lambda msg, fid=fire_id: self._fire_pose_callback(msg, fid),
            10,
            callback_group=self._cb_group,
        )
        self._fire_subs.append(sub_pose)

        radius_topic = f"/world/fire/{fire_id}/radius"
        sub_radius = self.create_subscription(
            Float64,
            radius_topic,
            lambda msg, fid=fire_id: self._fire_radius_callback(msg, fid),
            10,
            callback_group=self._cb_group,
        )
        self._fire_subs.append(sub_radius)

        self.get_logger().info(f"Subscribed to {fire_id} (Pose & Radius)")

    # -------------------------
    # Cmd helpers
    # -------------------------
    def _publish_cmd(self, linear: float, angular: float):
        # print(f"[CMD] linear={linear:.3f}, angular={angular:.3f}")  # 추가
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.twist.linear.x = float(linear)
        msg.twist.angular.z = float(angular)  # + => left
        self._cmd_pub.publish(msg)

    def _stop(self):
        self._publish_cmd(0.0, 0.0)

    # -------------------------
    # Scan utilities
    # -------------------------
    def _scan_ready(self) -> bool:
        return self._scan_ranges is not None and self._raw_scan is not None and len(self._scan_ranges) > 0

    def _angle_at(self, idx: float) -> float:
        """idx -> angle (rad), + left"""
        return self._scan_angle_min + idx * self._scan_angle_inc

    def _range_eff(self, r: float) -> float:
        """INF를 gap 계산에 반영하기 위한 유효 거리"""
        if math.isfinite(r):
            return r
        return self.clip_open_dist

    # -------------------------
    # Fire ignore zones (robot-frame angles)
    # -------------------------
    def _compute_fire_ignore_zones(self) -> List[Tuple[float, float]]:
        ignore_zones: List[Tuple[float, float]] = []
        if self._current_pose is None:
            return ignore_zones

        # 디버그 추가
        # print(f"[DEBUG] _fire_poses: {list(self._fire_poses.keys())}")
        # print(f"[DEBUG] _fire_radii: {self._fire_radii}")


        robot_x = self._current_pose.pose.position.x
        robot_y = self._current_pose.pose.position.y
        robot_yaw = quaternion_to_yaw(self._current_pose.pose.orientation)

        for fire_id, fire_pose in self._fire_poses.items():
            fire_x = fire_pose.pose.position.x
            fire_y = fire_pose.pose.position.y

            dx = fire_x - robot_x
            dy = fire_y - robot_y
            dist = math.hypot(dx, dy)
            if dist < 0.1:
                continue

            fire_dir_world = math.atan2(dy, dx)
            fire_dir_robot = normalize_angle(fire_dir_world - robot_yaw)

            current_radius = self._fire_radii.get(fire_id, self.default_fire_radius)
            effective_radius = current_radius + self.fire_ignore_margin

            half_angle = math.atan2(effective_radius, dist)
            # print(f"[FIRE_ZONE] {fire_id}: dir={fire_dir_robot:.2f}rad, half={half_angle:.2f}rad, dist={dist:.1f}m")
            ignore_zones.append((fire_dir_robot, half_angle))

        return ignore_zones

    def _is_in_fire_zone(self, angle: float, fire_zones: List[Tuple[float, float]]) -> bool:
        for fire_dir, half_angle in fire_zones:
            if abs(normalize_angle(angle - fire_dir)) < half_angle:
                return True
        return False

    # -------------------------
    # Gap finding (Fire-aware + INF-aware)
    # -------------------------
    def _find_gaps(self, fire_zones: List[Tuple[float, float]]) -> List[Dict]:
        if not self._scan_ready():
            return []

        ranges = self._scan_ranges
        n = len(ranges)
        if n == 0:
            return []

        # passable indices: (1) fire zone => passable (always)
        #                  (2) otherwise r_eff >= min_passable_dist
        passable: List[int] = []
        for i, r in enumerate(ranges):
            ang = self._angle_at(i)
            if self._is_in_fire_zone(ang, fire_zones):
                passable.append(i)
                continue

            if self._range_eff(r) >= self.min_passable_dist:
                passable.append(i)

        if not passable:
            return []

        # group consecutive passable indices
        gaps_idx = []
        gap_start = passable[0]
        prev = passable[0]
        for idx in passable[1:]:
            if idx != prev + 1:
                gaps_idx.append((gap_start, prev))
                gap_start = idx
            prev = idx
        gaps_idx.append((gap_start, prev))

        # compute gap properties (min_dist excludes fire zone)
        result: List[Dict] = []
        for start_idx, end_idx in gaps_idx:
            min_dist = float("inf")
            has_non_fire = False

            for i in range(start_idx, end_idx + 1):
                ang = self._angle_at(i)
                if self._is_in_fire_zone(ang, fire_zones):
                    continue
                has_non_fire = True
                r_eff = self._range_eff(ranges[i])
                if r_eff < min_dist:
                    min_dist = r_eff

            # if whole gap is inside fire zone => treat as very open
            if not has_non_fire:
                min_dist = self.clip_open_dist

            start_angle = self._angle_at(start_idx)
            end_angle = self._angle_at(end_idx)
            angular_width = abs(end_angle - start_angle)

            center_idx = 0.5 * (start_idx + end_idx)
            center_angle = self._angle_at(center_idx)

            gap_width = min_dist * angular_width
            if gap_width >= self.robot_width:
                result.append({
                    "start_idx": start_idx,
                    "end_idx": end_idx,
                    "center_angle": center_angle,
                    "min_dist": min_dist,
                    "width": gap_width,
                })

        return result

    def _get_front_min_distance(self, fire_zones: List[Tuple[float, float]]) -> float:
        """전방 섹터 최소거리 (Fire zone은 제외, INF는 clip_open_dist로 간주)"""
        if not self._scan_ready():
            return float("inf")

        ranges = self._scan_ranges
        min_dist = float("inf")

        for i, r in enumerate(ranges):
            ang = self._angle_at(i)

            if abs(ang) > self.front_sector_angle:
                continue
            if self._is_in_fire_zone(ang, fire_zones):
                continue

            r_eff = self._range_eff(r)
            if r_eff < min_dist:
                min_dist = r_eff

        return min_dist

    def _compute_obstacle_avoidance(
        self, goal_heading_error: float, fire_zones: List[Tuple[float, float]]
    ) -> Tuple[float, float, float]:
        """Return (front_min, avoidance_angular, danger_level)"""
        front_min = self._get_front_min_distance(fire_zones)

        if front_min <= self.obstacle_distance_stop:
            danger_level = 1.0
        elif front_min >= self.obstacle_distance_slow:
            danger_level = 0.0
        else:
            danger_level = (self.obstacle_distance_slow - front_min) / (
                self.obstacle_distance_slow - self.obstacle_distance_stop
            )

        if danger_level < 0.1:
            return front_min, 0.0, danger_level

        gaps = self._find_gaps(fire_zones)
        if not gaps:
            spin = -self.avoidance_gain if goal_heading_error > 0.0 else self.avoidance_gain
            return front_min, spin, danger_level

        best_gap = None
        best_score = -float("inf")
        for g in gaps:
            align_err = abs(normalize_angle(g["center_angle"] - goal_heading_error))
            goal_term = -align_err
            score = (
                self.w_goal_align * goal_term
                + self.w_gap_width * g["width"]
                + self.w_gap_dist * g["min_dist"]
            )
            if score > best_score:
                best_score = score
                best_gap = g

        if best_gap is None:
            return front_min, 0.0, danger_level

        print(f"[AVOID] front_min={front_min:.2f}m, danger={danger_level:.2f}")

        angle_to_gap = best_gap["center_angle"]  # +left
        avoidance_angular = self.avoidance_gain * angle_to_gap * danger_level
        return front_min, avoidance_angular, danger_level

    # -------------------------
    # Main velocity computation
    # -------------------------
    def _compute_velocity(self, goal_x: float, goal_y: float) -> Tuple[float, float]:
        if self._current_pose is None:
            return 0.0, 0.0

        px = self._current_pose.pose.position.x
        py = self._current_pose.pose.position.y
        yaw = quaternion_to_yaw(self._current_pose.pose.orientation)

        dx = goal_x - px
        dy = goal_y - py
        distance = math.hypot(dx, dy)

        target_yaw = math.atan2(dy, dx)
        heading_error = normalize_angle(target_yaw - yaw)  # + => goal is left

        # 디버그 추가
        # print(f"[VEL] goal=({goal_x:.1f},{goal_y:.1f}) robot=({px:.1f},{py:.1f}) heading_err={heading_error:.2f}rad")

        # Fire zones (robot-frame)
        fire_zones = self._compute_fire_ignore_zones()

        # Goal tracking
        goal_angular = clamp(self.k_angular * heading_error, -self.max_angular_vel, self.max_angular_vel)

        goal_linear = self.k_linear * distance
        heading_factor = max(math.cos(heading_error), 0.0)
        speed_scale = self.min_linear_scale + (1.0 - self.min_linear_scale) * heading_factor
        goal_linear *= speed_scale

        # Obstacle avoidance (Fire-aware)
        front_dist, avoidance_angular, danger_level = self._compute_obstacle_avoidance(heading_error, fire_zones)

        if danger_level >= 1.0:
            linear_vel = 0.05
            angular_vel = clamp(avoidance_angular, -self.max_angular_vel, self.max_angular_vel)
            self.get_logger().warn(f"Emergency! dist={front_dist:.2f}m", throttle_duration_sec=0.5)
        else:
            blend = danger_level
            angular_vel = (1.0 - blend) * goal_angular + blend * avoidance_angular
            angular_vel = clamp(angular_vel, -self.max_angular_vel, self.max_angular_vel)

            speed_reduction = 1.0 - (0.7 * blend)
            linear_vel = goal_linear * speed_reduction

        linear_vel = clamp(linear_vel, 0.0, self.max_linear_vel)
        return linear_vel, angular_vel

    # -------------------------
    # Action execute
    # -------------------------
    async def _execute_callback(self, goal_handle):
        self.get_logger().info("Executing navigation goal...")
        
        # Register this goal as current
        self._current_goal_handle = goal_handle

        goal_pose = goal_handle.request.pose
        goal_x = goal_pose.pose.position.x
        goal_y = goal_pose.pose.position.y

        start_time = self.get_clock().now()
        sleep_duration = 1.0 / self.control_rate

        result = NavigateToPose.Result()

        try:
            while rclpy.ok():
                # Check if this goal was superseded by a new one
                if self._current_goal_handle != goal_handle:
                    self.get_logger().info("Goal superseded by new goal, stopping this execution")
                    self._stop()
                    result.error_code = 0
                    result.error_msg = "Goal superseded"
                    try:
                        goal_handle.abort()
                    except Exception:
                        pass  # Goal already aborted
                    return result
                    
                if goal_handle.is_cancel_requested:
                    self._stop()
                    goal_handle.canceled()
                    self._current_goal_handle = None
                    result.error_code = 0
                    result.error_msg = "Navigation canceled"
                    return result

                if self._current_pose is None:
                    self.get_logger().warn("Waiting for pose...", throttle_duration_sec=2.0)
                    self._stop()
                    time.sleep(sleep_duration)
                    continue

                if not self._scan_ready():
                    self.get_logger().warn("Waiting for scan...", throttle_duration_sec=2.0)
                    self._stop()
                    time.sleep(sleep_duration)
                    continue

                px = self._current_pose.pose.position.x
                py = self._current_pose.pose.position.y
                distance = math.hypot(goal_x - px, goal_y - py)

                if distance <= self.goal_tolerance:
                    self._stop()
                    goal_handle.succeed()
                    self._current_goal_handle = None
                    result.error_code = 0
                    result.error_msg = "Goal reached"
                    self.get_logger().info(f"Goal reached! Dist: {distance:.3f}m")
                    return result

                linear_vel, angular_vel = self._compute_velocity(goal_x, goal_y)
                self._publish_cmd(linear_vel, angular_vel)

                feedback = NavigateToPose.Feedback()
                feedback.current_pose = self._current_pose
                feedback.distance_remaining = float(distance)

                elapsed = self.get_clock().now() - start_time
                elapsed_sec = elapsed.nanoseconds / 1e9
                feedback.navigation_time = DurationMsg(
                    sec=int(elapsed_sec),
                    nanosec=int((elapsed_sec % 1) * 1e9),
                )
                goal_handle.publish_feedback(feedback)

                time.sleep(sleep_duration)

        except Exception as e:
            self.get_logger().error(f"Navigation error: {e}")
            self._stop()
            self._current_goal_handle = None
            try:
                goal_handle.abort()
            except Exception:
                pass
            result.error_code = 1
            result.error_msg = str(e)
            return result

        self._stop()
        try:
            goal_handle.abort()
        except Exception:
            pass
        return result

    def destroy_node(self):
        self._stop()
        self._action_server.destroy()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--ns", type=str, default="", help="ROS namespace, e.g. /Fire_UGV_1")
    args = parser.parse_args()

    node = FireUGVNavigateServer(ns=args.ns)
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()