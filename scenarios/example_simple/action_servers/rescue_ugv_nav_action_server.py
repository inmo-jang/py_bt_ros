#!/usr/bin/env python3
"""
Rescue UGV NavigateToPose Action Server (Refactored + Robust Scan Normalization)
- Gap-based obstacle avoidance (틈새 주행 알고리즘)
- INF(개활지) 반영 + 거리 클리핑(<=0.5 -> 0.0, >=12 -> inf)
- angle_increment < 0 인 scan 정규화 (좌/우 뒤집힘 방지)
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


class RescueUGVNavigateServer(Node):
    """
    NavigateToPose Action Server for Rescue UGV
    - Logic: Goal tracking + Gap-based obstacle avoidance
    - Robust scan preprocessing:
        * normalize reversed scan (angle_increment < 0)
        * <= 0.5m -> 0.0 (blocked)
        * >= 12m -> inf (open)
        * inf included in gap candidates (treated as far distance)
    """

    def __init__(self, ns: str = ""):
        ns = (ns or "").strip("/")
        if ns and not ns.startswith("/"):
            ns = "/" + ns
        super().__init__("rescue_ugv_navigate_server", namespace=ns)

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
            f"RescueUGVNavigateServer started. "
            f"Namespace: {self.get_namespace()}, Action: navigate_to_pose"
        )

    def _declare_parameters(self):
        # 제어 주기 (Hz)
        self.declare_parameter("control_rate", 30.0)  # 20.0

        # 최대 속도 제한
        self.declare_parameter("max_linear_vel", 1.0)  # 1.0
        self.declare_parameter("max_angular_vel", 0.5)  # 1.0

        # 목표 추종 게인
        self.declare_parameter("k_linear", 0.8)  # 0.5
        self.declare_parameter("k_angular", 0.8)  # 1.0

        # 부드러운 이동 설정
        self.declare_parameter("min_linear_scale", 0.15)
        self.declare_parameter("heading_tolerance", 0.05)

        # 목표 도착 판정
        self.declare_parameter("goal_tolerance", 0.3)

        # 장애물 회피 설정 (Gap-based)
        self.declare_parameter("min_passable_dist", 2.0)        # 통과 가능 최소 거리
        self.declare_parameter("robot_width", 1.0)              # 로봇 폭 + 여유  1.0
        self.declare_parameter("obstacle_distance_slow", 5.0)   # 감속 시작 거리
        self.declare_parameter("obstacle_distance_stop", 1.2)   # 긴급 정지 거리
        self.declare_parameter("front_sector_angle", 45.0)      # 전방 감시 각도 (deg)
        self.declare_parameter("avoidance_gain", 4.0)           # 회피 강도

        # ✅ Scan preprocessing 요구사항 반영
        self.declare_parameter("clip_close_dist", 0.5)          # <= 이하면 0.0 처리  0.5
        self.declare_parameter("clip_open_dist", 12.0)          # >= 이면 inf 처리

        # Gap scoring weights (goal 정렬 반영 강화)
        self.declare_parameter("w_goal_align", 1.0)             # 목표 정렬 가중치
        self.declare_parameter("w_gap_width", 0.3)             # gap 폭 가중치 0.25
        self.declare_parameter("w_gap_dist", 0.10)              # gap 거리 가중치

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

    def _pose_callback(self, msg: PoseStamped):
        self._current_pose = msg

    # ------------------------------------------------------------
    # Scan preprocessing (핵심)
    # ------------------------------------------------------------
    def _scan_callback(self, msg: LaserScan):
        """
        1) angle_increment < 0 이면 scan을 뒤집어 정규화 (angle_min < angle_max, inc > 0)
        2) 거리 클리핑:
           - <= clip_close_dist -> 0.0 (막힘)
           - >= clip_open_dist  -> inf (개활지)
        3) INF는 gap 후보로 포함되도록 유지
        """
        self._raw_scan = msg

        ranges = list(msg.ranges)
        angle_min = float(msg.angle_min)
        angle_inc = float(msg.angle_increment)

        # (A) Normalize reversed scan (so angles increase with index)
        if angle_inc < 0.0:
            ranges.reverse()
            angle_min = float(msg.angle_max)
            angle_inc = -angle_inc

        # (B) Clip ranges as requested
        proc = []
        for r in ranges:
            if not math.isfinite(r):
                # keep INF as INF (open)
                proc.append(float('inf'))
                continue

            rr = float(r)
            if rr <= self.clip_close_dist:
                proc.append(0.0)              # blocked
            elif rr >= self.clip_open_dist:
                proc.append(float('inf'))     # open
            else:
                proc.append(rr)

        self._scan_ranges = proc
        self._scan_angle_min = angle_min
        self._scan_angle_inc = angle_inc

    def _goal_callback(self, goal_request) -> GoalResponse:
        self.get_logger().info(
            f"Received goal: x={goal_request.pose.pose.position.x:.2f}, "
            f"y={goal_request.pose.pose.position.y:.2f}"
        )
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        self.get_logger().info("Goal cancellation requested")
        return CancelResponse.ACCEPT

    def _publish_cmd(self, linear: float, angular: float):
        # angular.z > 0 => left turn (as user confirmed)
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.twist.linear.x = float(linear)
        msg.twist.angular.z = float(angular)
        self._cmd_pub.publish(msg)

    def _stop(self):
        self._publish_cmd(0.0, 0.0)

    # ================= Gap Utilities =================

    def _scan_ready(self) -> bool:
        return self._scan_ranges is not None and self._raw_scan is not None and len(self._scan_ranges) > 0

    def _range_eff(self, r: float) -> float:
        """
        INF를 gap 계산에 반영하기 위한 '유효 거리'로 변환.
        - r == inf => clip_open_dist 로 간주 (아주 멀리 트여있음)
        - r == 0.0 => 0.0 (막힘)
        - finite => 그대로
        """
        if math.isfinite(r):
            return r
        return self.clip_open_dist

    def _angle_at(self, idx: float) -> float:
        """idx -> angle (rad), 0 rad = forward"""
        return self._scan_angle_min + idx * self._scan_angle_inc

    def _find_gaps(self) -> List[Dict]:
        """
        LiDAR 데이터에서 통과 가능한 gap들을 찾음.
        - INF도 passable로 인정 (clip_open_dist로 계산)
        - gap 폭은 (min_dist * angular_width) 근사 사용
        """
        if not self._scan_ready():
            return []

        ranges = self._scan_ranges
        n = len(ranges)
        if n == 0:
            return []

        # 1) passable indices (INF 포함)
        passable = []
        for i, r in enumerate(ranges):
            r_eff = self._range_eff(r)
            if r_eff >= self.min_passable_dist:
                passable.append(i)

        if not passable:
            return []

        # 2) group consecutive indices into gaps
        gaps_idx = []
        gap_start = passable[0]
        prev = passable[0]
        for idx in passable[1:]:
            if idx != prev + 1:
                gaps_idx.append((gap_start, prev))
                gap_start = idx
            prev = idx
        gaps_idx.append((gap_start, prev))

        # 3) compute gap properties
        result = []
        for start_idx, end_idx in gaps_idx:
            # min distance inside gap (INF -> clip_open_dist)
            min_dist = float('inf')
            for i in range(start_idx, end_idx + 1):
                r_eff = self._range_eff(ranges[i])
                if r_eff < min_dist:
                    min_dist = r_eff

            if not math.isfinite(min_dist):
                # theoretically shouldn't happen due to _range_eff
                min_dist = self.clip_open_dist

            start_angle = self._angle_at(start_idx)
            end_angle = self._angle_at(end_idx)
            angular_width = abs(end_angle - start_angle)

            center_idx = 0.5 * (start_idx + end_idx)
            center_angle = self._angle_at(center_idx)

            # physical width approximation
            gap_width = min_dist * angular_width

            if gap_width >= self.robot_width:
                result.append({
                    "start_idx": start_idx,
                    "end_idx": end_idx,
                    "center_angle": center_angle,  # rad, +left
                    "min_dist": min_dist,
                    "width": gap_width,
                })

        return result

    def _get_front_min_distance(self) -> float:
        """전방 섹터의 최소 거리 반환 (INF는 clip_open_dist로 간주)"""
        if not self._scan_ready():
            return float('inf')

        ranges = self._scan_ranges
        min_dist = float('inf')

        for i, r in enumerate(ranges):
            angle = self._angle_at(i)
            if abs(angle) > self.front_sector_angle:
                continue

            r_eff = self._range_eff(r)
            if r_eff < min_dist:
                min_dist = r_eff

        return min_dist

    def _compute_obstacle_avoidance(self, goal_heading_error: float) -> Tuple[float, float, float]:
        """
        Gap-based 장애물 회피 계산.
        반환: (front_min, avoidance_angular, danger_level)
        """
        front_min = self._get_front_min_distance()

        # danger level
        if front_min <= self.obstacle_distance_stop:
            danger_level = 1.0
        elif front_min >= self.obstacle_distance_slow:
            danger_level = 0.0
        else:
            danger_level = (self.obstacle_distance_slow - front_min) / (
                self.obstacle_distance_slow - self.obstacle_distance_stop
            )

        # not dangerous -> no avoidance
        if danger_level < 0.1:
            return front_min, 0.0, danger_level

        gaps = self._find_gaps()
        if not gaps:
            # No gap: rotate away from goal heading direction (heuristic)
            # heading_error > 0 => goal is left => rotate right (negative)
            spin = -self.avoidance_gain if goal_heading_error > 0.0 else self.avoidance_gain
            return front_min, spin, danger_level

        # Choose best gap using:
        # - goal alignment (closest to goal_heading_error)
        # - wide gap
        # - far gap
        best_gap = None
        best_score = -float("inf")

        for g in gaps:
            # 얼마나 목표 방향(상대각)과 잘 맞는가
            align_err = abs(normalize_angle(g["center_angle"] - goal_heading_error))
            goal_term = -align_err  # closer is better

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

        # angle_to_gap: +면 왼쪽, -면 오른쪽
        angle_to_gap = best_gap["center_angle"]
        avoidance_angular = self.avoidance_gain * angle_to_gap * danger_level

        return front_min, avoidance_angular, danger_level

    def _compute_velocity(self, goal_x: float, goal_y: float) -> Tuple[float, float]:
        """Goal tracking + obstacle avoidance blending"""
        if self._current_pose is None:
            return 0.0, 0.0

        px = self._current_pose.pose.position.x
        py = self._current_pose.pose.position.y
        yaw = quaternion_to_yaw(self._current_pose.pose.orientation)

        dx = goal_x - px
        dy = goal_y - py
        distance = math.hypot(dx, dy)

        target_yaw = math.atan2(dy, dx)
        heading_error = normalize_angle(target_yaw - yaw)  # +면 목표가 왼쪽

        # Goal tracking
        goal_angular = clamp(self.k_angular * heading_error, -self.max_angular_vel, self.max_angular_vel)

        goal_linear = self.k_linear * distance
        heading_factor = max(math.cos(heading_error), 0.0)
        speed_scale = self.min_linear_scale + (1.0 - self.min_linear_scale) * heading_factor
        goal_linear *= speed_scale

        # Obstacle avoidance (gap)
        front_dist, avoidance_angular, danger_level = self._compute_obstacle_avoidance(heading_error)

        if danger_level >= 1.0:
            # emergency: almost stop and rotate to escape
            linear_vel = 0.05
            angular_vel = clamp(avoidance_angular, -self.max_angular_vel, self.max_angular_vel)
            self.get_logger().warn(f"Emergency! dist={front_dist:.2f}m", throttle_duration_sec=0.5)
        else:
            blend = danger_level  # 0..1
            angular_vel = (1.0 - blend) * goal_angular + blend * avoidance_angular
            angular_vel = clamp(angular_vel, -self.max_angular_vel, self.max_angular_vel)

            speed_reduction = 1.0 - (0.7 * blend)
            linear_vel = goal_linear * speed_reduction

        linear_vel = clamp(linear_vel, 0.0, self.max_linear_vel)
        return linear_vel, angular_vel

    async def _execute_callback(self, goal_handle):
        self.get_logger().info("Executing navigation goal...")

        goal_pose = goal_handle.request.pose
        goal_x = goal_pose.pose.position.x
        goal_y = goal_pose.pose.position.y

        start_time = self.get_clock().now()
        sleep_duration = 1.0 / self.control_rate

        result = NavigateToPose.Result()

        try:
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    self._stop()
                    goal_handle.canceled()
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
                    nanosec=int((elapsed_sec % 1) * 1e9)
                )
                goal_handle.publish_feedback(feedback)

                time.sleep(sleep_duration)

        except Exception as e:
            self.get_logger().error(f"Navigation error: {e}")
            self._stop()
            goal_handle.abort()
            result.error_code = 1
            result.error_msg = str(e)
            return result

        self._stop()
        goal_handle.abort()
        return result

    def destroy_node(self):
        self._stop()
        self._action_server.destroy()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    # 네임스페이스 설정
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--ns", type=str, default="", help="ROS namespace, e.g. /Rescue_UGV_1")
    args = parser.parse_args()

    node = RescueUGVNavigateServer(ns=args.ns)
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