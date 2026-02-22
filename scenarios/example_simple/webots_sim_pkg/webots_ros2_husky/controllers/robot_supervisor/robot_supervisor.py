#!/usr/bin/env python3
"""
Robot Supervisor 
- Webots world에서 여러 로봇의 translation/rotation을 읽어서
  /{robot_id}/pose_world (PoseStamped) 로 publish

- 로봇간 local communication 중계:
  /{robot_id}/local_comm/outbox (JSON) 를 수집하고
  수신 로봇의 comm_radius 기준으로 이웃만 필터링하여
  /{robot_id}/local_comm/inbox (JSON array) 로 publish

권장:
- 로봇 고유 ID는 DEF 사용 (robot_launch.py도 DEF 기반)
"""

# Webots 월드에서 pose_world를 publish할 로봇 DEF prefix 목록
# robot_launch.py의 ROBOTS_NAME_LIST와 동일하게 유지할 것
ROBOT_DEF_PREFIXES = ["Fire_UGV"]
OUTBOX_TIMEOUT = 0.5  # seconds: 이 시간 이상 outbox 미수신 시 stale로 판정

from controller import Supervisor
import json
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


def axis_angle_to_quaternion(axis_x, axis_y, axis_z, angle):
    """axis-angle -> quaternion (x,y,z,w)"""
    norm = math.sqrt(axis_x * axis_x + axis_y * axis_y + axis_z * axis_z)
    if norm < 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    axis_x, axis_y, axis_z = axis_x / norm, axis_y / norm, axis_z / norm
    sin_half = math.sin(angle * 0.5)
    return (axis_x * sin_half, axis_y * sin_half, axis_z * sin_half, math.cos(angle * 0.5))


class TrackedRobot:
    def __init__(self, ros_node: Node, wb_node, robot_id: str, frame_id_world: str):
        self.node = ros_node
        self.wb_node = wb_node
        self.robot_id = robot_id
        self.frame_id_world = frame_id_world

        self.translation_field = wb_node.getField("translation")
        self.rotation_field = wb_node.getField("rotation")

        # pose publisher: Webots 월드에서 읽은 pose를 ROS topic으로 publish
        self.pub_pose_world = ros_node.create_publisher(
            PoseStamped, f"/{self.robot_id}/pose_world", 10
        )

        # local comm: 로봇이 broadcast하는 outbox 수신
        self.sub_outbox = ros_node.create_subscription(
            String,
            f"/{self.robot_id}/local_comm/outbox",
            self._on_outbox,
            10
        )

        # local comm: 이웃 정보를 로봇에게 전달하는 inbox 송신
        self.pub_inbox = ros_node.create_publisher(
            String, f"/{self.robot_id}/local_comm/inbox", 10
        )

        self.last_outbox = None       # 최신 outbox dict (None = 아직 수신 전)
        self.last_outbox_time = None  # 마지막 수신 시각 (stale 감지용)

    def _on_outbox(self, msg: String):
        try:
            self.last_outbox = json.loads(msg.data)
            self.last_outbox_time = time.time()
        except json.JSONDecodeError as e:
            self.node.get_logger().warn(f"[{self.robot_id}] outbox JSON parse error: {e}")

    def get_position_2d(self):
        """Webots translation field에서 (x, y) 반환"""
        t = self.translation_field.getSFVec3f()
        return (t[0], t[1])

    def publish_world_pose(self):
        t = self.translation_field.getSFVec3f()
        r = self.rotation_field.getSFRotation()
        q = axis_angle_to_quaternion(r[0], r[1], r[2], r[3])

        msg = PoseStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id_world
        msg.pose.position.x = float(t[0])
        msg.pose.position.y = float(t[1])
        msg.pose.position.z = float(t[2])
        msg.pose.orientation.x = float(q[0])
        msg.pose.orientation.y = float(q[1])
        msg.pose.orientation.z = float(q[2])
        msg.pose.orientation.w = float(q[3])

        self.pub_pose_world.publish(msg)

    def publish_inbox(self, neighbors: list):
        msg = String()
        msg.data = json.dumps(neighbors)
        self.pub_inbox.publish(msg)


class RobotSupervisor:
    def __init__(self):
        self.supervisor = Supervisor()
        self.timestep = int(self.supervisor.getBasicTimeStep())

        rclpy.init(args=None)
        self.node = rclpy.create_node("robot_supervisor")
        self.log = self.node.get_logger()

        self.frame_id_world = "world"
        self.def_prefixes = ROBOT_DEF_PREFIXES

        self.tracked = []
        self._discover_robots_by_def()

        self.log.info("===== robot_supervisor started =====")
        self.log.info(f"Tracked robots (DEF): {[r.robot_id for r in self.tracked]}")

    def _discover_robots_by_def(self):
        root = self.supervisor.getRoot()
        children_field = root.getField("children")

        found = []
        for i in range(children_field.getCount()):
            wb_node = children_field.getMFNode(i)
            if wb_node is None:
                continue

            def_name = wb_node.getDef()
            if not def_name:
                continue

            if any(def_name == p or def_name.startswith(p + "_") or def_name.startswith(p)
                   for p in self.def_prefixes):
                found.append((def_name, wb_node))

        found.sort(key=lambda x: x[0])

        for def_name, wb_node in found:
            try:
                self.tracked.append(
                    TrackedRobot(self.node, wb_node, def_name, self.frame_id_world)
                )
            except Exception as e:
                self.log.warn(f"Failed to track robot DEF='{def_name}': {e}")

    

    def _is_stale(self, robot) -> bool:
        if robot.last_outbox_time is None:
            return True
        return (time.time() - robot.last_outbox_time) > OUTBOX_TIMEOUT

    def _relay_communications(self):
        """수신 로봇의 comm_radius 기준으로 이웃 outbox를 inbox에 전달"""
        for receiver in self.tracked:
            if self._is_stale(receiver):
                continue  # 수신 로봇 자체가 stale이면 inbox 계산 불필요

            comm_radius = receiver.last_outbox.get("comm_radius", 0.0)
            rx, ry = receiver.get_position_2d()

            neighbors = []
            for sender in self.tracked:
                if sender.robot_id == receiver.robot_id:
                    continue
                if self._is_stale(sender):
                    continue  # stale sender는 이웃에서 제외

                sx, sy = sender.get_position_2d()
                dist = math.sqrt((rx - sx) ** 2 + (ry - sy) ** 2)
                if dist <= comm_radius:
                    neighbors.append(sender.last_outbox)

            receiver.publish_inbox(neighbors)

    def run(self):
        while self.supervisor.step(self.timestep) != -1 and rclpy.ok():
            for robot in self.tracked:
                robot.publish_world_pose()

            rclpy.spin_once(self.node, timeout_sec=0.0)
            self._relay_communications()

        self.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    RobotSupervisor().run()
