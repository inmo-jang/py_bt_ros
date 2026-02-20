#!/usr/bin/env python3
"""
Robot Supervisor (UGV side)
- Webots world에서 여러 로봇의 translation/rotation을 읽어서
  /robot/<DEF>/pose_world (PoseStamped) 로 publish

권장:
- 로봇 고유 ID는 DEF 사용 (robot_launch.py도 DEF 기반)
"""

from controller import Supervisor
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


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

        self.pub_pose_world = ros_node.create_publisher(
            PoseStamped, f"/{self.robot_id}/pose_world", 10
        )

    def publish_world_pose(self):
        t = self.translation_field.getSFVec3f()     # (x,y,z) in Webots world
        r = self.rotation_field.getSFRotation()     # (ax,ay,az,angle)
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


class RobotSupervisor:
    def __init__(self):
        self.supervisor = Supervisor()
        self.timestep = int(self.supervisor.getBasicTimeStep())

        rclpy.init(args=None)
        self.node = rclpy.create_node("robot_supervisor")
        self.log = self.node.get_logger()

        # world_supervisor와 frame 통일
        self.frame_id_world = "world" # "webots_world"

        # 취급한 DEF 정의
        self.def_prefixes = ["Rescue_UGV", "Fire_UGV", "UAV"]

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

            def_name = wb_node.getDef()  # ✅ DEF 기반
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

    def run(self):
        while self.supervisor.step(self.timestep) != -1 and rclpy.ok():
            for robot in self.tracked:
                robot.publish_world_pose()

            rclpy.spin_once(self.node, timeout_sec=0.0)

        self.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    RobotSupervisor().run()
