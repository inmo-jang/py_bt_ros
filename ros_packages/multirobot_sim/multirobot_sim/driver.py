"""Simulation-only differential motor adapter with isolated ROS sensors and watchdog."""
import math
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, DurabilityPolicy
from .sensors import LimoSensors


class MultiLimoDriver(LimoSensors):
    def init(self, webots_node, properties):
        super().init(webots_node, properties)
        self.robot = webots_node.robot
        self.ns = properties['namespace'].strip('/')
        self.node = rclpy.create_node('motor_adapter', namespace=self.ns)
        self.motors = [self.robot.getDevice(side + '_wheel') for side in
                       ('front_left', 'rear_left', 'front_right', 'rear_right')]
        for motor in self.motors:
            motor.setPosition(float('inf'))
            motor.setVelocity(0.)
        self.command = (0., 0.)
        self.last_command = -100.
        self.stopped = True
        self.last_heartbeat = -100.
        self.stop_sub = self.node.create_subscription(Bool, '/multi_bt/emergency_stop', self.stop, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.cmd_sub = self.node.create_subscription(Twist, 'cmd_vel', self.velocity, 10)

    def stop(self, msg):
        self.stopped = msg.data
        self.last_heartbeat = self.robot.getTime()
        if self.stopped:
            self.command = (0., 0.)

    def velocity(self, msg):
        if self.stopped or not all(math.isfinite(x) for x in (msg.linear.x, msg.angular.z)):
            return
        self.command = (max(-.12, min(.22, msg.linear.x)), max(-.65, min(.65, msg.angular.z)))
        self.last_command = self.robot.getTime()

    def step(self):
        super().step()
        rclpy.spin_once(self.node, timeout_sec=0)
        v, w = self.command if not self.stopped and self.robot.getTime() - self.last_heartbeat < 1. and self.robot.getTime() - self.last_command < .5 else (0., 0.)
        left, right = (v - .086 * 2.414 * w) / .045, (v + .086 * 2.414 * w) / .045
        for motor, velocity in zip(self.motors, (left, left, right, right)):
            motor.setVelocity(velocity)
