"""Connect Twist command publishers to Jazzy's stamped drive controller."""

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node


class TwistStamper(Node):
    def __init__(self):
        super().__init__('limo_twist_stamper')
        self.publisher = self.create_publisher(TwistStamped, 'cmd_vel_stamped', 10)
        self.subscription = self.create_subscription(
            Twist, 'cmd_vel', self.on_velocity, 10)

    def on_velocity(self, velocity):
        command = TwistStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = 'base_link'
        command.twist = velocity
        self.publisher.publish(command)


def main(args=None):
    rclpy.init(args=args)
    node = TwistStamper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
