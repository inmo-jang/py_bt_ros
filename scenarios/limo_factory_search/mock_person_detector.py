"""Explicit stand-in: detect a known dummy while approaching the office, not YOLO."""
import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener, TransformException


def main():
    rclpy.init()
    node = Node('mock_person_detector')
    for name, default in [('person_regions', [3]), ('person_topic', '/perception/is_person'),
                          ('person_pose_topic', '/perception/person_pose'),
                          ('person_x', 5.55), ('person_y', .65), ('person_z', 0.0),
                          ('detection_distance', 1.0)]:
        node.declare_parameter(name, default)
    enabled = 3 in node.get_parameter('person_regions').value
    x,y,z = [node.get_parameter('person_'+a).value for a in 'xyz']
    distance = node.get_parameter('detection_distance').value
    publisher = node.create_publisher(Bool, node.get_parameter('person_topic').value, 10)
    poses = node.create_publisher(PoseStamped, node.get_parameter('person_pose_topic').value, 10)
    tf = Buffer(node=node); listener = TransformListener(tf,node)
    def tick():
        found = False
        try:
            p=tf.lookup_transform('map','base_link',Time()).transform.translation
            found=enabled and math.hypot(p.x-x,p.y-y) <= distance
        except TransformException:
            pass
        if found:
            pose=PoseStamped();pose.header.frame_id='map';pose.header.stamp=node.get_clock().now().to_msg()
            pose.pose.position.x=x;pose.pose.position.y=y;pose.pose.position.z=z;pose.pose.orientation.w=1.
            poses.publish(pose)
        publisher.publish(Bool(data=found))
    node.create_timer(.2,tick)
    node.get_logger().info(f'MOCK ONLY: office proximity, known person coordinate ({x}, {y}, {z}); no image inference')
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__ == '__main__':main()
