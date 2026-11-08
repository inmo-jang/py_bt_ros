"""Gate Nav2 startup on all three robots' live sensor/odometry messages."""
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


def main():
    rclpy.init();node=Node('multi_robot_ready');seen=set();subs=[]
    for robot in ('robot1','robot2','robot3'):
        for name,typ in [('scan',LaserScan),('odom',Odometry)]:
            key=robot+'/'+name
            subs.append(node.create_subscription(typ,'/'+key,lambda msg,k=key:seen.add(k),qos_profile_sensor_data))
    end=time.monotonic()+100
    try:
        while rclpy.ok() and len(seen)<6 and time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.1)
        ok=len(seen)==6
        node.get_logger().info(f'Sensors ready: {sorted(seen)}')
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    raise SystemExit(0 if ok else 1)
