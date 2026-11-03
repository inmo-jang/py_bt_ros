"""Standard py_bt_ros Agent with queued LIMO missions and map-frame home capture."""
from copy import deepcopy
import json
import math
from queue import Empty, SimpleQueue
import threading
import time

from geometry_msgs.msg import PoseStamped
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import Empty as EmptyMessage, String
from tf2_ros import Buffer, TransformException, TransformListener

from modules.agent import Agent as BaseAgent
from modules.base_bt_nodes import Status
from modules.ros_bridge import ROSBridge
from modules.utils import config


class Agent(BaseAgent):
    def __init__(self, ros_namespace=None):
        self.config = config
        ROSBridge.get(namespace=ros_namespace or '', **config.get('ros', {}))
        super().__init__(ros_namespace)
        self.node = self.ros_bridge.node
        self.options = config['mission']
        self.blackboard.update(active=False, error='', mission_id=0)
        self.shutdown_futures = []
        self.pending_goals = SimpleQueue()
        self.cancel_requested = threading.Event()
        self.tf = Buffer(node=self.node)
        self.listener = TransformListener(self.tf, self.node)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.node.create_publisher(String, '/bt/mission_status', latched)
        self.home_pub = self.node.create_publisher(PoseStamped, '/bt/home_pose', latched)
        self.goal_sub = self.node.create_subscription(
            PoseStamped, self.options['goal_topic'], self.receive_goal, 10)
        self.cancel_sub = self.node.create_subscription(
            EmptyMessage, self.options['cancel_topic'], lambda _: self.cancel_requested.set(), 10)
        self.publish_status('IDLE')

    def publish_status(self, stage):
        payload = {key: self.blackboard[key] for key in ('active', 'error', 'mission_id')}
        payload.update(stage=stage, sim_time=self.node.get_clock().now().nanoseconds / 1e9)
        self.status_pub.publish(String(data=json.dumps(payload)))
        self.node.get_logger().info(f'{stage}: {self.blackboard["error"]}')

    def current_pose(self):
        now = self.node.get_clock().now()
        if self.config.get('ros', {}).get('use_sim_time') and now.nanoseconds == 0:
            raise ValueError('Waiting for simulation /clock')
        frame = self.options['robot_frame']
        transform = self.tf.lookup_transform(self.options['global_frame'], frame, Time())
        odom = self.tf.lookup_transform(self.options['odom_frame'], frame, Time())
        age = abs((now - Time.from_msg(odom.header.stamp)).nanoseconds) / 1e9
        if age > self.options['tf_max_age']:
            raise ValueError('Odometry TF is stale; check Webots and use_sim_time')
        pose = PoseStamped()
        pose.header.frame_id = self.options['global_frame']
        pose.header.stamp = now.to_msg()
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.orientation = deepcopy(transform.transform.rotation)
        return pose

    def receive_goal(self, message):
        # ROS callbacks run on the bridge thread. Only the BT thread changes the tree.
        if self.blackboard['active']:
            self.node.get_logger().warning('Goal ignored: a mission is already active')
        else:
            self.pending_goals.put(deepcopy(message))

    def start_mission(self, message):
        try:
            if message.header.frame_id != self.options['global_frame']:
                raise ValueError(f'Goal frame must be {self.options["global_frame"]}')
            goal = deepcopy(message)
            p, q = goal.pose.position, goal.pose.orientation
            if not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
                raise ValueError('Goal contains non-finite coordinates')
            norm = math.hypot(q.x, q.y, q.z, q.w)
            if norm < 1e-6:
                raise ValueError('Goal quaternion has zero length')
            q.x, q.y, q.z, q.w = q.x / norm, q.y / norm, q.z / norm, q.w / norm
            home = self.current_pose()
        except (ValueError, TransformException) as error:
            self.blackboard['error'] = str(error)
            self.publish_status('REJECTED')
            return
        self.halt_tree()
        self.tree.reset()
        self.blackboard.update(
            active=True, goal_pose=goal, home_pose=home, error='',
            mission_id=self.blackboard['mission_id'] + 1)
        self.home_pub.publish(home)
        self.publish_status('STARTING')

    async def run_tree(self):
        if self.cancel_requested.is_set():
            self.cancel_requested.clear()
            self.halt_tree()
            self.blackboard['active'] = False
            while not self.pending_goals.empty():
                self.pending_goals.get_nowait()
            self.publish_status('CANCELED')
        if not self.blackboard['active']:
            try:
                message = self.pending_goals.get_nowait()
            except Empty:
                return
            self.start_mission(message)
            # Discard double clicks instead of running a second queued mission later.
            while not self.pending_goals.empty():
                self.pending_goals.get_nowait()
        if not self.blackboard['active']:
            return
        result = await super().run_tree()
        if result in (Status.SUCCESS, Status.FAILURE):
            self.blackboard['active'] = False
            self.publish_status('COMPLETE' if result == Status.SUCCESS else 'FAILED')
        self.shutdown_futures[:] = [f for f in self.shutdown_futures if not f.done()]
        return result

    def close(self):
        self.halt_tree()
        if self.blackboard['active']:
            self.blackboard['active'] = False
            self.publish_status('CANCELED')
        # The executor still spins here, including late goal acceptance/cancellation.
        deadline = time.monotonic() + 1.0
        time.sleep(0.05)
        while time.monotonic() < deadline and any(not f.done() for f in self.shutdown_futures):
            time.sleep(0.01)
        self.ros_bridge.shutdown()
