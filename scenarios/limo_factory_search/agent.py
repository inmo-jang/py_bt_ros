"""Mission lifecycle for a Nav2-driven, three-region person search."""
from copy import deepcopy
from pathlib import Path
from datetime import datetime, timezone
from tf2_geometry_msgs import do_transform_pose_stamped
import json
import math
import threading
import time

from geometry_msgs.msg import PoseStamped
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import Bool, Empty, Int32, String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from modules.agent import Agent as BaseAgent
from modules.base_bt_nodes import Status
from modules.ros_bridge import ROSBridge
from modules.utils import config


class Agent(BaseAgent):
    def __init__(self, ros_namespace=None):
        ROSBridge.get(namespace=ros_namespace or '', **config.get('ros', {}))
        super().__init__(ros_namespace)
        self.node = self.ros_bridge.node
        self.options = config['mission']
        self.regions = {int(k): v for k, v in self.options['regions'].items()}
        self.blackboard.update(active=False, error='', mission_id=0, current_region=0,
                               target_region=0, checked_regions=[], found_region=None)
        self.shutdown_futures = []
        self.start_requested = threading.Event()
        self.cancel_requested = threading.Event()
        self.observation_lock = threading.Lock()
        self.observation = None
        self.arrived_at = None
        self.person_pose = None
        self.mission_started = 0.
        self.tf = Buffer(node=self.node)
        self.listener = TransformListener(self.tf, self.node)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.node.create_publisher(String, self.options['status_topic'], latched)
        self.found_pub = self.node.create_publisher(PoseStamped, '/bt/search/found_person', latched)
        self.region_pub = self.node.create_publisher(Int32, self.options['region_topic'], latched)
        self.marker_pub = self.node.create_publisher(MarkerArray, '/bt/search/regions', latched)
        self.subscriptions = [
            self.node.create_subscription(Empty, self.options['start_topic'], self.receive_start, 10),
            self.node.create_subscription(Empty, self.options['cancel_topic'],
                                          lambda _: self.cancel_requested.set(), 10),
            self.node.create_subscription(Bool, self.options['person_topic'], self.receive_person, 10),
            self.node.create_subscription(PoseStamped, self.options['person_pose_topic'], self.receive_person_pose, 10),
        ]
        self.region_pub.publish(Int32(data=0))
        self.publish_markers()
        self.publish_status('IDLE')

    def receive_start(self, _):
        if not self.blackboard['active']:
            self.start_requested.set()

    def receive_person(self, message):
        with self.observation_lock:
            self.observation = (self.blackboard['current_region'],
                                self.node.get_clock().now().nanoseconds / 1e9, bool(message.data))

    def region_pose(self, region):
        point = self.regions[region]
        pose = PoseStamped()
        pose.header.frame_id = self.options['global_frame']
        pose.pose.position.x = float(point['x'])
        pose.pose.position.y = float(point['y'])
        pose.pose.orientation.z = math.sin(float(point['yaw']) / 2)
        pose.pose.orientation.w = math.cos(float(point['yaw']) / 2)
        return pose

    def publish_status(self, stage):
        fields = ('active', 'error', 'mission_id', 'current_region', 'target_region',
                  'checked_regions', 'found_region')
        payload = {key: deepcopy(self.blackboard[key]) for key in fields}
        payload['person_record'] = self.blackboard.get('person_record')
        payload.update(stage=stage, sim_time=self.node.get_clock().now().nanoseconds / 1e9)
        self.status_pub.publish(String(data=json.dumps(payload)))
        self.node.get_logger().info(f'{stage}: {json.dumps(payload)}')

    def begin_region(self, region):
        with self.observation_lock:
            self.blackboard.update(current_region=0, target_region=region)
            self.arrived_at = None
        self.region_pub.publish(Int32(data=0))

    def enter_region(self, region):
        with self.observation_lock:
            self.blackboard['current_region'] = region
            self.arrived_at = self.node.get_clock().now().nanoseconds / 1e9
        self.region_pub.publish(Int32(data=region))
        self.blackboard['checked_regions'].append(region)
        self.publish_status('REGION_REACHED')

    def receive_person_pose(self, message):
        with self.observation_lock:
            self.person_pose = deepcopy(message)

    def person_observation(self):
        with self.observation_lock:
            observation = self.observation
        if observation is None:
            return None
        _, stamp, value = observation
        age = self.node.get_clock().now().nanoseconds / 1e9 - stamp
        if stamp < self.mission_started or not 0 <= age <= self.options['detection_max_age']:
            return None
        return value

    def record_person(self):
        with self.observation_lock:
            pose = deepcopy(self.person_pose)
        if pose is None or not pose.header.frame_id:
            return None
        stamp = Time.from_msg(pose.header.stamp).nanoseconds / 1e9
        now = self.node.get_clock().now().nanoseconds / 1e9
        if stamp < self.mission_started or not 0 <= now-stamp <= self.options['detection_max_age']:
            return None
        source_frame = pose.header.frame_id
        if source_frame != self.options['global_frame']:
            try:
                transform = self.tf.lookup_transform(self.options['global_frame'], source_frame,
                                                     Time.from_msg(pose.header.stamp))
                pose = do_transform_pose_stamped(pose, transform)
            except TransformException:
                return None
        p = pose.pose.position
        if not all(math.isfinite(v) for v in (p.x, p.y, p.z)):
            raise ValueError('Person position is not finite')
        record = dict(utc=datetime.now(timezone.utc).isoformat(), sim_time=stamp,
                      mission_id=self.blackboard['mission_id'], frame_id=pose.header.frame_id,
                      source_frame=source_frame, position=dict(x=p.x, y=p.y, z=p.z))
        path = Path(self.options['person_log']).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open('a') as output:
            output.write(json.dumps(record) + '\n')
        self.found_pub.publish(pose)
        return record

    def publish_markers(self):
        array = MarkerArray()
        points = {0: dict(self.options['start_pose'], label='Start'), **self.regions}
        for region, point in points.items():
            for text in (False, True):
                marker = Marker()
                marker.header.frame_id = self.options['global_frame']
                marker.ns = 'factory_search'
                marker.id = 2 * region + int(text)
                marker.type = Marker.TEXT_VIEW_FACING if text else Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x, marker.pose.position.y = float(point['x']), float(point['y'])
                marker.pose.position.z = .65 if text else .15
                marker.pose.orientation.w = 1.
                marker.scale.x = marker.scale.y = .3
                marker.scale.z = .4 if text else .3
                marker.color.a = 1.
                marker.color.r, marker.color.g, marker.color.b = ((1., 1., 1.) if text else (.1, .8, .3))
                marker.text = f'Region {region}' if region else 'Start'
                array.markers.append(marker)
        self.marker_pub.publish(array)

    def start_mission(self):
        try:
            now = self.node.get_clock().now()
            if now.nanoseconds == 0:
                raise ValueError('Waiting for simulation clock')
            self.tf.lookup_transform(self.options['global_frame'], self.options['robot_frame'], Time())
            odom = self.tf.lookup_transform(self.options['odom_frame'], self.options['robot_frame'], Time())
            if abs((now - Time.from_msg(odom.header.stamp)).nanoseconds) / 1e9 > self.options['tf_max_age']:
                raise ValueError('Odometry is stale')
        except (ValueError, TransformException) as error:
            self.blackboard['error'] = str(error)
            self.publish_status('REJECTED')
            return
        self.halt_tree()
        self.tree.reset()
        self.mission_started = now.nanoseconds / 1e9
        self.blackboard.update(active=True, error='', person_record=None, current_region=0, target_region=0,
                               checked_regions=[], found_region=None,
                               mission_id=self.blackboard['mission_id'] + 1)
        self.publish_status('SEARCHING')

    async def run_tree(self):
        if self.cancel_requested.is_set():
            self.cancel_requested.clear()
            self.start_requested.clear()
            self.halt_tree()
            self.blackboard['active'] = False
            self.begin_region(0)
            self.publish_status('CANCELED')
        if self.start_requested.is_set():
            self.start_requested.clear()
            if not self.blackboard['active']:
                self.start_mission()
        if not self.blackboard['active']:
            return
        result = await super().run_tree()
        if result in (Status.SUCCESS, Status.FAILURE):
            self.blackboard['active'] = False
            self.halt_tree()
            stage = 'FAILED' if self.blackboard['error'] else ('FOUND' if result == Status.SUCCESS else 'NOT_FOUND')
            self.region_pub.publish(Int32(data=0))
            self.publish_status(stage)
        self.shutdown_futures[:] = [f for f in self.shutdown_futures if not f.done()]
        return result

    def close(self):
        self.halt_tree()
        if self.blackboard['active']:
            self.blackboard['active'] = False
            self.publish_status('CANCELED')
        deadline = time.monotonic() + 1.
        while time.monotonic() < deadline and any(not f.done() for f in self.shutdown_futures):
            time.sleep(.01)
        self.ros_bridge.shutdown()
