import math
import random
from modules.base_bt_nodes import BTNodeList, Status, Node, Sequence, Fallback, ReactiveSequence, ReactiveFallback, Parallel, AlwaysFailure, AlwaysSuccess
from modules.base_bt_nodes import GatherLocalInfo, TeachBT, LearnBT, AgentBT, SyncAction, SyncCondition
# BT Node List
CUSTOM_ACTION_NODES = [
    'CompleteTarget',
    'CheckTarget',
    'SuppressFire',
    'MoveToTarget',
    'MoveToBase',
    'MoveToFire',
    'MoveInFormation',
    'Wait',
    'FlyToTarget',
    'FlyToBase',
    'Explore',
    'AssignTarget'
]

CUSTOM_CONDITION_NODES = [
    'IsAllTargetCompleted',
    'IsTargetChecked',
    'IsTargetNearby',
    'IsInCheckRange',
    'IsSafeAround',
    'IsInFormation',
    'IsFireSuppressed',
    'IsInSuppressRange',
    'IsTargetAssigned',
    'IsFireAssigned',
    'IsInBase',
    'CheckTargetAfterArrive',
    'EscortLeader',
    'SuppressFireAfterAssign',
    'CompleteTargetAndReturn',
    'CheckTargetAndExplore',
    'IsTargetCompleted',
    'IsLeaderSafe'
]

# BT Node List
BTNodeList.ACTION_NODES.extend(CUSTOM_ACTION_NODES)
BTNodeList.CONDITION_NODES.extend(CUSTOM_CONDITION_NODES)

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from std_srvs.srv import SetBool
from std_srvs.srv import Empty
from modules.base_bt_nodes_ros import ConditionWithROSTopics, ActionWithROSAction, ActionWithROSService
from modules.ros_bridge import ROSBridge
from rclpy.action import ActionClient

from std_msgs.msg import UInt16MultiArray, Float64, UInt8
from typing import Dict, Optional, Tuple, List, Set

# Explicitly import AssignTarget to ensure it is recognized as an attribute of this module
from scenarios.example_webots_fire_rescue.bt_assign import AssignTarget
from modules.utils import config
from .formation import *


# Ensure AssignTarget is added to BTNodeList.ACTION_NODES
BTNodeList.ACTION_NODES.append('AssignTarget')



# =============================================
# ============== Condition Nodes ==============
# =============================================
class CompleteTargetAndReturn(AlwaysFailure):
    def __init__(self, name, agent):
        super().__init__(name, agent)

class CheckTargetAndExplore(AlwaysFailure):
    def __init__(self, name, agent):
        super().__init__(name, agent)

class CheckTargetAfterArrive(AlwaysFailure):
    def __init__(self, name, agent):
        super().__init__(name, agent)

class EscortLeader(AlwaysFailure):
    def __init__(self, name, agent):
        super().__init__(name, agent)

class SuppressFireAfterAssign(AlwaysFailure):
    def __init__(self, name, agent):
        super().__init__(name, agent)


class IsAllTargetCompleted(ConditionWithROSTopics):
    def __init__(self, name, agent, params=None):
        super().__init__(name, agent, [(UInt16MultiArray, "/world/target/summary", "target_summary")])

    def _predicate(self, agent, blackboard):
        msg = self._cache.get("target_summary", None)
        if msg is None or len(msg.data) < 4:
            return False
        
        total_targets = msg.data[0]
        unchecked_targets = msg.data[1]
        checked_targets = msg.data[2]
        completed_targets = msg.data[3]

        if total_targets == 0: # completed_targets:
            return True
        return False

class IsTargetChecked(ConditionWithROSTopics):
    def __init__(self, name, agent, params=None):
        super().__init__(name, agent, msg_types_topics=[])
        self._cache["ready"] = True
        self._subs = {}
        self._target_status = {}

    def _ensure_sub(self, target_id: str):
        if target_id in self._subs:
            return
        topic = f"/world/target/{target_id}/status"
        self._subs[target_id] = self.ros.node.create_subscription(
            UInt8, topic, lambda msg, tid=target_id: self._target_status.__setitem__(
                tid, int(msg.data)), 10)

    def _predicate(self, agent, blackboard):
        target_id = blackboard.get("assigned_target")
        if not isinstance(target_id, str) or not target_id.startswith("Target_"):
            return False

        self._ensure_sub(target_id)
        status = self._target_status.get(target_id, None)
        if status == 1:
            return True
        return False

        # if status is None:
        #     return False
        # return status == 1  # status: 0=unchecked, 1=checked, 2=completed


class IsTargetCompleted(ConditionWithROSTopics):
    def __init__(self, name, agent, params=None):
        super().__init__(name, agent, msg_types_topics=[])
        self._cache["ready"] = True
        self._subs = {}
        self._target_status = {}

    def _ensure_sub(self, target_id: str):
        if target_id in self._subs:
            return
        topic = f"/world/target/{target_id}/status"
        self._subs[target_id] = self.ros.node.create_subscription(
            UInt8, topic, lambda msg, tid=target_id: self._target_status.__setitem__(
                tid, int(msg.data)), 10)

    def _predicate(self, agent, blackboard):
        target_id = blackboard.get("assigned_target")
        if not isinstance(target_id, str) or not target_id.startswith("Target_"):
            return False

        self._ensure_sub(target_id)
        status = self._target_status.get(target_id, None)
        if status == 2:
            return True
        return False


class IsTargetNearby(ConditionWithROSTopics):
    def __init__(self, name, agent, default_thresh=1.5):
        ns = agent.ros_namespace or ""
        super().__init__(name, agent, [(PoseStamped, f"{ns}/pose_world", "self")])
        self.default_thresh = default_thresh
        self._target_xy = {}
        self._subs = {}

    def _predicate(self, agent, blackboard):
        a = self._cache.get("self")
        target_id = blackboard.get("assigned_target")
        if a is None or not isinstance(target_id, str) or not target_id.startswith("Target_"):
            return False

        if target_id not in self._subs:
            topic = f"/world/target/{target_id}/pose"
            self._subs[target_id] = self.ros.node.create_subscription(
                PoseStamped, topic,
                lambda msg, _tid=target_id: self._target_xy.__setitem__(
                    _tid, (msg.pose.position.x, msg.pose.position.y)), 10)

        t = self._target_xy.get(target_id)
        if t is None:
            return False

        ax = a.pose.position.x
        ay = a.pose.position.y
        tx, ty = t

        dist = math.hypot(ax - tx, ay - ty)
        # blackboard["distance_to_target"] = dist
        # blackboard["assigned_target_xy"] = t
        return dist <= self.default_thresh


class IsInCheckRange(ConditionWithROSTopics):
    def __init__(self, name, agent, default_thresh=1.0):
        ns = agent.ros_namespace or ""
        super().__init__(name, agent, [(PoseStamped, f"{ns}/pose_world", "self")])
        self.default_thresh = default_thresh
        self._target_xy = {}
        self._subs = {}

    def _predicate(self, agent, blackboard):
        a = self._cache.get("self")
        target_id = blackboard.get("assigned_target")
        if a is None or not isinstance(target_id, str) or not target_id.startswith("Target_"):
            return False

        if target_id not in self._subs:
            topic = f"/world/target/{target_id}/pose"
            self._subs[target_id] = self.ros.node.create_subscription(
                PoseStamped, topic,
                lambda msg, _tid=target_id: self._target_xy.__setitem__(
                    _tid, (msg.pose.position.x, msg.pose.position.y)), 10)

        t = self._target_xy.get(target_id)
        if t is None:
            return False

        ax = a.pose.position.x
        ay = a.pose.position.y
        tx, ty = t

        dist = math.hypot(ax - tx, ay - ty)
        # blackboard["distance_to_target"] = dist
        # blackboard["assigned_target_xy"] = t
        return dist <= self.default_thresh


class IsInBase(ConditionWithROSTopics):
    def __init__(self, name, agent, default_thresh=0.5):
        ns = agent.ros_namespace or ""
        super().__init__(name, agent, [(PoseStamped, f"{ns}/pose_world", "self")])
        self.default_thresh = default_thresh
        self._base_pose = None
        
        # Base pose subscription
        self.ros.node.create_subscription(
            PoseStamped, "/world/base/pose", 
            self._on_base_pose, 10
        )

    def _on_base_pose(self, msg: PoseStamped):
        self._base_pose = msg

    def _predicate(self, agent, blackboard):
        a = self._cache.get("self")
        b = self._base_pose
        if a is None or b is None:
            return False

        ax = a.pose.position.x
        ay = a.pose.position.y
        bx = b.pose.position.x
        by = b.pose.position.y

        dist = math.hypot(ax - bx, ay - by)
        return dist <= self.default_thresh

class IsLeaderSafe(ConditionWithROSTopics):
    """
    Leader(Rescue_UGV_1) 주변에 Fire가 있는지 확인하는 노드.
    Leader가 화재 근처에 있다면 Unsafe(False) 반환.
    
    [개선 사항]
    모든 Fire_UGV가 단순히 Leader 주변 Fire를 감지하고 동시에 반응하는 문제를 해결하기 위해,
    해당 Fire에 대해 '자신이 가장 가까운지'를 판단하는 로직을 추가함.
    다른 Fire_UGV가 더 가까이 있다면, 자신은 이를 무시(Safe)하고 본래 임무(Escort 등)를 수행.
    """
    def __init__(self, name, agent, safe_radius=5.5, max_fire_id=100):
        # Leader namespace 가져오기
        leader_ns = config['formation']['leader_namespace']
        ns = agent.ros_namespace or ""
        
        # Leader와 Self 모두 구독
        super().__init__(name, agent, [
            (PoseStamped, f"{leader_ns}/pose_world", "leader"),
            (PoseStamped, f"{ns}/pose_world", "self")
        ])
        
        self.safe_radius = safe_radius
        self.max_fire_id = max_fire_id
        self._fire_xy = {}
        self._subs = {}
        
        # Peer(다른 Fire_UGV) 위치 구독을 위한 설정
        self._peer_poses = {}
        self._peer_subs = {}
        self.my_name = ns.strip("/")
        self.peers = []
        
        # Config에서 다른 Fire_UGV 목록 추출
        if 'formation' in config and 'offsets' in config['formation']:
            for member in config['formation']['offsets'].keys():
                if "Fire_UGV" in member and member != self.my_name:
                    self.peers.append(member)

    def _sub_fire_pose(self, fire_id: str):
        if fire_id in self._subs:
            return
        topic = f"/world/fire/{fire_id}/pose"
        self._subs[fire_id] = self.ros.node.create_subscription(
            PoseStamped, topic,
            lambda msg, fid=fire_id: self._fire_xy.__setitem__(
                fid, (float(msg.pose.position.x), float(msg.pose.position.y))), 10)
    
    def _sub_peer_pose(self, peer_name: str):
        if peer_name in self._peer_subs:
            return
        topic = f"/{peer_name}/pose_world"
        self._peer_subs[peer_name] = self.ros.node.create_subscription(
            PoseStamped, topic,
            lambda msg, pn=peer_name: self._peer_poses.__setitem__(pn, msg), 
            10
        )
    
    def _fire_exists(self, fire_id: str) -> bool:
        topic = f"/world/fire/{fire_id}/pose"
        pubs = self.ros.node.get_publishers_info_by_topic(topic)
        return len(pubs) > 0

    def _predicate(self, agent, blackboard):
        if "leader" not in self._cache:
            return False 
        
        leader_pose = self._cache["leader"]
        lx = leader_pose.pose.position.x
        ly = leader_pose.pose.position.y

        # 내 위치 가져오기
        mx, my = None, None
        if "self" in self._cache:
            my_pose = self._cache["self"]
            mx = my_pose.pose.position.x
            my = my_pose.pose.position.y

        nearest = None
        nearest_fire_id = None
        
        for i in range(1, self.max_fire_id + 1):
            fid = f"Fire_{i}"
            if not self._fire_exists(fid):
                continue

            self._sub_fire_pose(fid)
            fxy = self._fire_xy.get(fid)
            if fxy is None:
                continue

            fx, fy = fxy
            dist = math.hypot(lx - fx, ly - fy)
            
            if nearest is None or dist < nearest:
                nearest = dist
                nearest_fire_id = fid

            # Leader 안전 반경 내에 Fire 발견
            if dist <= self.safe_radius:
                # 내가 가장 가까운지 확인 (Task Allocation)
                if mx is not None:
                    my_dist = math.hypot(mx - fx, my - fy)
                    am_closest = True
                    
                    for peer in self.peers:
                        self._sub_peer_pose(peer)
                        p_pose = self._peer_poses.get(peer)
                        if p_pose:
                            px = p_pose.pose.position.x
                            py = p_pose.pose.position.y
                            p_dist = math.hypot(px - fx, py - fy)
                            
                            # 다른 친구가 더 가깝다면 나는 무시
                            if p_dist < my_dist:
                                am_closest = False
                                break
                    
                    if not am_closest:
                        # 내가 가장 가깝지 않으므로 이 위협은 다른 agent에게 맡김
                        continue

                # 내가 가장 가깝거나(또는 내 위치 모름), 맡아야 할 상황
                blackboard["nearest_fire"] = fid
                blackboard["nearest_fire_dist"] = float(dist)
                return False  # Unsafe -> Leader is near fire
            
        blackboard["nearest_fire"] = nearest_fire_id
        blackboard["nearest_fire_dist"] = float(nearest) if nearest is not None else None
        return True

class IsSafeAround(ConditionWithROSTopics):
    """
    주변에 Fire가 있는지 확인하는 노드인데, fire 관련 메세지가 /world/fire/fire_id 이런 형식이라
    모든 fire_id를 확인하기 어려움. 따라서 max_fire_id 개수를 높게 설정해두었음.
    """
    def __init__(self, name, agent, safe_radius=2.0, max_fire_id=100):
        ns = agent.ros_namespace or ""
        super().__init__(name, agent, [(PoseStamped, f"{ns}/pose_world", "self")])
        self.safe_radius = safe_radius
        self.max_fire_id = max_fire_id
        self._fire_xy = {}
        self._subs = {}

    def _sub_fire_pose(self, fire_id: str):
        if fire_id in self._subs:
            return
        topic = f"/world/fire/{fire_id}/pose"
        self._subs[fire_id] = self.ros.node.create_subscription(
            PoseStamped, topic,
            lambda msg, fid=fire_id: self._fire_xy.__setitem__(
                fid, (float(msg.pose.position.x), float(msg.pose.position.y))), 10)
    
    def _fire_exists(self, fire_id: str) -> bool:
        # publisher가 0이면 supervisor에서 제거된 상태(=suppressed)
        topic = f"/world/fire/{fire_id}/pose"
        pubs = self.ros.node.get_publishers_info_by_topic(topic)
        return len(pubs) > 0

    def _predicate(self, agent, blackboard):
        cache = self._cache
        if "self" not in cache:
            return False
        a = cache["self"]
        ax = a.pose.position.x
        ay = a.pose.position.y

        nearest = None
        nearest_fire_id = None
        for i in range(1, self.max_fire_id + 1):
            fid = f"Fire_{i}"
            if not self._fire_exists(fid):
                continue

            self._sub_fire_pose(fid)
            fxy = self._fire_xy.get(fid)
            if fxy is None:
                continue

            fx, fy = fxy
            dist = math.hypot(ax - fx, ay - fy)
            if nearest is None or dist < nearest:
                nearest = dist
                nearest_fire_id = fid

            if dist <= self.safe_radius:
                blackboard["nearest_fire"] = nearest_fire_id
                blackboard["nearest_fire_dist"] = float(dist)
                return False  # unsafe
            
        # 주변 safe_radius 안에 fire 없음
        blackboard["nearest_fire"] = nearest_fire_id
        blackboard["nearest_fire_dist"] = float(nearest) if nearest is not None else None
        return True


class IsFireSuppressed(ConditionWithROSTopics):
    def __init__(self, name, agent, params=None):
        super().__init__(name, agent, msg_types_topics=[])
        self._cache["ready"] = True
        self._seen_pub = set()  # publisher가 한 번이라도 있었던 fire_id 기록

    def _predicate(self, agent, blackboard) -> bool:
        fire_id = blackboard.get("assigned_target")
        if not isinstance(fire_id, str) or not fire_id.startswith("Fire_"):
            return False

        topic = f"/world/fire/{fire_id}/pose"
        pubs = self.ros.node.get_publishers_info_by_topic(topic)
        has_pub = (len(pubs) > 0)

        if has_pub:
            self._seen_pub.add(fire_id)
            return False

        # publisher가 0개일 때:
        # "한 번이라도 publisher가 있었던 fire"만 suppressed로 인정 (초기 오판 방지)
        return (fire_id in self._seen_pub)
    

class IsInSuppressRange(ConditionWithROSTopics):
    def __init__(self, name, agent, default_thresh=2.0):
        ns = agent.ros_namespace or ""
        super().__init__(name, agent, [(PoseStamped, f"{ns}/pose_world", "self")])
        self.default_thresh = default_thresh
        self._fire_xy = {}
        self._fire_rad = {}
        self._subs_pose = {}
        self._subs_rad = {}

    def _sub_fire_pose(self, fire_id: str):
        if fire_id in self._subs_pose:
            return
        topic = f"/world/fire/{fire_id}/pose"
        self._subs_pose[fire_id] = self.ros.node.create_subscription(
            PoseStamped, topic,
            lambda msg, fid=fire_id: self._fire_xy.__setitem__(
                fid, (float(msg.pose.position.x), float(msg.pose.position.y))
            ),
            10
        )

    def _sub_fire_radius(self, fire_id: str):
        if fire_id in self._subs_rad:
            return
        topic = f"/world/fire/{fire_id}/radius"
        self._subs_rad[fire_id] = self.ros.node.create_subscription(
            Float64, topic,
            lambda msg, fid=fire_id: self._fire_rad.__setitem__(fid, float(msg.data)),
            10
        )

    def _predicate(self, agent, blackboard):
        cache = self._cache
        fire_id = blackboard.get("assigned_target")
        
        if "self" not in cache or not isinstance(fire_id, str) or not fire_id.startswith("Fire_"):
            return False

        self._sub_fire_pose(fire_id)
        self._sub_fire_radius(fire_id)

        a = cache["self"]
        fire_xy = self._fire_xy.get(fire_id)
        fire_rad = self._fire_rad.get(fire_id)
        if fire_xy is None or fire_rad is None:
            return False
        
        ax = a.pose.position.x
        ay = a.pose.position.y
        fx, fy = fire_xy
        dist = math.hypot(ax - fx, ay - fy)
        blackboard["distance_to_fire"] = dist
        blackboard["fire_radius"] = fire_rad
        return dist <= self.default_thresh


class IsInFormation(ConditionWithROSTopics):
    def __init__(self, name, agent, params=None):
        ns = agent.ros_namespace or ""
        leader_ns = config['formation']['leader_namespace']
        super().__init__(name, agent, [
            (PoseStamped, f"{ns}/pose_world", "self"),
            (PoseStamped, f"{leader_ns}/pose_world", "leader")
        ])
        self.tolerance = config['formation']['tolerance_xy']
        agent_name = ns.lstrip("/")
        self.offset_xy = tuple(config['formation']['offsets'].get(agent_name, [0.0, 0.0]))

    def _predicate(self, agent, blackboard):
        self_pose = self._cache.get("self")
        leader_pose = self._cache.get("leader")
        
        if self_pose is None or leader_pose is None:
            return False
        
        # pose → (x, y, yaw) 변환
        follower_xyyaw = pose_to_xyyaw(self_pose)
        leader_xyyaw = pose_to_xyyaw(leader_pose)
        
        # 리더 기준 목표 위치 계산
        target_xy = compute_target_xy(leader_xyyaw, self.offset_xy)
        
        # 포메이션 내에 있는지 판정
        return in_formation((follower_xyyaw[0], follower_xyyaw[1]), target_xy, self.tolerance)


class IsFireAssigned(SyncCondition):
    def __init__(self, name, agent):
        super().__init__(name, self._check)
        self.agent = agent

    def _check(self, agent, blackboard):
        fire_id = blackboard.get("assigned_target")
        if isinstance(fire_id, str) and fire_id.startswith("Fire_"):
            return Status.SUCCESS
        return Status.FAILURE


class IsTargetAssigned(SyncCondition):
    def __init__(self, name, agent, params=None):
        super().__init__(name, self._check)
        self._params = params

    def _check(self, agent, blackboard):
        target_id = blackboard.get("assigned_target")
        if isinstance(target_id, str) and target_id.startswith("Target_"):
            return Status.SUCCESS
        return Status.FAILURE


# =============================================
# =============== Action Nodes ================
# =============================================
class CompleteTarget(ActionWithROSService):
    def __init__(self, name, agent):
        super().__init__(name, agent, (Empty, "/world/target/Target_1/complete"))
        self._clients = {}
        self._sent_for = None

    def _build_request(self, agent, blackboard):
        return Empty.Request()
    
    async def run(self, agent, blackboard):
        target_id = blackboard.get("assigned_target")
        if not isinstance(target_id, str) or not target_id.startswith("Target_"):
            self.status = Status.FAILURE
            return self.status
        
        if target_id != self._sent_for:  # target이 바뀌었을 때 적용
            self._sent = False
            self._future = None
            self._sent_for = target_id

        if target_id not in self._clients:
            srv = f"/world/target/{target_id}/complete"
            self._clients[target_id] = self.ros.node.create_client(Empty, srv)

        self.client = self._clients[target_id]
        return await super().run(agent, blackboard)


class CheckTarget(ActionWithROSService):
    def __init__(self, name, agent):
        super().__init__(name, agent, (Empty, "/world/target/Target_1/check"))
        self._clients = {}
        self._sent_for = None

    def _build_request(self, agent, blackboard):
        return Empty.Request()
    
    async def run(self, agent, blackboard):
        target_id = blackboard.get("assigned_target")
        if not isinstance(target_id, str) or not target_id.startswith("Target_"):
            self.status = Status.FAILURE
            return self.status
        
        if target_id != self._sent_for:  # target이 바뀌었을 때 적용
            self._sent = False
            self._future = None
            self._sent_for = target_id

        if target_id not in self._clients:
            srv = f"/world/target/{target_id}/check"
            self._clients[target_id] = self.ros.node.create_client(Empty, srv)

        self.client = self._clients[target_id]
        return await super().run(agent, blackboard)


class SuppressFire(ActionWithROSService):
    def __init__(self, name, agent):
        super().__init__(name, agent, (Empty, "/world/fire/Fire_1/suppress"))
        self._clients = {}
        self._sent_for = None

    def _build_request(self, agent, blackboard):
        return Empty.Request()
    
    async def run(self, agent, blackboard):
        fire_id = blackboard.get("assigned_target")
        if not isinstance(fire_id, str) or not fire_id.startswith("Fire_"):
            self.status = Status.FAILURE
            return self.status
        
        if fire_id != self._sent_for:  # fire가 바뀌었을 때 적용
            self._sent = False
            self._future = None
            self._sent_for = fire_id

        if fire_id not in self._clients:
            srv = f"/world/fire/{fire_id}/suppress"
            self._clients[fire_id] = self.ros.node.create_client(Empty, srv)

        self.client = self._clients[fire_id]
        return await super().run(agent, blackboard)


class MoveToTarget(ActionWithROSAction):
    def __init__(self, name, agent):
        ns = agent.ros_namespace or ""  # 예: "/Rescue_UGV_1"
        super().__init__(name, agent, (NavigateToPose, f"{ns}/navigate_to_pose"))

        goal_topic = f"{ns}/goal_pose" if ns else "/goal_pose"
        self.goal_pub = self.ros.node.create_publisher(PoseStamped, goal_topic, 10)

        self._target_xy: Dict[str, Tuple[float, float]] = {}
        self._subscribed: set[str] = set()
        self._last_printed: Optional[str] = None
        self._last_target_id: Optional[str] = None

    def _ensure_target_sub(self, target_id: str):
        topic = f"/world/target/{target_id}/pose"
        if topic in self._subscribed:
            return

        def cb(msg: PoseStamped, tid=target_id):
            self._target_xy[tid] = (float(msg.pose.position.x), float(msg.pose.position.y))

        self.ros.node.create_subscription(PoseStamped, topic, cb, 10)
        self._subscribed.add(topic)

    def _get_xy(self, bb):
        target_id = bb.get("assigned_target")
        if not isinstance(target_id, str) or not target_id.startswith("Target_"):
            return None

        self._ensure_target_sub(target_id)
        return self._target_xy.get(target_id, None)

    def _build_goal(self, agent, bb):
        xy = self._get_xy(bb)
        if xy is None:
            return None
        x, y = xy

        ps = PoseStamped()
        ps.header.frame_id = "world"
        ps.header.stamp = self.ros.node.get_clock().now().to_msg()
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.orientation.w = 1.0

        goal = NavigateToPose.Goal()
        goal.pose = ps

        label = bb.get("assigned_target")
        if self._last_printed != label:
            print(f"[MoveToTarget] goal -> {label} ({x:.2f},{y:.2f})")
            self._last_printed = label

        return goal

    def _on_running(self, agent, bb):
        xy = self._get_xy(bb)
        if xy is None:
            return
        x, y = xy

        ps = PoseStamped()
        ps.header.frame_id = "world"
        ps.header.stamp = self.ros.node.get_clock().now().to_msg()
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.orientation.w = 1.0
        self.goal_pub.publish(ps)

    def _interpret_result(self, result, agent, bb, status_code=None):
        if status_code == GoalStatus.STATUS_SUCCEEDED:
            return Status.SUCCESS
        elif status_code == GoalStatus.STATUS_CANCELED:
            print("[MoveToTarget] canceled")
            return Status.FAILURE
        else:
            print(f"[MoveToTarget] aborted status={status_code}")
            return Status.FAILURE

    async def run(self, agent, blackboard):
        current_target_id = blackboard.get("assigned_target")

        if current_target_id != self._last_target_id:
            self._last_target_id = current_target_id
            if hasattr(self, "_goal_handle") and self._goal_handle:
                self._goal_handle.cancel_goal_async()
            self._phase = "idle"

        return await super().run(agent, blackboard)

    def halt(self):
        """BT에서 다른 노드로 전환될 때 호출됨"""
        self._last_target_id = None
        super().halt()


class MoveToBase(ActionWithROSAction):
    def __init__(self, name, agent):
        ns = agent.ros_namespace or ""
        super().__init__(name, agent,(NavigateToPose, f"{ns}/navigate_to_pose"),)
        self._base_pose = None
        self._was_active = False  # 이전에 MoveToBase가 활성화되었는지 추적
        self.ros.node.create_subscription(PoseStamped, "/world/base/pose", self._on_base_pose, 10)

    def _on_base_pose(self, msg: PoseStamped):
        self._base_pose = msg

    def _build_goal(self, agent, bb):
        if self._base_pose is None:
            return None
        goal = NavigateToPose.Goal()
        goal.pose = self._base_pose
        goal.pose.header.stamp = self.ros.node.get_clock().now().to_msg()
        goal.pose.header.frame_id = "world"
        return goal
    
    def _interpret_result(self, result, agent, blackboard, status_code=None):
        if status_code == GoalStatus.STATUS_SUCCEEDED:
            return Status.SUCCESS
        elif status_code == GoalStatus.STATUS_CANCELED:
            return Status.FAILURE
        else:
            return Status.FAILURE

    async def run(self, agent, blackboard):
        """MoveToFire에서 MoveToBase로 전환될 때 기존 goal 취소 및 새로 시작"""
        ns = getattr(agent, 'ros_namespace', 'unknown') or 'unknown'
        
        # 처음 활성화되었을 때 (MoveToFire에서 전환됨)
        if not self._was_active:
            print(f"[MoveToBase] agent={ns} activating, canceling any previous goal")
            self._was_active = True
            # 기존 goal handle이 있으면 취소
            if hasattr(self, '_goal_handle') and self._goal_handle:
                self._goal_handle.cancel_goal_async()
            self._phase = 'idle'  # 새로운 goal 전송을 위해 idle로 리셋
        
        return await super().run(agent, blackboard)
    
    def halt(self):
        """노드가 halt될 때 (다른 노드로 전환될 때) 상태 리셋"""
        self._was_active = False
        super().halt()
        

class MoveToFire(ActionWithROSAction):
    def __init__(self, name, agent):
        ns = agent.ros_namespace or ""  # 예: "/Fire_UGV_1"
        super().__init__(name, agent,(NavigateToPose, f"{ns}/navigate_to_pose"))
        goal_topic = f"{ns}/goal_pose" if ns else "/goal_pose"
        self.goal_pub = self.ros.node.create_publisher(PoseStamped, goal_topic, 10)

        self._fire_xy: Dict[str, Tuple[float, float]] = {}
        self._subscribed: set[str] = set()
        self._last_printed: Optional[str] = None
        self._last_target_id: Optional[str] = None  # 이전 target ID 저장

    def _ensure_fire_sub(self, fire_id: str):
        topic = f"/world/fire/{fire_id}/pose"
        if topic in self._subscribed:
            return

        def cb(msg: PoseStamped, fid=fire_id):
            self._fire_xy[fid] = (float(msg.pose.position.x), float(msg.pose.position.y))

        self.ros.node.create_subscription(PoseStamped, topic, cb, 10)
        self._subscribed.add(topic)

    def _get_xy(self, bb):
        fire_id = bb.get("assigned_target")
        if not isinstance(fire_id, str) or not fire_id.startswith("Fire_"):
            return None

        self._ensure_fire_sub(fire_id)
        return self._fire_xy.get(fire_id, None)

    def _build_goal(self, agent, bb):
        xy = self._get_xy(bb)
        if xy is None:
            return None
        x, y = xy

        ps = PoseStamped()
        # ⚠️ frame_id는 서버가 odom 기준이면 'odom', world 기준이면 'world'
        ps.header.frame_id = "world"
        ps.header.stamp = self.ros.node.get_clock().now().to_msg()
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.orientation.w = 1.0

        goal = NavigateToPose.Goal()
        goal.pose = ps

        # 목표 변경 시에만 로그
        label = bb.get("assigned_target")
        if self._last_printed != label:
            print(f"[MoveToFire] goal -> {label} ({x:.2f},{y:.2f})")
            self._last_printed = label

        return goal

    def _on_running(self, agent, bb):
        xy = self._get_xy(bb)
        if xy is None:
            return
        x, y = xy

        ps = PoseStamped()
        ps.header.frame_id = "world"
        ps.header.stamp = self.ros.node.get_clock().now().to_msg()
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.orientation.w = 1.0
        self.goal_pub.publish(ps)

    def _interpret_result(self, result, agent, bb, status_code=None):
        if status_code == GoalStatus.STATUS_SUCCEEDED:
            return Status.SUCCESS
        elif status_code == GoalStatus.STATUS_CANCELED:
            print("[MoveToFire] canceled")
            return Status.FAILURE
        else:
            print(f"[MoveToFire] aborted status={status_code}")
            return Status.FAILURE

    async def run(self, agent, blackboard):
        """매 틱마다 target 변경을 확인하고 필요시 새로운 goal 생성"""
        current_target_id = blackboard.get("assigned_target")

        # target이 바뀌었거나 처음 실행이면 새로운 goal 생성
        if current_target_id != self._last_target_id:
            self._last_target_id = current_target_id
            # 기존 액션 취소 및 재시작
            if self._goal_handle:
                self._goal_handle.cancel_goal_async()
            self._phase = 'idle'

        # 새로운 Fire 토픽 데이터가 아직 도착하지 않은 경우 대기
        if self._phase == 'idle':
            xy = self._get_xy(blackboard)
            if xy is None:
                self.status = Status.RUNNING
                return self.status

        # 기본 run 메서드 호출
        return await super().run(agent, blackboard)

    def halt(self):
        """BT에서 다른 노드로 전환될 때 호출됨"""
        self._last_target_id = None
        super().halt()

class MoveInFormation(ActionWithROSAction):
    """
    리더 기준 포메이션 위치로 이동하는 액션 노드.
    - Nav2 NavigateToPose 사용 (장애물 회피)
    - 리더가 움직이면 goal 갱신
    """
    def __init__(self, name, agent):
        ns = agent.ros_namespace or ""
        super().__init__(name, agent, (NavigateToPose, f"{ns}/navigate_to_pose"))

        # goal_pose 퍼블리셔 (rviz 시각화용)
        goal_topic = f"{ns}/goal_pose" if ns else "/goal_pose"
        self.goal_pub = self.ros.node.create_publisher(PoseStamped, goal_topic, 10)

        # config에서 formation 설정
        leader_ns = config['formation']['leader_namespace']
        self.tolerance = config['formation']['tolerance_xy']
        agent_name = ns.lstrip("/")
        self.offset_xy = tuple(config['formation']['offsets'].get(agent_name, [0.0, 0.0]))

        # 리더 pose 캐시
        self._leader_pose = None
        self.ros.node.create_subscription(
            PoseStamped, f"{leader_ns}/pose_world", self._leader_cb, 10)

        # 이전 goal 위치 (변화 감지용)
        self._last_goal_xy: Optional[Tuple[float, float]] = None
        self._goal_change_thresh = 1.5 # 1.0  # goal이 1m 이상 변하면 replan

    def _leader_cb(self, msg):
        self._leader_pose = msg

    def _get_target_xy(self) -> Optional[Tuple[float, float]]:
        if self._leader_pose is None:
            return None
        leader_xyyaw = pose_to_xyyaw(self._leader_pose)
        return compute_target_xy(leader_xyyaw, self.offset_xy)

    def _build_goal(self, agent, bb):
        target_xy = self._get_target_xy()
        if target_xy is None:
            return None
        x, y = target_xy

        ps = PoseStamped()
        ps.header.frame_id = "world"
        ps.header.stamp = self.ros.node.get_clock().now().to_msg()
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.orientation.w = 1.0

        goal = NavigateToPose.Goal()
        goal.pose = ps

        print(f"[MoveInFormation] goal -> ({x:.2f},{y:.2f})")
        self._last_goal_xy = target_xy
        return goal

    def _on_running(self, agent, bb):
        target_xy = self._get_target_xy()
        if target_xy is None:
            return
        x, y = target_xy

        # rviz 시각화용 goal_pose 퍼블리시
        ps = PoseStamped()
        ps.header.frame_id = "world"
        ps.header.stamp = self.ros.node.get_clock().now().to_msg()
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.orientation.w = 1.0
        self.goal_pub.publish(ps)

    def _interpret_result(self, result, agent, bb, status_code=None):
        if status_code == GoalStatus.STATUS_SUCCEEDED:
            return Status.SUCCESS
        elif status_code == GoalStatus.STATUS_CANCELED:
            print("[MoveInFormation] canceled")
            return Status.FAILURE
        else:
            print(f"[MoveInFormation] aborted status={status_code}")
            return Status.FAILURE

    async def run(self, agent, blackboard):
        # 리더가 많이 움직였으면 goal 갱신 (replan)
        current_xy = self._get_target_xy()
        if current_xy is not None and self._last_goal_xy is not None:
            dist = math.hypot(
                current_xy[0] - self._last_goal_xy[0],
                current_xy[1] - self._last_goal_xy[1]
            )
            if dist > self._goal_change_thresh:
                # goal이 많이 바뀌었으면 action cancel 후 재시작
                if hasattr(self, '_goal_handle') and self._goal_handle:
                    self._goal_handle.cancel_goal_async()
                self._phase = 'idle'

        return await super().run(agent, blackboard)


class Wait(Node):
    """
    정지 액션 노드
    - UGV (Husky): TwistStamped 타입
    - UAV (Mavic): Twist 타입
    """
    def __init__(self, name, agent):
        super().__init__(name)
        self.type = "Action"
        self.ros = ROSBridge.get()
        self.ns = agent.ros_namespace or ""
        self.agent = agent

        self._is_ugv = "UGV" in self.ns
        topic = f"{self.ns}/cmd_vel"

        if self._is_ugv:
            self.pub = self.ros.node.create_publisher(TwistStamped, topic, 10)
        else:
            self.pub = self.ros.node.create_publisher(Twist, topic, 10)

    async def run(self, agent, blackboard):
        if self._is_ugv:
            msg = TwistStamped()
            msg.header.stamp = self.ros.node.get_clock().now().to_msg()
            self.pub.publish(msg)
        else:
            msg = Twist()
            self.pub.publish(msg)

        self.status = Status.RUNNING
        return self.status
            
    def halt(self):
        try:
            if self._is_ugv:
                msg = TwistStamped()
                msg.header.stamp = self.ros.node.get_clock().now().to_msg()
                self.pub.publish(msg)
            else:
                msg = Twist()
                self.pub.publish(msg)
        except Exception:
            pass
        self.status = None

def _yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def _world_to_body(vx_w, vy_w, yaw):
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    vx_b =  cy * vx_w + sy * vy_w
    vy_b = -sy * vx_w + cy * vy_w
    return vx_b, vy_b

def _slew(prev, target, dv):
    return max(prev - dv, min(prev + dv, target))

class FlyToTarget(Node):
    def __init__(self, name, agent, default_thresh=0.5):
        super().__init__(name)
        self.type = "Action"
        self.ros = ROSBridge.get()
        self.ns = agent.ros_namespace or ""
        self.default_thresh = float(default_thresh)

        self._self_pose = None
        self._target_xy = {}
        self._subs = {}

        self.ros.node.create_subscription(PoseStamped, f"{self.ns}/pose_world", self._self_cb, 10)
        self.cmd_pub = self.ros.node.create_publisher(Twist, f"{self.ns}/cmd_vel", 10)

        # NEW: 부드러운 가속 제한용
        self._last_vx = 0.0
        self._last_vy = 0.0

    def _self_cb(self, msg: PoseStamped):
        self._self_pose = msg

    def _publish_zero(self):
        self.cmd_pub.publish(Twist())
        self._last_vx = 0.0
        self._last_vy = 0.0

    def _ensure_target_sub(self, target_id: str):
        if target_id in self._subs:
            return
        topic = f"/world/target/{target_id}/pose"
        self._subs[target_id] = self.ros.node.create_subscription(
            PoseStamped,
            topic,
            lambda msg, tid=target_id: self._target_xy.__setitem__(
                tid, (float(msg.pose.position.x), float(msg.pose.position.y))), 10)

    async def run(self, agent, blackboard):
        target_id = blackboard.get("assigned_target")
        if not isinstance(target_id, str) or not target_id.startswith("Target_"):
            self._publish_zero()
            self.status = Status.FAILURE
            return self.status

        if self._self_pose is None:
            self._publish_zero()
            self.status = Status.RUNNING
            return self.status

        self._ensure_target_sub(target_id)
        t = self._target_xy.get(target_id)
        if t is None:
            self._publish_zero()
            self.status = Status.RUNNING
            return self.status

        ax = float(self._self_pose.pose.position.x)
        ay = float(self._self_pose.pose.position.y)
        tx, ty = t

        dx = tx - ax
        dy = ty - ay
        dist = math.hypot(dx, dy)

        blackboard["distance_to_target"] = dist
        blackboard["assigned_target_xy"] = (tx, ty)

        thresh = float(blackboard.get("nearby_target_threshold", self.default_thresh))
        if dist <= thresh:
            self._publish_zero()
            self.status = Status.SUCCESS
            return self.status

        # 속도 프로파일 (NEW: min_speed 추가 + 감속 반경 확장)
        max_speed   = float(blackboard.get("uav_max_speed", 1.0))  # 1.5
        min_speed   = float(blackboard.get("uav_min_speed", 0.1))  # 0.2
        slow_radius = float(blackboard.get("target_slow_radius", 5.0))  # 3.0

        ratio = min(1.0, dist / max(slow_radius, 1e-6))
        speed = min_speed + (max_speed - min_speed) * ratio

        if dist > 1e-6:
            ux, uy = dx / dist, dy / dist
        else:
            ux, uy = 0.0, 0.0

        # world 속도 (목표방향)
        vx_w = ux * speed
        vy_w = uy * speed

        # NEW: yaw로 world->body 변환
        yaw = _yaw_from_quat(self._self_pose.pose.orientation)
        vx, vy = _world_to_body(vx_w, vy_w, yaw)

        # 혹시 드라이버가 y를 "right"로 쓰면 여기만 반전 옵션
        if bool(blackboard.get("uav_y_is_right", False)):
            vy = -vy

        # NEW: 가속 제한(기동 튐 줄이기)
        max_acc = float(blackboard.get("uav_max_acc", 1.5))  # m/s^2
        dt = float(blackboard.get("bt_dt", 0.1))            # tick 간격(없으면 0.1 가정)
        dv = max_acc * max(dt, 1e-3)

        vx = max(self._last_vx - dv, min(self._last_vx + dv, vx))
        vy = max(self._last_vy - dv, min(self._last_vy + dv, vy))
        self._last_vx, self._last_vy = vx, vy

        cmd = Twist()
        cmd.linear.x = float(vx)
        cmd.linear.y = float(vy)
        cmd.linear.z = 0.0  # 고도는 여기서는 건드리지 않음(유지/별도 제어)
        self.cmd_pub.publish(cmd)

        self.status = Status.RUNNING
        return self.status

    def halt(self):
        try:
            self._publish_zero()
        except Exception:
            pass
        self.status = None

class FlyToBase(Node):
    """
    UAV를 Base로 이동시키는 액션 노드
    - pose_world: /<ns>/pose_world 구독
    - base pose: /world/base/pose (기본) 또는 blackboard['base_xy']
    - 월드→바디 변환 적용
    """
    def __init__(self, name, agent, default_thresh=5.0):
        super().__init__(name)
        self.type = "Action"
        self.ros = ROSBridge.get()
        self.ns = agent.ros_namespace or ""
        self.default_thresh = float(default_thresh)

        self._self_pose: Optional[PoseStamped] = None
        self._base_xy: Optional[Tuple[float, float]] = None
        self._base_sub_ok = False

        self.cmd_pub = self.ros.node.create_publisher(Twist, f"{self.ns}/cmd_vel", 10)
        self.ros.node.create_subscription(PoseStamped, f"{self.ns}/pose_world", self._self_cb, 10)

        # 부드러운 가속 제한
        self._last_vx = 0.0
        self._last_vy = 0.0

    def _self_cb(self, msg: PoseStamped):
        self._self_pose = msg

    def _publish_zero(self):
        self.cmd_pub.publish(Twist())
        self._last_vx = 0.0
        self._last_vy = 0.0

    def _ensure_base_sub(self, blackboard):
        # 1) blackboard에 base_xy가 있으면 그걸 우선 사용
        bb_base = blackboard.get("base_xy")
        if isinstance(bb_base, (tuple, list)) and len(bb_base) == 2:
            self._base_xy = (float(bb_base[0]), float(bb_base[1]))
            return

        # 2) 아니면 base pose 토픽을 한 번만 구독
        if self._base_sub_ok:
            return

        base_topic = str(blackboard.get("base_pose_topic", "/world/base/pose"))
        def _on_base(msg: PoseStamped):
            self._base_xy = (float(msg.pose.position.x), float(msg.pose.position.y))

        self.ros.node.create_subscription(PoseStamped, base_topic, _on_base, 10)
        self._base_sub_ok = True

    async def run(self, agent, blackboard):
        if self._self_pose is None:
            self._publish_zero()
            self.status = Status.RUNNING
            return self.status

        self._ensure_base_sub(blackboard)
        if self._base_xy is None:
            self._publish_zero()
            self.status = Status.RUNNING
            return self.status

        ax = float(self._self_pose.pose.position.x)
        ay = float(self._self_pose.pose.position.y)
        bx, by = self._base_xy

        dx = bx - ax
        dy = by - ay
        dist = math.hypot(dx, dy)

        blackboard["distance_to_base"] = dist
        blackboard["base_xy"] = (bx, by)

        thresh = float(blackboard.get("nearby_base_threshold", self.default_thresh))
        if dist <= thresh:
            self._publish_zero()
            self.status = Status.SUCCESS
            return self.status

        # 속도 프로파일
        max_speed   = float(blackboard.get("uav_max_speed", 1.5))
        min_speed   = float(blackboard.get("uav_min_speed", 0.2))
        slow_radius = float(blackboard.get("base_slow_radius", 3.0))

        ratio = min(1.0, dist / max(slow_radius, 1e-6))
        speed = min_speed + (max_speed - min_speed) * ratio

        if dist > 1e-6:
            ux, uy = dx / dist, dy / dist
        else:
            ux, uy = 0.0, 0.0

        vx_w = ux * speed
        vy_w = uy * speed

        yaw = _yaw_from_quat(self._self_pose.pose.orientation)
        vx, vy = _world_to_body(vx_w, vy_w, yaw)

        # y축 방향이 드라이버에서 반대면 옵션으로 뒤집기
        if bool(blackboard.get("uav_y_is_right", False)):
            vy = -vy

        # 가속 제한
        max_acc = float(blackboard.get("uav_max_acc", 1.5))
        dt = float(blackboard.get("bt_dt", 0.1))
        dv = max_acc * max(dt, 1e-3)

        vx = _slew(self._last_vx, vx, dv)
        vy = _slew(self._last_vy, vy, dv)
        self._last_vx, self._last_vy = vx, vy

        cmd = Twist()
        cmd.linear.x = float(vx)
        cmd.linear.y = float(vy)
        cmd.linear.z = 0.0
        self.cmd_pub.publish(cmd)

        self.status = Status.RUNNING
        return self.status

    def halt(self):
        try:
            self._publish_zero()
        except Exception:
            pass
        self.status = None


class Explore(Node):
    """
    UAV 탐색(순찰) 노드
    - 랜덤 웨이포인트를 생성해서 계속 추종 (RUNNING 유지)
    - 도착하면 새 웨이포인트 생성
    """
    def __init__(self, name, agent, default_thresh=0.8):
        super().__init__(name)
        self.type = "Action"
        self.ros = ROSBridge.get()
        self.ns = agent.ros_namespace or ""
        self.default_thresh = float(default_thresh)

        self._self_pose: Optional[PoseStamped] = None
        self._wp: Optional[Tuple[float, float]] = None  # current waypoint (x,y)

        self.cmd_pub = self.ros.node.create_publisher(Twist, f"{self.ns}/cmd_vel", 10)
        self.ros.node.create_subscription(PoseStamped, f"{self.ns}/pose_world", self._self_cb, 10)

        self._last_vx = 0.0
        self._last_vy = 0.0

    def _self_cb(self, msg: PoseStamped):
        self._self_pose = msg

    def _publish_zero(self):
        self.cmd_pub.publish(Twist())
        self._last_vx = 0.0
        self._last_vy = 0.0

    def _pick_waypoint(self, blackboard):
        # 중심점: 설정이 있으면 사용, 없으면 현재 위치
        c = blackboard.get("explore_center_xy")
        if isinstance(c, (tuple, list)) and len(c) == 2:
            cx, cy = float(c[0]), float(c[1])
        else:
            cx = float(self._self_pose.pose.position.x)
            cy = float(self._self_pose.pose.position.y)

        r = float(blackboard.get("explore_radius", 6.0))

        # 균일 분포(원 내부)
        ang = random.uniform(-math.pi, math.pi)
        rad = r * math.sqrt(random.random())
        x = cx + rad * math.cos(ang)
        y = cy + rad * math.sin(ang)

        # 월드 범위 클램프(있으면)
        mn = blackboard.get("world_min_xy")
        mx = blackboard.get("world_max_xy")
        if isinstance(mn, (tuple, list)) and len(mn) == 2 and isinstance(mx, (tuple, list)) and len(mx) == 2:
            x = max(float(mn[0]), min(float(mx[0]), x))
            y = max(float(mn[1]), min(float(mx[1]), y))

        self._wp = (x, y)
        blackboard["explore_waypoint_xy"] = self._wp

    async def run(self, agent, blackboard):
        if self._self_pose is None:
            self._publish_zero()
            self.status = Status.RUNNING
            return self.status

        if self._wp is None:
            self._pick_waypoint(blackboard)

        wx, wy = self._wp
        ax = float(self._self_pose.pose.position.x)
        ay = float(self._self_pose.pose.position.y)

        dx = wx - ax
        dy = wy - ay
        dist = math.hypot(dx, dy)

        blackboard["distance_to_explore_wp"] = dist

        thresh = float(blackboard.get("explore_wp_threshold", self.default_thresh))
        if dist <= thresh:
            # 도착 -> 새 웨이포인트
            self._pick_waypoint(blackboard)
            self._publish_zero()
            self.status = Status.RUNNING
            return self.status

        # 속도 프로파일(탐색은 좀 더 부드럽게)
        max_speed   = float(blackboard.get("explore_max_speed", blackboard.get("uav_max_speed", 1.2)))
        min_speed   = float(blackboard.get("explore_min_speed", blackboard.get("uav_min_speed", 0.2)))
        slow_radius = float(blackboard.get("explore_slow_radius", 3.0))

        ratio = min(1.0, dist / max(slow_radius, 1e-6))
        speed = min_speed + (max_speed - min_speed) * ratio

        if dist > 1e-6:
            ux, uy = dx / dist, dy / dist
        else:
            ux, uy = 0.0, 0.0

        vx_w = ux * speed
        vy_w = uy * speed

        yaw = _yaw_from_quat(self._self_pose.pose.orientation)
        vx, vy = _world_to_body(vx_w, vy_w, yaw)

        if bool(blackboard.get("uav_y_is_right", False)):
            vy = -vy

        # 가속 제한
        max_acc = float(blackboard.get("uav_max_acc", 1.2))
        dt = float(blackboard.get("bt_dt", 0.1))
        dv = max_acc * max(dt, 1e-3)

        vx = _slew(self._last_vx, vx, dv)
        vy = _slew(self._last_vy, vy, dv)
        self._last_vx, self._last_vy = vx, vy

        cmd = Twist()
        cmd.linear.x = float(vx)
        cmd.linear.y = float(vy)
        cmd.linear.z = 0.0
        self.cmd_pub.publish(cmd)

        self.status = Status.RUNNING
        return self.status

    def halt(self):
        try:
            self._publish_zero()
        except Exception:
            pass
        self._wp = None
        self.status = None
