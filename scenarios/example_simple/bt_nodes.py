import math
import json
import random

from modules.base_bt_nodes import BTNodeList, Status, Node, Sequence, Fallback, ReactiveSequence, ReactiveFallback, SyncAction
from modules.base_bt_nodes_ros import ActionWithROSAction, ConditionWithROSTopics
from modules.utils import config

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose

# ── BT Node registration ───────────────────────────────────────────────────────

CUSTOM_ACTION_NODES = [
    'GatherLocalInfo',
    'AssignTask',
    'MoveToTarget',
    'ExecuteTask',
    'Explore',
]

CUSTOM_CONDITION_NODES = [
    'IsTaskCompleted',
    'IsArrivedAtTarget',
]

BTNodeList.ACTION_NODES.extend(CUSTOM_ACTION_NODES)
BTNodeList.CONDITION_NODES.extend(CUSTOM_CONDITION_NODES)

# ── Config shortcuts ───────────────────────────────────────────────────────────

_agent_cfg   = config.get('agent', {})
_comm_radius = _agent_cfg.get('comm_radius', 0.0)
_map_bounds = config.get('tasks', {}).get('locations', {})



class GatherLocalInfo(ConditionWithROSTopics):
    def __init__(self, name, agent):
        ns = agent.ros_namespace or ''
        super().__init__(name, agent, [
            (PoseStamped, f"{ns}/pose_world", "ego_pose"),
            (String, 'world/fire/list', 'local_tasks_info'),
            (String, f"{ns}/local_comm/inbox", 'local_comm_inbox'),
        ])

        # outbox publisher: 자신의 상태를 robot_supervisor에 broadcast
        self._pub_outbox = agent.ros_bridge.node.create_publisher(
            String, f"{ns}/local_comm/outbox", 10
        )

    def _predicate(self, agent, blackboard):
        cache = self._cache

        # [1] Outbox broadcast: 이전 틱에서 설정한 상태를 먼저 송신
        outbox = blackboard.get('local_comm_outbox')
        if outbox is not None:
            msg = String()
            msg.data = json.dumps(outbox)
            self._pub_outbox.publish(msg)

        # [2] 필수 topic 수신 확인: 하나라도 없으면 False
        required = ["ego_pose", "local_tasks_info"]
        if any(k not in cache for k in required):
            return False

        # [3] 필수 데이터 처리
        try:
            tasks_list = json.loads(cache["local_tasks_info"].data)
        except (json.JSONDecodeError, TypeError):
            tasks_list = []
        blackboard["local_tasks_info"] = tasks_list
        blackboard["ego_position"] = (cache["ego_pose"].pose.position.x, cache["ego_pose"].pose.position.y)

        # [4] 선택적 topic: 미수신 시 빈 리스트로 폴백
        try:
            blackboard['local_comm_inbox'] = json.loads(cache["local_comm_inbox"].data)
        except (KeyError, AttributeError, json.JSONDecodeError, TypeError):
            blackboard['local_comm_inbox'] = []

        return True
    
class AssignTask(SyncAction):
    def __init__(self, name, agent):
        super().__init__(name, self._decide)
        # if decision_making_class is None:
        #     raise RuntimeError("[AssignTask] 'decision_making.plugin' is not set in config.")
        # self.decision_maker = decision_making_class(agent)

    def _decide(self, agent, blackboard):
        local_tasks_info = blackboard.get('local_tasks_info', [])
        ego_position = blackboard.get('ego_position', None)
        task_dist_list = []
        for t in local_tasks_info:
            dist = math.hypot(ego_position[0] - t['x'], ego_position[1] - t['y'])
            task_dist_list.append((t['id'], dist))

        # Assign the closest task
        if not task_dist_list:
            assigned_task_id = None
        else:
            assigned_task_id = min(task_dist_list, key=lambda x: x[1])[0]

        blackboard['assigned_task_id'] = assigned_task_id

        # Update outbox for GatherLocalInfo to broadcast on next tick
        blackboard['local_comm_outbox'] = {
            "robot_id": (agent.ros_namespace or '').lstrip('/'),
            "comm_radius": _comm_radius,
            "assigned_task_id": assigned_task_id,
        }

        if assigned_task_id is None:
            return Status.FAILURE
        else:
            return Status.SUCCESS


# ── IsTaskCompleted ────────────────────────────────────────────────────────────

class IsTaskCompleted(ConditionWithROSTopics):
    def __init__(self, name, agent):
        super().__init__(name, agent, [
            (String, 'world/fire/list', 'local_tasks_info'),
        ])

    def _predicate(self, agent, blackboard) -> bool:
        cache = self._cache  # 베이스 설계대로 내부 캐시 사용
        if "local_tasks_info" not in cache:
            return False

        # Tasks List: JSON 문자열을 파싱하여 리스트로 변환
        try:
            raw_data = cache["local_tasks_info"].data
            tasks_list = json.loads(raw_data)
        except (json.JSONDecodeError, TypeError):
            tasks_list = []        
        # blackboard["local_tasks_info"] = tasks_list  # ✅ 매 프레임 새로 갱신하기보다는 GatherLocalInfo에서만 업데이트하도록 변경

        # Check if assigned task is still in the list (not completed)
        assigned_task_id = blackboard.get('assigned_task_id', None)
        if assigned_task_id is None:
            return False
        task_ids = [t['id'] for t in tasks_list]
        if assigned_task_id in task_ids:
            return False
        
        return True  



# ── MoveToTarget ───────────────────────────────────────────────────────────────

class MoveToTarget(ActionWithROSAction):
    """
    Navigate to the assigned task position using Nav2 NavigateToPose.
    Mirrors space-sim _MoveToTask / agent.follow() → NavigateToPose.
    """

    def __init__(self, name, agent):
        ns = agent.ros_namespace or ''
        super().__init__(name, agent, (NavigateToPose, f'{ns}/navigate_to_pose'))

    def _build_goal(self, agent, blackboard):
        task_id = blackboard.get('assigned_task_id')
        tasks   = blackboard.get('local_tasks_info', [])
        task    = next((t for t in tasks if t['id'] == task_id), None)
        if task is None:
            return False

        ps = PoseStamped()
        ps.header.frame_id    = 'map'
        ps.header.stamp       = self.ros.node.get_clock().now().to_msg()
        ps.pose.position.x    = float(task['x'])
        ps.pose.position.y    = float(task['y'])
        ps.pose.orientation.w = 1.0

        goal      = NavigateToPose.Goal()
        goal.pose = ps
        return goal


# ── ExecuteTask ────────────────────────────────────────────────────────────────

class ExecuteTask(Node):
    """Fire를 suppress하기 위해 /world/fire/suppress 토픽에 fire_id를 publish"""
    
    def __init__(self, name, agent):
        super().__init__(name)
        self.type = "Action"
        self.status = Status.RUNNING
        self._pub = agent.ros_bridge.node.create_publisher(
            # String, '/world/fire/suppress', 1
            String, '/world/fire/reduce', 1            
        )
    
    async def run(self, agent, blackboard):
        fire_id = blackboard.get("assigned_task_id", None)
        if fire_id is None:
            self.status = Status.FAILURE
            return self.status
        
        # fire_id를 suppress 토픽으로 publish
        msg = String()
        msg.data = str(fire_id)
        self._pub.publish(msg)
        
        self.status = Status.SUCCESS
        return self.status


class Explore(ActionWithROSAction):
    """
    Navigate to a random point within the map bounds.
    Mirrors space-sim _ExploreArea.
    """

    def __init__(self, name, agent, timeout=20.0):
        ns = agent.ros_namespace or ''
        super().__init__(name, agent, (NavigateToPose, f'{ns}/navigate_to_pose'))
        self.timeout = timeout  # 최대 탐색 시간 (초)
        self.time_started = None

    def get_random_goal(self):
        x = random.uniform(
            _map_bounds.get('x_min', -10.0), _map_bounds.get('x_max', 10.0)
        )
        y = random.uniform(
            _map_bounds.get('y_min', -10.0), _map_bounds.get('y_max', 10.0)
        )
        return x, y

    def _build_goal(self, agent, blackboard):
        ps = PoseStamped()
        ps.header.frame_id    = 'map'
        ps.header.stamp       = self.ros.node.get_clock().now().to_msg()
        
        x, y = self.get_random_goal()
        ps.pose.position.x    = x
        ps.pose.position.y    = y
        ps.pose.orientation.w = 1.0

        goal      = NavigateToPose.Goal()
        goal.pose = ps

        self.time_started = self.ros.node.get_clock().now().nanoseconds / 1e9  # 시간 초기화  
        return goal

    # ★ RUNNING 중 타임아웃 시 새로운 목표로 갱신
    def _on_running(self, agent, blackboard):
        if self.time_started is None:
            return  # 아직 목표가 설정되지 않음
        
        elapsed_time = (self.ros.node.get_clock().now().nanoseconds / 1e9) - self.time_started
        if elapsed_time > self.timeout:
            # 타임아웃: 현재 목표 취소 및 새로운 랜덤 목표로 갱신
            if self._goal_handle is not None:
                self._goal_handle.cancel_goal_async()
            
            # 새로운 목표 생성 및 송신
            new_goal = self._build_goal(agent, blackboard)
            if new_goal is not False:
                self.client.send_goal_async(new_goal).add_done_callback(self._on_goal_response)
            self.time_started = self.ros.node.get_clock().now().nanoseconds / 1e9  # 시간 초기화  



class IsArrivedAtTarget(ConditionWithROSTopics):
    def __init__(self, name, agent, default_thresh=0.6):
        ns = agent.ros_namespace or ""
        super().__init__(name, agent, [(PoseStamped, f"{ns}/pose_world", "ego_pose")])
        self.default_thresh = default_thresh
        self._target_xy = {}
        self._subs = {}

    def _predicate(self, agent, blackboard):
        cache = self._cache  # 베이스 설계대로 내부 캐시 사용
        if "ego_pose" not in cache:
            return False
        
        ego_pose = cache["ego_pose"]
        target_id = blackboard.get("assigned_task_id", None)

        local_tasks_info = blackboard.get("local_tasks_info", [])
        target_info = next((t for t in local_tasks_info if t['id'] == target_id), None)
        if target_info is None:
            return False


        dist = math.hypot(ego_pose.pose.position.x - target_info['x'], ego_pose.pose.position.y - target_info['y'])
        return dist <= self.default_thresh + target_info['radius']
