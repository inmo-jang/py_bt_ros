import math
import json
import random

import pygame

from modules.base_bt_nodes import BTNodeList, Status, Sequence, Fallback, ReactiveSequence, ReactiveFallback, AssignTask, SyncCondition
from modules.base_bt_nodes_ros import ActionWithROSAction, ActionWithROSTopic, ConditionWithROSTopics
from modules.utils import config, AttrDict, msg_serialize_default, msg_deserialize_hook

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

_map_bounds = config.get('tasks', {}).get('locations', {})


# ── Nodes  ─────────────────────────────────────────────────────────────────────

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

        self.agent = agent  # outbox 송신 위해 agent 속성 저장

    def _predicate(self, agent, blackboard):
        cache = self._cache

        # [1] Outbox broadcast: 이전 틱에서 설정한 상태를 먼저 송신
        outbox = getattr(agent, 'message_to_share', {})  # GatherLocalInfo 실행 시점에 agent의 임시 속성에서 메시지 가져오기
        if outbox is not None:
            msg = String()
            msg.data = json.dumps(outbox, default=msg_serialize_default)
            self._pub_outbox.publish(msg)

        # [2] 필수 topic 수신 확인: 하나라도 없으면 False
        required = ["ego_pose", "local_tasks_info"]
        if any(k not in cache for k in required):
            return False

        # [3] 필수 데이터 처리
        try:
            tasks_list = [AttrDict(t) for t in json.loads(cache["local_tasks_info"].data)]
            for task in tasks_list:
                task['position'] = pygame.math.Vector2(task['x'], task['y'])
                task['amount'] = task.get('radius', 0.0)
                # 여기서 또다른 전처리가 필요하면 추가 가능
        except (json.JSONDecodeError, TypeError):
            tasks_list = []
        blackboard["local_tasks_info"] = {t.task_id: t for t in tasks_list}
        self.agent.position = pygame.math.Vector2(cache["ego_pose"].pose.position.x, cache["ego_pose"].pose.position.y)

        # [4] 수신 메시지: 미수신 시 빈 리스트로 폴백
        try:
            self.agent.messages_received = json.loads(cache["local_comm_inbox"].data, object_hook=msg_deserialize_hook)
        except (KeyError, AttributeError, json.JSONDecodeError, TypeError):
            self.agent.messages_received = []

        return True


class IsTaskCompleted(SyncCondition):
    def __init__(self, name, agent):
        super().__init__(name, self._update)

    def _update(self, agent, blackboard):
        assigned_task_id = blackboard.get('assigned_task_id', None)
        if assigned_task_id is None:
            return Status.FAILURE

        local_tasks_info = blackboard.get('local_tasks_info', {})
        if assigned_task_id in local_tasks_info:
            return Status.FAILURE  # 아직 불이 남아있음

        return Status.SUCCESS  # 불이 사라짐 = 완료


class IsArrivedAtTarget(ConditionWithROSTopics):
    def __init__(self, name, agent, default_thresh=1.2):
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

        target_info = blackboard.get("local_tasks_info", {}).get(target_id)
        if target_info is None:
            return False

        dist = math.hypot(ego_pose.pose.position.x - target_info['x'], ego_pose.pose.position.y - target_info['y'])
        return dist <= self.default_thresh + target_info['radius']


class MoveToTarget(ActionWithROSAction):
    """
    Navigate to the assigned task position using Nav2 NavigateToPose.
    Mirrors space-sim _MoveToTask / agent.follow() → NavigateToPose.
    """

    def __init__(self, name, agent):
        ns = agent.ros_namespace or ''
        super().__init__(name, agent, (NavigateToPose, f'{ns}/navigate_to_pose'))
        self.moving_task_id = None  # 현재 이동 중인 task 정보 저장 (없으면 None)

    def _build_goal(self, agent, blackboard):
        task_id = blackboard.get('assigned_task_id')
        task    = blackboard.get('local_tasks_info', {}).get(task_id)
        if task is None:
            return False

        self.moving_task_id = task_id  # 이동 시작 시점에 task_id 저장

        ps = PoseStamped()
        ps.header.frame_id    = 'world'
        ps.header.stamp       = self.ros.node.get_clock().now().to_msg()
        ps.pose.position.x    = float(task['x'])
        ps.pose.position.y    = float(task['y'])
        ps.pose.orientation.w = 1.0

        goal      = NavigateToPose.Goal()
        goal.pose = ps
        return goal

    def _on_running(self, agent, blackboard):
        # 이동 중에도 목표 위치가 유효한지 체크: 만약 할당된 Task이 사라졌다면 목표 취소
        self.status = Status.RUNNING  # 기본적으로 RUNNING 유지
        task_id = blackboard.get('assigned_task_id')
        if task_id != self.moving_task_id:        
            if self._goal_handle is not None:
                self._goal_handle.cancel_goal_async()
                self.status = Status.FAILURE  # 목표 취소 후 실패 반환
        return self.status


class ExecuteTask(ActionWithROSTopic):
    """Fire를 suppress하기 위해 /world/fire/reduce 토픽에 fire_id를 publish"""

    def __init__(self, name, agent):
        super().__init__(name, agent, (String, '/world/fire/reduce'))

    def _build_message(self, agent, blackboard):
        fire_id = blackboard.get("assigned_task_id", None)
        if fire_id is None:
            return None

        msg = String()
        msg.data = str(fire_id)
        return msg


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
        ps.header.frame_id    = 'world'
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
        return Status.RUNNING

