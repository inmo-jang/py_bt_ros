import random
import time
import pygame
from modules.utils import config
MODE = config['decision_making']['FirstClaimGreedy']['mode']
W_FACTOR_COST = config['decision_making']['FirstClaimGreedy']['weight_factor_cost']
ENFORCED_COLLABORATION = config['decision_making']['FirstClaimGreedy'].get('enforced_collaboration', False)

class FirstClaimGreedy: # Task selection within each agent's `situation_awareness_radius`
    def __init__(self, agent):
        self.agent = agent
        self.assigned_task = None
        self.my_claim_time = {}  # task_id -> 내가 해당 task를 claim한 timestamp

    def decide(self, blackboard):
        # Place your decision-making code for each agent
        '''
        Output:
            - `task_id`, if task allocation works well
            - `None`, otherwise
        '''
        local_tasks_info = blackboard.get('local_tasks_info', {})
        assigned_task_id = blackboard.get('assigned_task_id', None)

        # Check if the existing task is still available
        self.assigned_task = local_tasks_info.get(assigned_task_id)

        # Give up the decision-making process if there is no task nearby
        if len(local_tasks_info) == 0:
            self.assigned_task = None
            self.agent.message_to_share = {
                'agent_id': self.agent.agent_id,
                'assigned_task_id': None,
                'time_stamp': None,
            }
            return None

        # Conflict resolution: 현재 assigned task를 이웃이 먼저 claim했으면 양보
        if self.assigned_task is not None:
            if self._has_priority_conflict(assigned_task_id):
                self.assigned_task = None  # 더 빠른 claim이 있으므로 양보
                self.my_claim_time.pop(assigned_task_id, None)

        # 매 tick마다 재평가: conflict resolution 후 최적 task 선택
        candidates = self.filter_tasks_with_conflict_resolution(list(local_tasks_info.values()))
        if len(candidates) == 0:
            self.agent.message_to_share = {
                'agent_id': self.agent.agent_id,
                'assigned_task_id': None,
                'time_stamp': None,
            }
            return None

        if MODE == "Random":
            target_task_id = random.choice(candidates).task_id
        elif MODE == "MinDist":
            target_task_id = self.find_min_dist_task(candidates)
        elif MODE == "MaxUtil":
            target_task_id, _ = self.find_max_utility_task(candidates)

        self.assigned_task = local_tasks_info[target_task_id]
        # 새로 claim한 경우에만 timestamp 기록 (이미 있으면 유지)
        if target_task_id not in self.my_claim_time:
            self.my_claim_time[target_task_id] = time.time()

        self.agent.message_to_share = {
            'agent_id': self.agent.agent_id,
            'assigned_task_id': self.assigned_task.task_id,
            'time_stamp': self.my_claim_time.get(self.assigned_task.task_id),
        }

        return self.assigned_task.task_id

    def _has_priority_conflict(self, task_id) -> bool:
        """이웃이 나보다 먼저 task_id를 claim했으면 True 반환."""
        my_time = self.my_claim_time.get(task_id)
        for msg in self.agent.messages_received:
            if msg.get('assigned_task_id') != task_id:
                continue
            neighbor_time = msg.get('time_stamp')
            if neighbor_time is None:
                continue
            if my_time is None or neighbor_time < my_time:
                return True  # 이웃이 더 빠른 claim을 가짐
        return False

    def filter_tasks_with_conflict_resolution(self, tasks_info):
        """내가 우선권을 가지는 task만 반환 (이웃이 먼저 claim했으면 제외)."""
        result = []
        for task in tasks_info:
            task_id = task.task_id
            my_time = self.my_claim_time.get(task_id)

            yielded = False
            for msg in self.agent.messages_received:
                if msg.get('assigned_task_id') != task_id:
                    continue
                neighbor_time = msg.get('time_stamp')
                if neighbor_time is None:
                    # timestamp 없는 이웃 claim → 보수적으로 양보
                    yielded = True
                    break
                if my_time is None or neighbor_time < my_time:
                    # 이웃이 먼저 claim → 양보
                    yielded = True
                    break

            if not yielded:
                result.append(task)
            else:
                self.my_claim_time.pop(task_id, None)

        return result

    def find_min_dist_task(self, tasks_info):
        _tasks_distance = {
            task.get('task_id'): self.compute_distance(task) for task in tasks_info
        }
        _min_task_id = min(_tasks_distance, key=_tasks_distance.get)
        return _min_task_id

    def find_max_utility_task(self, tasks_info):
        _current_utilities = {
            task.get('task_id'): self.compute_utility(task) for task in tasks_info
        }

        _max_task_id = max(_current_utilities, key=_current_utilities.get)
        _max_utility = _current_utilities[_max_task_id]

        return _max_task_id, _max_utility

    def compute_utility(self, task): # Individual Utility Function
        if task is None:
            return float('-inf')

        distance = (self.agent.position - task.position).length()
        return task.amount - W_FACTOR_COST * distance

    def compute_distance(self, task): # Individual Utility Function
        if task is None:
            return float('inf')

        distance = (self.agent.position - task.position).length()
        return distance
