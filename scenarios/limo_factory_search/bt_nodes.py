"""Existing Nav2/ROS condition style, with ordered region search semantics."""
from modules.base_bt_nodes import BTNodeList as BaseBTNodeList, Fallback, Sequence, Status, Node, AlwaysFailure
from modules.base_bt_nodes_ros import ConditionWithROSTopics
from scenarios.limo_round_trip.bt_nodes import MoveToGoal as NavMoveToGoal, WaitForSecond


class BTNodeList(BaseBTNodeList):
    ACTION_NODES = ['MoveToGoal', 'WaitForSecond', 'RecordPerson']
    CONDITION_NODES = ['IsPerson', 'AlwaysFailure']


class MoveToGoal(NavMoveToGoal):
    def __init__(self, name, agent, region):
        self.region = int(region)
        if self.region not in agent.regions:
            raise ValueError(f'Unknown search region: {region}')
        self.arrival_handled = False
        super().__init__(name, agent)
        self.name = f'{name} (R{self.region})'

    def _build_goal(self, agent, blackboard):
        blackboard['goal_pose'] = agent.region_pose(self.region)
        return super()._build_goal(agent, blackboard)

    async def run(self, agent, blackboard):
        # A navigation/sensor failure must not be interpreted as "no person".
        if blackboard.get('error'):
            self.status = Status.FAILURE
            return self.status
        if self.started_at is None:
            agent.begin_region(self.region)
        status = await super().run(agent, blackboard)
        if status == Status.SUCCESS and not self.arrival_handled:
            self.arrival_handled = True
            agent.enter_region(self.region)
        return status

    def halt(self):
        super().halt()
        self.arrival_handled = False


def halt_recursive(node):
    for child in getattr(node, 'children', []):
        halt_recursive(child)
    node.halt()


class ReactiveFallback(Node):
    """Recheck detection each tick; cancel lower-priority navigation on preemption."""
    def __init__(self, name, children):
        super().__init__(name)
        self.children = children

    async def run(self, agent, blackboard):
        for index, child in enumerate(self.children):
            status = await child.run(agent, blackboard)
            if status != Status.FAILURE:
                for lower in self.children[index + 1:]:
                    halt_recursive(lower)
                self.status = status
                return status
        self.status = Status.FAILURE
        return self.status


class IsPerson(ConditionWithROSTopics):
    def __init__(self, name, agent):
        super().__init__(name, agent, [])
        self.missing_since = None
        self.mission_id = None

    async def run(self, agent, blackboard):
        now = agent.node.get_clock().now().nanoseconds / 1e9
        if self.mission_id != blackboard.get('mission_id'):
            self.mission_id = blackboard.get('mission_id')
            self.missing_since = None
        found = agent.person_observation()
        if found is None:
            if self.missing_since is None:
                self.missing_since = now
            if now - self.missing_since >= agent.options['perception_timeout']:
                blackboard['error'] = 'No fresh person observation'
            # Brief startup gaps allow navigation; prolonged loss fails the mission.
            self.status = Status.FAILURE
        else:
            self.missing_since = None
            self.status = Status.SUCCESS if found else Status.FAILURE
        return self.status

    def halt(self):
        pass


class RecordPerson(Node):
    def __init__(self, name, agent):
        super().__init__(name)
        self.type = 'Action'
        self.started_at = None

    async def run(self, agent, blackboard):
        now = agent.node.get_clock().now().nanoseconds / 1e9
        if self.started_at is None:
            self.started_at = now
            agent.publish_status('PERSON_DETECTED')
        try:
            saved = agent.record_person()
            if saved:
                blackboard['person_record'] = saved
                # Travel target is not the person's room; the recorded map position is authoritative.
                blackboard['found_region'] = None
                self.status = Status.SUCCESS
            elif now - self.started_at >= agent.options['perception_timeout']:
                blackboard['error'] = 'Person detected but no fresh map-transformable person position'
                self.status = Status.FAILURE
            else:
                self.status = Status.RUNNING
        except (OSError, ValueError) as error:
            blackboard['error'] = f'Cannot save person position: {error}'
            self.status = Status.FAILURE
        return self.status

    def halt(self):
        self.started_at = None
