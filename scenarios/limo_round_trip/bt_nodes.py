"""Nav2 actions in the same node/registration style as turtle_catcher."""
from copy import deepcopy
import math
import time

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose

from modules.base_bt_nodes import BTNodeList as BaseBTNodeList, Node, Sequence, Status
from modules.base_bt_nodes_ros import ActionWithROSAction


class BTNodeList(BaseBTNodeList):
    # Register scenario actions without changing the lists of other scenarios.
    ACTION_NODES = ['MoveToGoal', 'WaitForSecond']
    CONDITION_NODES = []


class MoveToGoal(ActionWithROSAction):
    """Send one saved pose to Nav2; handle rejection, failure and cancellation."""

    def __init__(self, name, agent, pose_key='goal_pose'):
        if pose_key not in ('goal_pose', 'home_pose'):
            raise ValueError('pose_key must be goal_pose or home_pose')
        self.pose_key = pose_key
        self.agent = agent
        super().__init__(
            name, agent, (NavigateToPose, agent.options['navigation_action']))
        self.name = f'{name} ({"goal" if pose_key == "goal_pose" else "home"})'
        self.goal_future = None
        self.started_at = None
        self.sent_at = None

    async def run(self, agent, blackboard):
        try:
            if self.started_at is None:
                self.started_at = time.monotonic()
                agent.publish_status(
                    'MOVING_TO_GOAL' if self.pose_key == 'goal_pose' else 'RETURNING_HOME')

            if self.goal_future is None:
                if not self.client.server_is_ready():
                    if time.monotonic() - self.started_at > agent.options['server_timeout']:
                        raise TimeoutError('Nav2 action server is not ready')
                    self.status = Status.RUNNING
                    return self.status
                goal = self._build_goal(agent, blackboard)
                self.sent_at = self.ros.node.get_clock().now()
                self.goal_future = self.client.send_goal_async(goal)

            if self.goal_future.done() and self._goal_handle is None:
                self._goal_handle = self.goal_future.result()
                if not self._goal_handle.accepted:
                    raise RuntimeError('Nav2 rejected the goal')
                self._result_future = self._goal_handle.get_result_async()

            if self._result_future is not None and self._result_future.done():
                response = self._result_future.result()
                self.status = self._interpret_result(
                    response.result, agent, blackboard, response.status)
                if self.status == Status.FAILURE:
                    raise RuntimeError(
                        f'Nav2 navigation failed: status={response.status}, '
                        f'error_code={getattr(response.result, "error_code", 0)}')
            else:
                elapsed = (self.ros.node.get_clock().now() - self.sent_at).nanoseconds / 1e9
                if elapsed < 0:
                    raise RuntimeError('Simulation clock moved backwards; restart the mission')
                if elapsed > agent.options['navigation_timeout']:
                    raise TimeoutError('Navigation exceeded navigation_timeout')
                if time.monotonic() - self.started_at > agent.options['server_timeout']:
                    if not self.goal_future.done():
                        raise TimeoutError('Nav2 did not acknowledge the goal')
                self.status = Status.RUNNING
        except Exception as error:
            blackboard['error'] = f'{self.name}: {error}'
            self.status = Status.FAILURE
        return self.status

    def _build_goal(self, agent, blackboard):
        goal = NavigateToPose.Goal()
        goal.pose = deepcopy(blackboard[self.pose_key])
        goal.pose.header.stamp = self.ros.node.get_clock().now().to_msg()
        return goal

    def _interpret_result(self, result, agent, blackboard, status_code=None):
        succeeded = status_code == GoalStatus.STATUS_SUCCEEDED
        return Status.SUCCESS if succeeded and not getattr(result, 'error_code', 0) else Status.FAILURE

    def _cancel_accepted_goal(self, future):
        try:
            handle = future.result()
            if handle.accepted:
                self.agent.shutdown_futures.append(handle.cancel_goal_async())
        except Exception:
            pass  # The action server or ROS context may already be gone.

    def halt(self):
        if self.goal_future is not None:
            if self._goal_handle is None:
                self.agent.shutdown_futures.append(self.goal_future)
                self.goal_future.add_done_callback(self._cancel_accepted_goal)
            elif self._goal_handle.accepted:
                if self._result_future is None or not self._result_future.done():
                    try:
                        self.agent.shutdown_futures.append(self._goal_handle.cancel_goal_async())
                    except Exception:
                        pass
        self.goal_future = None
        self._goal_handle = None
        self._result_future = None
        self.started_at = None
        self.sent_at = None


class WaitForSecond(Node):
    """Use simulation time without blocking ROS callbacks or the BT viewer."""

    def __init__(self, name, agent, seconds=10.0):
        super().__init__(name)
        self.type = 'Action'
        self.seconds = float(seconds)
        if not math.isfinite(self.seconds) or self.seconds < 0:
            raise ValueError('seconds must be finite and nonnegative')
        self.name = f'{name} ({self.seconds:g}s)'
        self.started_at = None

    async def run(self, agent, blackboard):
        now = agent.node.get_clock().now()
        if self.started_at is None:
            self.started_at = now
            agent.publish_status('WAITING')
        elapsed = (now - self.started_at).nanoseconds / 1e9
        if elapsed < 0:
            blackboard['error'] = 'Simulation clock moved backwards during wait'
            self.status = Status.FAILURE
        else:
            self.status = Status.SUCCESS if elapsed >= self.seconds else Status.RUNNING
        return self.status

    def halt(self):
        self.started_at = None
