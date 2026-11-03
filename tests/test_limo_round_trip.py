"""Check mission transitions with controlled Nav2 results and simulation time."""
import asyncio
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from geometry_msgs.msg import PoseStamped
from rclpy.task import Future
from rclpy.time import Time

from modules.utils import set_config

ROOT = Path(__file__).resolve().parents[1]
set_config(ROOT / 'scenarios/limo_round_trip/config_sim.yaml')

from modules.base_bt_nodes import Status
from modules.bt_constructor import build_behavior_tree
from scenarios.limo_round_trip.bt_nodes import WaitForSecond


class Clock:
    seconds = 100.0

    def now(self):
        return Time(seconds=self.seconds)


class Handle:
    accepted = True

    def __init__(self):
        self.result = Future()
        self.cancelled = False

    def get_result_async(self):
        return self.result

    def cancel_goal_async(self):
        self.cancelled = True
        future = Future()
        future.set_result(None)
        return future


class Client:
    def __init__(self, *args):
        self.ready = True
        self.goals = []
        self.handle = Handle()
        self.defer_acceptance = False
        self.sent = None

    def server_is_ready(self):
        return self.ready

    def send_goal_async(self, goal):
        self.goals.append(goal)
        self.sent = Future()
        if not self.defer_acceptance:
            self.sent.set_result(self.handle)
        return self.sent


class MissionTest(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.stages = []
        self.agent = SimpleNamespace(
            ros_bridge=SimpleNamespace(node=SimpleNamespace(get_clock=lambda: self.clock)),
            node=SimpleNamespace(get_clock=lambda: self.clock),
            options={'navigation_action': '/navigate_to_pose',
                     'server_timeout': 30.0, 'navigation_timeout': 240.0},
            shutdown_futures=[], publish_status=self.stages.append)
        self.board = {'goal_pose': PoseStamped(), 'home_pose': PoseStamped(), 'error': ''}
        self.board['goal_pose'].pose.position.x = 2.0
        self.board['home_pose'].pose.position.x = -1.0
        patcher = patch('modules.base_bt_nodes_ros.ActionClient', Client)
        patcher.start()
        self.addCleanup(patcher.stop)
        self.tree = build_behavior_tree(
            self.agent, str(ROOT / 'scenarios/limo_round_trip/default_bt.xml'),
            'scenarios.limo_round_trip')

    def tick(self):
        self.tree.reset()  # Match the standard py_bt_ros Agent tick behavior.
        return asyncio.run(self.tree.run(self.agent, self.board))

    @staticmethod
    def finish(node, status=4, error_code=0):
        node.client.handle.result.set_result(
            SimpleNamespace(status=status, result=SimpleNamespace(error_code=error_code)))

    def test_round_trip_wait_boundary_and_repeated_mission(self):
        outbound, wait, inbound = self.tree.children
        self.assertEqual(self.tick(), Status.RUNNING)
        self.assertEqual(outbound.client.goals[0].pose.pose.position.x, 2.0)
        self.finish(outbound)
        self.assertEqual(self.tick(), Status.RUNNING)
        self.assertEqual(self.stages[-1], 'WAITING')
        for _ in range(5):  # A paused Webots clock cannot advance the timer.
            self.assertEqual(self.tick(), Status.RUNNING)
        self.clock.seconds = 109.99
        self.assertEqual(self.tick(), Status.RUNNING)
        self.assertEqual(inbound.client.goals, [])
        self.clock.seconds = 110.0
        self.assertEqual(self.tick(), Status.RUNNING)
        self.assertEqual(inbound.client.goals[0].pose.pose.position.x, -1.0)
        self.assertEqual(len(outbound.client.goals), 1)
        self.finish(inbound)
        self.assertEqual(self.tick(), Status.SUCCESS)
        self.assertIsNone(wait.started_at)
        outbound.client.handle = Handle()
        self.assertEqual(self.tick(), Status.RUNNING)
        self.assertEqual(len(outbound.client.goals), 2)

    def test_rejection_stops_before_wait_or_return(self):
        outbound, _, inbound = self.tree.children
        outbound.client.handle.accepted = False
        self.assertEqual(self.tick(), Status.FAILURE)
        self.assertIn('rejected', self.board['error'])
        self.assertEqual(inbound.client.goals, [])
        self.assertNotIn('WAITING', self.stages)

    def test_aborted_navigation_does_not_start_return(self):
        outbound, _, inbound = self.tree.children
        self.tick()
        self.finish(outbound, status=6)
        self.assertEqual(self.tick(), Status.FAILURE)
        self.assertNotIn('WAITING', self.stages)
        self.assertEqual(inbound.client.goals, [])

    def test_nav2_error_code_is_failure_even_with_succeeded_status(self):
        self.tick()
        self.finish(self.tree.children[0], error_code=1)
        self.assertEqual(self.tick(), Status.FAILURE)

    def test_halt_cancels_running_navigation(self):
        outbound = self.tree.children[0]
        self.tick()
        outbound.halt()
        self.assertTrue(outbound.client.handle.cancelled)

    def test_halt_cancels_goal_accepted_after_halt(self):
        outbound = self.tree.children[0]
        outbound.client.defer_acceptance = True
        self.tick()
        outbound.halt()
        self.assertFalse(outbound.client.handle.cancelled)
        outbound.client.sent.set_result(outbound.client.handle)
        self.assertTrue(outbound.client.handle.cancelled)

    def test_wait_detects_backwards_simulation_clock(self):
        wait = WaitForSecond('WaitForSecond', self.agent)
        asyncio.run(wait.run(self.agent, self.board))
        self.clock.seconds = 99.0
        self.assertEqual(asyncio.run(wait.run(self.agent, self.board)), Status.FAILURE)

    def test_navigation_timeout_cancels_goal(self):
        self.tick()
        self.clock.seconds = 341.0
        self.assertEqual(self.tick(), Status.FAILURE)
        self.assertTrue(self.tree.children[0].client.handle.cancelled)
        self.assertIn('navigation_timeout', self.board['error'])


if __name__ == '__main__':
    unittest.main()
