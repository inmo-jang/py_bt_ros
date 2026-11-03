"""Reactive detection must cancel travel and persist person (not robot) coordinates."""
import asyncio,json,threading,tempfile
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from geometry_msgs.msg import PoseStamped
from rclpy.task import Future
from rclpy.time import Time
from modules.utils import set_config
ROOT=Path(__file__).resolve().parents[1]
set_config(ROOT/'scenarios/limo_factory_search/config_sim.yaml')
from modules.base_bt_nodes import Status
from modules.bt_constructor import build_behavior_tree
from scenarios.limo_factory_search.agent import Agent

class Client:
    def __init__(self,*_):
        self.goals=[];self.accepted=True;self.result=Future();self.cancels=0;self.pending=False
    def server_is_ready(self):return True
    def cancel(self):self.cancels+=1;return Future()
    def send_goal_async(self,goal):
        self.goals.append(goal)
        self.handle=SimpleNamespace(accepted=self.accepted,get_result_async=lambda:self.result,cancel_goal_async=self.cancel)
        self.future=Future()
        if not self.pending:self.future.set_result(self.handle)
        return self.future

class SearchTest(unittest.TestCase):
    def setUp(self):
        self.seconds=100.;self.found=False;self.saved={'position':{'x':5.55,'y':.65,'z':0.}}
        clock=SimpleNamespace(now=lambda:Time(seconds=self.seconds))
        self.agent=SimpleNamespace(node=SimpleNamespace(get_clock=lambda:clock),
            ros_bridge=SimpleNamespace(node=SimpleNamespace(get_clock=lambda:clock)),
            options={'navigation_action':'/navigate_to_pose','server_timeout':30.,'navigation_timeout':240.,'perception_timeout':5.,'detection_max_age':1.},
            regions={1:{},2:{},3:{}},shutdown_futures=[],publish_status=lambda _:None,region_pose=lambda _:PoseStamped(),
            person_observation=lambda:self.found,record_person=lambda:self.saved)
        self.board=dict(error='',checked_regions=[],found_region=None,current_region=0,target_region=0,mission_id=1)
        self.agent.begin_region=lambda r:self.board.update(current_region=0,target_region=r)
        def enter(r):self.board['current_region']=r;self.board['checked_regions'].append(r)
        self.agent.enter_region=enter
        p=patch('modules.base_bt_nodes_ros.ActionClient',Client);p.start();self.addCleanup(p.stop)
        self.tree=build_behavior_tree(self.agent,ROOT/'scenarios/limo_factory_search/default_bt.xml','scenarios.limo_factory_search')
        self.moves=self.tree.children[1].children[:3]
    def tick(self):self.tree.reset();return asyncio.run(self.tree.run(self.agent,self.board))
    def arrive(self,i):
        self.moves[i-1].client.result.set_result(SimpleNamespace(status=4,result=SimpleNamespace(error_code=0)))
        return self.tick()
    def test_detection_during_first_move_cancels_and_records(self):
        self.assertEqual(self.tick(),Status.RUNNING);self.found=True
        self.assertEqual(self.tick(),Status.SUCCESS)
        self.assertEqual(self.moves[0].client.cancels,1)
        self.assertEqual(self.moves[1].client.goals,[])
        self.assertEqual(self.board['person_record'],self.saved)
    def test_all_false_visits_in_order_and_fails(self):
        self.tick();self.assertEqual(self.arrive(1),Status.RUNNING)
        self.assertEqual(self.arrive(2),Status.RUNNING);self.assertEqual(self.arrive(3),Status.FAILURE)
        self.assertEqual(self.board['checked_regions'],[1,2,3])
        self.assertTrue(all(len(n.client.goals)==1 for n in self.moves))
    def test_missing_position_stops_travel_then_fails(self):
        self.tick();self.found=True;self.saved=None
        self.assertEqual(self.tick(),Status.RUNNING);self.assertEqual(self.moves[0].client.cancels,1)
        self.seconds+=5.;self.assertEqual(self.tick(),Status.FAILURE)
        self.assertIn('position',self.board['error']);self.assertEqual(self.moves[1].client.goals,[])
    def test_sensor_loss_cancels_travel(self):
        self.tick();self.found=None;self.tick();self.seconds+=5.
        self.assertEqual(self.tick(),Status.FAILURE);self.assertEqual(self.moves[0].client.cancels,1)
    def test_rejected_navigation_does_not_send_next_goal(self):
        self.moves[0].client.accepted=False
        self.assertEqual(self.tick(),Status.FAILURE);self.assertEqual(self.moves[1].client.goals,[])
    def test_record_person_uses_detection_position_and_appends_json(self):
        with tempfile.TemporaryDirectory() as d:
            self.agent.options.update(global_frame='map',person_log=d+'/people.jsonl')
            self.agent.observation_lock=threading.Lock();self.agent.mission_started=99.
            self.agent.blackboard=self.board;self.agent.found_pub=SimpleNamespace(publish=lambda _:None)
            p=PoseStamped();p.header.frame_id='map';p.header.stamp=Time(seconds=99.5).to_msg();p.pose.position.x=5.55;p.pose.position.y=.65
            self.agent.person_pose=p
            saved=Agent.record_person(self.agent)
            self.assertEqual(saved['position'],dict(x=5.55,y=.65,z=0.))
            self.assertEqual(json.loads(Path(d+'/people.jsonl').read_text()),saved)
            self.seconds+=2.;self.assertIsNone(Agent.record_person(self.agent))

if __name__=='__main__':unittest.main()
