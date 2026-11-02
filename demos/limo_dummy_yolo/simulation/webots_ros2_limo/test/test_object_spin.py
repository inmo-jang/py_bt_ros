import unittest
from webots_ros2_limo.object_spin import DetectionGate

def message(t,score=.9,name='box'):
 return {'stamp':{'sec':int(t),'nanosec':int(round((t-int(t))*1e9))},'detections':[{'class_name':name,'confidence':score}]}

class DetectionGateTests(unittest.TestCase):
 def test_three_fresh_detections(self):
  g=DetectionGate()
  self.assertFalse(g.accept(message(10.),10.1));self.assertFalse(g.accept(message(10.2),10.3));self.assertTrue(g.accept(message(10.4),10.5))
 def test_repeated_frame_does_not_count(self):
  g=DetectionGate()
  for _ in range(5):self.assertFalse(g.accept(message(10.),10.1))
  self.assertEqual(g.count,1)
 def test_miss_breaks_consecutive_run(self):
  g=DetectionGate();g.accept(message(10.),10.1);g.accept(message(10.2),10.3)
  self.assertFalse(g.accept(message(10.4,.4),10.5));self.assertEqual(g.count,0)
  self.assertFalse(g.accept(message(10.6),10.7))
 def test_stale_or_future_frames_do_not_trigger(self):
  g=DetectionGate()
  self.assertFalse(g.accept(message(10.),15.));self.assertFalse(g.accept(message(20.),15.));self.assertEqual(g.count,0)
 def test_wrong_class_and_gap(self):
  g=DetectionGate();self.assertFalse(g.accept(message(10.,name='person'),10.1))
  g.accept(message(10.2),10.3);g.accept(message(10.4),10.5)
  self.assertFalse(g.accept(message(12.),12.1));self.assertEqual(g.count,1)


class ResumeTests(unittest.TestCase):
 def test_resume_uses_saved_pose_and_new_stamp(self):
  from types import SimpleNamespace
  from concurrent.futures import Future
  from geometry_msgs.msg import PoseStamped
  from builtin_interfaces.msg import Time
  from webots_ros2_limo.object_spin import ObjectSpin
  target=PoseStamped();target.header.frame_id='map';target.pose.position.x=1.2;target.pose.position.y=3.4;target.pose.orientation.w=1.
  sent=[];reported=[]
  def send(g):sent.append(g);return Future()
  n=SimpleNamespace(target=target,target_revision=7,navigation=SimpleNamespace(send_goal_async=send),get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(to_msg=lambda:Time(sec=123))),report=lambda *a:reported.append(a))
  ObjectSpin.navigate(n,True)
  self.assertEqual(sent[0].pose.pose,target.pose);self.assertEqual(sent[0].pose.header.stamp.sec,123)
  self.assertEqual(target.header.stamp.sec,0);self.assertEqual(reported[0][0],'RESUMING')
 def test_same_object_cannot_interrupt_resumed_goal(self):
  from types import SimpleNamespace
  from std_msgs.msg import String
  from webots_ros2_limo.object_spin import ObjectSpin
  n=SimpleNamespace(phase='RESUMING',reacted=True,target=object(),nav_busy={'nav':True})
  ObjectSpin.detect(n,String(data='invalid JSON is never evaluated'))
 def test_old_goal_result_does_not_finish_new_goal(self):
  from types import SimpleNamespace
  from concurrent.futures import Future
  from webots_ros2_limo.object_spin import ObjectSpin
  result=Future();result.set_result(SimpleNamespace(status=4));pending=Future();pending.set_result(None)
  reported=[]
  n=SimpleNamespace(nav_pending=pending,nav_handle=object(),nav_result=result,nav_cancel=None,phase='MOVING',nav_revision=1,target_revision=2,report=lambda *a:reported.append(a))
  ObjectSpin.pump_navigation(n)
  self.assertEqual(reported,[]);self.assertIsNone(n.nav_pending)


class MeasuredSpinTests(unittest.TestCase):
 def run_completion(self,degrees):
  import math
  from types import SimpleNamespace
  from concurrent.futures import Future
  from webots_ros2_limo.object_spin import ObjectSpin
  pending=Future();pending.set_result(None)
  result=Future();result.set_result(SimpleNamespace(status=4,result=SimpleNamespace(error_code=0,error_msg='')))
  reports=[];resumed=[]
  n=SimpleNamespace(now=lambda:10.,pump_navigation=lambda:None,phase='SPINNING',started=0.,last_status=10.,reason=None,spin_pending=pending,spin_handle=object(),spin_result=result,spin_cancel=None,odom_angle=math.radians(degrees),target=object(),report=lambda *a:reports.append(a),navigate=lambda x:resumed.append(x))
  ObjectSpin.tick(n)
  return reports,resumed
 def test_action_success_without_turn_does_not_resume(self):
  reports,resumed=self.run_completion(90)
  self.assertEqual(reports[-1][0],'FAILURE');self.assertEqual(resumed,[])
 def test_measured_full_turn_resumes(self):
  reports,resumed=self.run_completion(360)
  self.assertEqual(reports[-1][0],'DONE');self.assertEqual(resumed,[True])


class NavigationRetryTests(unittest.TestCase):
 def test_aborted_goal_gets_one_delayed_retry(self):
  from types import SimpleNamespace
  from concurrent.futures import Future
  from webots_ros2_limo.object_spin import ObjectSpin
  pending=Future();pending.set_result(None);result=Future();result.set_result(SimpleNamespace(status=6,result=SimpleNamespace(error_msg='Transform too old')))
  reports=[];n=SimpleNamespace(nav_pending=pending,nav_handle=object(),nav_result=result,nav_cancel=None,phase='RESUMING',nav_revision=1,target_revision=1,navigation_attempts=0,navigation_retries=1,now=lambda:10.,report=lambda *a:reports.append(a))
  ObjectSpin.pump_navigation(n)
  self.assertEqual(reports[-1][0],'REPLANNING');self.assertEqual(n.retry_at,12.);self.assertEqual(n.navigation_attempts,1)
 def test_user_cancellation_is_not_retried(self):
  from types import SimpleNamespace
  from concurrent.futures import Future
  from webots_ros2_limo.object_spin import ObjectSpin
  pending=Future();pending.set_result(None);result=Future();result.set_result(SimpleNamespace(status=5,result=SimpleNamespace(error_msg='')))
  reports=[];n=SimpleNamespace(nav_pending=pending,nav_handle=object(),nav_result=result,nav_cancel=None,phase='MOVING',nav_revision=1,target_revision=1,report=lambda *a:reports.append(a))
  ObjectSpin.pump_navigation(n)
  self.assertEqual(reports[-1][0],'STOPPED')


class WorkDistanceTests(unittest.TestCase):
 def check_distance(self,gap):
  from types import SimpleNamespace
  from webots_ros2_limo.object_spin import ObjectSpin
  calls=[]
  n=SimpleNamespace(current_gap=lambda:gap,now=lambda:10.,started=0.,box_observation={'base':[1.,0.,0.]},work_distance=.3,distance_tolerance=.1,approach_attempts=0,start_approach=lambda:calls.append('approach'),begin_spin=lambda:calls.append('spin'),report=lambda *a:calls.append(a[0]))
  ObjectSpin.verify_distance(n)
  return calls
 def test_far_detection_approaches_not_spins(self):self.assertEqual(self.check_distance(1.4),['approach'])
 def test_work_distance_allows_spin(self):self.assertEqual(self.check_distance(.3),['spin'])
 def test_too_close_does_not_spin(self):self.assertEqual(self.check_distance(.1),['FAILURE'])
 def test_missing_depth_does_not_spin(self):self.assertEqual(self.check_distance(None),['FAILURE'])

if __name__=='__main__':unittest.main()
