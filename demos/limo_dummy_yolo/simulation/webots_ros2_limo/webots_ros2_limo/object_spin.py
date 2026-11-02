"""YOLO detection -> cancel navigation -> confirm stop -> one Nav2 360 spin."""
import json
import math
import time
import copy

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
from nav2_msgs.action import Spin, NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatusArray, GoalStatus
from action_msgs.srv import CancelGoal
from tf2_ros import Buffer, TransformListener, TransformException
from .depth_geometry import transform_xyz


class DetectionGate:
    def __init__(self, confidence=.7, consecutive=3, max_age=2.):
        self.confidence=confidence;self.consecutive=consecutive;self.max_age=max_age
        self.count=0;self.last_stamp=-math.inf

    def accept(self, data, now):
        stamp=data['stamp']['sec']+data['stamp']['nanosec']/1e9
        if not math.isfinite(stamp) or stamp<=self.last_stamp:return False
        gap=stamp-self.last_stamp;self.last_stamp=stamp
        if not 0<=now-stamp<=self.max_age:
            self.count=0;return False
        target=any(d.get('class_name')=='box' and math.isfinite(float(d['confidence'])) and float(d['confidence'])>=self.confidence for d in data['detections'])
        self.count=(self.count+1 if gap<=1 else 1) if target else 0
        return self.count>=self.consecutive


class ObjectSpin(Node):
    def __init__(self):
        super().__init__('limo_object_spin')
        for key,value in [('confidence',.7),('consecutive_frames',3),('navigation_retries',1),('work_distance',.30),('front_offset',.20),('distance_tolerance',.10),('approach_timeout',60.)]:self.declare_parameter(key,value)
        confidence=self.get_parameter('confidence').value;frames=self.get_parameter('consecutive_frames').value
        if not 0<confidence<=1 or frames<1:raise ValueError('Invalid detection gate parameters')
        self.gate=DetectionGate(confidence,frames)
        self.work_distance=float(self.get_parameter('work_distance').value)
        self.front_offset=float(self.get_parameter('front_offset').value)
        self.distance_tolerance=float(self.get_parameter('distance_tolerance').value)
        self.approach_timeout=float(self.get_parameter('approach_timeout').value)
        if self.work_distance<.25 or self.front_offset<.15 or not 0<self.distance_tolerance<self.work_distance-.10:
            raise ValueError('Work distance must retain a safe chassis clearance')
        self.tf_buffer=Buffer();self.tf_listener=TransformListener(self.tf_buffer,self)
        self.box_observation=None;self.locked_box=None;self.after_stop='approach'
        self.approach_attempts=0;self.approach_started=0.;self.nav_role='mission' 
        self.navigation_retries=self.get_parameter('navigation_retries').value;self.navigation_attempts=0;self.retry_at=0.
        self.pub=self.create_publisher(String,'/object_spin/status',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.sub=self.create_subscription(String,'/yolo/detections',self.detect,10)
        self.odom_sub=self.create_subscription(Odometry,'/odom',self.odom,10)
        self.velocity=None;self.odom_at=-math.inf
        qos=QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.nav_busy={};self.status_subs=[];self.cancel_clients={}
        for action in ['navigate_to_pose','navigate_through_poses']:
            self.cancel_clients[action]=self.create_client(CancelGoal,f'/{action}/_action/cancel_goal')
            self.status_subs.append(self.create_subscription(GoalStatusArray,f'/{action}/_action/status',lambda m,k=action:self.nav_status(k,m),qos))
        self.spin=ActionClient(self,Spin,'/spin')
        self.navigation=ActionClient(self,NavigateToPose,'/navigate_to_pose')
        self.target=None;self.reacted=False;self.target_revision=0;self.nav_revision=0
        self.nav_pending=self.nav_handle=self.nav_result=self.nav_cancel=None
        # RViz /goal_pose is already executed by Nav2; remember it without duplicating.
        self.goal_sub=self.create_subscription(PoseStamped,'/goal_pose',lambda m:self.goal(m,False),10)
        # Programmatic callers can use this input: this node owns the initial move too.
        self.owned_goal_sub=self.create_subscription(PoseStamped,'/object_navigation/goal',lambda m:self.goal(m,True),10)
        self.reset_service=self.create_service(Trigger,'/object_spin/reset',self.reset)
        self.stop_service=self.create_service(Trigger,'/object_spin/stop',self.stop)
        self.phase='ARMED';self.cancel_futures={};self.stopped_since=None
        self.spin_pending=self.spin_handle=self.spin_result=self.spin_cancel=None
        self.reason=None;self.angle=0.;self.odom_angle=0.;self.last_yaw=None;self.last_status=0.;self.started=self.now()
        self.timer=self.create_timer(.05,self.tick)
        self.report('ARMED','Waiting for navigation goal; one object reaction per goal')

    def now(self):return self.get_clock().now().nanoseconds/1e9

    def publish_status(self,detail):
        self.pub.publish(String(data=json.dumps({'state':self.phase,'detail':detail,
            'angle_radians':self.angle,'measured_angle_radians':self.odom_angle,
            'front_gap_m':self.current_gap(),'work_distance_m':self.work_distance})))
        self.last_status=self.now()

    def report(self,phase,detail):
        self.phase=phase;self.publish_status(detail)
        self.get_logger().info(f'{phase}: {detail}')

    def nav_status(self,key,msg):
        self.nav_busy[key]=any(s.status in (GoalStatus.STATUS_ACCEPTED,GoalStatus.STATUS_EXECUTING,GoalStatus.STATUS_CANCELING) for s in msg.status_list)

    def odom(self,msg):
        self.velocity=(math.hypot(msg.twist.twist.linear.x,msg.twist.twist.linear.y),abs(msg.twist.twist.angular.z));self.odom_at=self.now()
        q=msg.pose.pose.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        if self.phase=='SPINNING' and self.last_yaw is not None:
            self.odom_angle+=math.atan2(math.sin(yaw-self.last_yaw),math.cos(yaw-self.last_yaw))
        self.last_yaw=yaw

    def goal(self,msg,dispatch):
        p=msg.pose.position;q=msg.pose.orientation
        values=[p.x,p.y,p.z,q.x,q.y,q.z,q.w]
        if not msg.header.frame_id or not all(math.isfinite(v) for v in values) or sum(v*v for v in [q.x,q.y,q.z,q.w])<.5:
            self.get_logger().warning('Ignoring invalid navigation pose');return
        if dispatch and self.nav_pending is not None and self.phase not in ('STOPPING_NAV','APPROACHING','VERIFYING_DISTANCE','SPINNING','CANCELLING'):
            self.get_logger().warning('Owned navigation busy; finish or stop before sending another goal');return
        self.target_revision+=1;self.navigation_attempts=0
        self.target=copy.deepcopy(msg)
        if self.phase not in ('STOPPING_NAV','APPROACHING','VERIFYING_DISTANCE','SPINNING','CANCELLING'):
            self.reacted=False;self.gate.count=0;self.locked_box=None;self.box_observation=None
        if self.phase in ('STOPPING_NAV','APPROACHING','VERIFYING_DISTANCE','SPINNING','CANCELLING'):
            self.get_logger().info('New goal saved for after current reaction');return
        if dispatch:
            if self.nav_pending is not None:
                self.get_logger().warning('Owned navigation busy; finish or stop before sending another goal');return
            self.navigate(False)
        else:self.report('MOVING','RViz goal remembered; monitoring YOLO during navigation')

    def navigate(self,resume):
        if self.target is None:return
        goal=NavigateToPose.Goal();goal.pose=copy.deepcopy(self.target)
        goal.pose.header.stamp=self.get_clock().now().to_msg()
        self.nav_role='mission'
        self.nav_revision=self.target_revision
        self.nav_pending=self.navigation.send_goal_async(goal)
        self.nav_handle=self.nav_result=self.nav_cancel=None
        self.report('RESUMING' if resume else 'MOVING','Returning to interrupted goal' if resume else 'Navigating to requested goal')

    def pump_navigation(self):
        if self.nav_pending is None:return
        if not self.nav_pending.done():return
        if self.nav_handle is None:
            try:
                self.nav_handle=self.nav_pending.result()
                if not self.nav_handle.accepted:
                    self.nav_pending=None;self.report('FAILURE','Navigation goal rejected');return
                self.nav_result=self.nav_handle.get_result_async()
            except Exception as exc:
                self.nav_pending=None;self.report('FAILURE',str(exc));return
        if self.phase in ('STOPPING_NAV','STOPPED') and self.nav_cancel is None:
            self.nav_cancel=self.nav_handle.cancel_goal_async()
        if not self.nav_result.done():return
        try:
            response=self.nav_result.result();status=response.status
            success=status==GoalStatus.STATUS_SUCCEEDED
            error=getattr(response.result,'error_msg','') if hasattr(response,'result') else ''
        except Exception as exc:success=False;status=GoalStatus.STATUS_ABORTED;error=str(exc)
        self.nav_pending=self.nav_handle=self.nav_result=self.nav_cancel=None
        if self.phase=='APPROACHING':
            if success:self.begin_stop('verify','Approach reached; checking fresh working distance')
            else:self.report('FAILURE','Box approach failed: '+error)
            return
        if self.phase in ('MOVING','RESUMING') and self.nav_revision==self.target_revision:
            if success:
                self.target=None;self.report('ARRIVED','Navigation goal reached')
            elif status==GoalStatus.STATUS_CANCELED:self.report('STOPPED','Navigation was cancelled')
            elif self.navigation_attempts<self.navigation_retries:
                self.navigation_attempts+=1;self.retry_at=self.now()+2.
                self.report('REPLANNING','Navigation interrupted; retrying saved goal once: '+error)
            else:self.report('FAILURE','Navigation failed after retry: '+error)

    def observations(self,data):
        stamp=data['stamp']['sec']+data['stamp']['nanosec']/1e9
        if not 0<=self.now()-stamp<=1.:return []
        observation_time=rclpy.time.Time(seconds=data['stamp']['sec'],nanoseconds=data['stamp']['nanosec'])
        frame=data['frame_id'];out=[]
        for detection in data['detections']:
            point=detection.get('position_camera')
            threshold=.5 if self.locked_box is not None else self.gate.confidence
            if detection.get('class_name')!='box' or float(detection['confidence'])<threshold or point is None:continue
            xyz=[point['x'],point['y'],point['z']]
            if not all(math.isfinite(v) for v in xyz) or not .15<=point['z']<=3.:continue
            try:
                base=transform_xyz(self.tf_buffer.lookup_transform('base_link',frame,observation_time).transform,xyz)
                world=transform_xyz(self.tf_buffer.lookup_transform('map',frame,observation_time).transform,xyz)
            except (TransformException,ValueError):continue
            if base[0]<=0:continue
            out.append({'stamp':stamp,'map':world,'base':base,'detection':detection})
        return out

    def current_gap(self):
        if self.box_observation is None or self.now()-self.box_observation['stamp']>.8:return None
        return math.hypot(*self.box_observation['base'][:2])-self.front_offset

    def detect(self,msg):
        if self.reacted and self.phase not in ('APPROACHING','VERIFYING_DISTANCE','STOPPING_NAV'):return
        try:
            data=json.loads(msg.data);observations=self.observations(data)
        except (ValueError,KeyError,TypeError,OverflowError):return
        if self.locked_box is not None:
            observations=[o for o in observations if math.dist(o['map'][:2],self.locked_box[:2])<.5]
        if observations:self.box_observation=min(observations,key=lambda o:math.hypot(*o['base'][:2]))
        if self.phase not in ('ARMED','MOVING','RESUMING') or self.reacted or self.target is None or not any(self.nav_busy.values()):return
        # A sliver at the horizontal image edge is not enough to initiate approach.
        candidates=[]
        for o in observations:
            x0,y0,x1,y1=o['detection']['xyxy'];center=(x0+x1)/2
            if 64<center<576 and (x1-x0)*(y1-y0)>=640*480*.008:candidates.append(o)
        gated=dict(data);gated['detections']=[o['detection'] for o in candidates]
        try:trigger=self.gate.accept(gated,self.now())
        except (ValueError,KeyError,TypeError,OverflowError):return
        if trigger:
            self.box_observation=max(candidates,key=lambda o:o['detection']['confidence'])
            self.locked_box=list(self.box_observation['map']);self.reacted=True;self.approach_attempts=0
            self.angle=self.odom_angle=0.
            self.begin_stop('approach','Box confirmed with depth; stopping mission to approach')

    def begin_stop(self,after,detail):
        self.after_stop=after;self.started=self.now();self.cancel_futures={};self.stopped_since=None
        self.report('STOPPING_NAV',detail)

    def start_approach(self):
        if self.box_observation is None or self.now()-self.box_observation['stamp']>1.:
            self.report('FAILURE','No fresh RGB-D box position; holding stopped');return
        try:t=self.tf_buffer.lookup_transform('map','base_link',rclpy.time.Time()).transform.translation
        except TransformException:self.report('FAILURE','No robot map pose for approach');return
        surface=self.box_observation['map'];yaw=math.atan2(surface[1]-t.y,surface[0]-t.x)
        offset=self.work_distance+self.front_offset
        pose=PoseStamped();pose.header.frame_id='map';pose.header.stamp=self.get_clock().now().to_msg()
        pose.pose.position.x=surface[0]-offset*math.cos(yaw);pose.pose.position.y=surface[1]-offset*math.sin(yaw)
        pose.pose.orientation.z=math.sin(yaw/2);pose.pose.orientation.w=math.cos(yaw/2)
        goal=NavigateToPose.Goal();goal.pose=pose
        self.nav_role='approach';self.nav_revision=self.target_revision
        self.nav_pending=self.navigation.send_goal_async(goal)
        self.nav_handle=self.nav_result=self.nav_cancel=None
        self.approach_attempts+=1;self.approach_started=self.now()
        self.report('APPROACHING',f'Approaching box; target front clearance {self.work_distance:.2f}m')

    def verify_distance(self):
        gap=self.current_gap()
        if gap is None:
            if self.now()-self.started>4:self.report('FAILURE','No fresh depth at work pose; not spinning')
            return
        bearing=abs(math.atan2(self.box_observation['base'][1],self.box_observation['base'][0]))
        if abs(gap-self.work_distance)<=self.distance_tolerance and bearing<=.20:
            self.after_stop='spin';self.begin_spin()
        elif gap<self.work_distance-self.distance_tolerance:
            self.report('FAILURE',f'Box too close ({gap:.2f}m); holding stopped')
        elif self.approach_attempts<2:self.start_approach()
        else:self.report('FAILURE',f'Work pose not verified: gap={gap:.2f}m bearing={bearing:.2f}; not spinning')

    def begin_spin(self):
        if not self.spin.server_is_ready():return
        goal=Spin.Goal();goal.target_yaw=2*math.pi;goal.time_allowance.sec=45
        self.spin_pending=self.spin.send_goal_async(goal,feedback_callback=self.feedback)
        self.spin_handle=self.spin_result=self.spin_cancel=None;self.reason=None;self.started=self.now()
        self.report('SPINNING',f'Work distance verified ({self.current_gap():.2f}m); rotating 360 degrees')

    def reset(self,request,response):
        if self.phase in ('STOPPING_NAV','APPROACHING','VERIFYING_DISTANCE','SPINNING','CANCELLING'):
            response.success=False;response.message='Wait for stop/spin completion'
        else:
            self.gate.count=0;self.reacted=False;self.locked_box=None;self.report('MOVING' if self.target else 'ARMED','Rearmed for next box detection')
            response.success=True;response.message='Rearmed'
        return response

    def stop(self,request,response):
        if self.spin_pending is not None:
            self.reason='stop requested';self.report('CANCELLING',self.reason)
        else:
            for c in self.cancel_clients.values():
                if c.service_is_ready():c.call_async(CancelGoal.Request())
            self.report('STOPPED','Detection trigger disabled; navigation cancellation requested')
        response.success=True;response.message='Stop requested'
        return response

    def feedback(self,msg):self.angle=float(msg.feedback.angular_distance_traveled)

    def tick(self):
        now=self.now()
        self.pump_navigation()
        if self.phase=='REPLANNING' and now>=self.retry_at and not any(self.nav_busy.values()) and self.navigation.server_is_ready():self.navigate(True)
        if self.phase=='APPROACHING':
            if now-self.approach_started>self.approach_timeout:
                self.begin_stop('failure','Approach timeout; stopping')
            elif now-self.box_observation['stamp']>6:
                self.begin_stop('failure','Box depth lost during approach; stopping')
            elif now-self.last_status>.3:self.publish_status('Approaching box')
        if self.phase=='VERIFYING_DISTANCE':self.verify_distance()
        if self.phase=='STOPPING_NAV':
            for key,c in self.cancel_clients.items():
                if key not in self.cancel_futures and c.service_is_ready():self.cancel_futures[key]=c.call_async(CancelGoal.Request())
            complete=len(self.cancel_futures)==len(self.cancel_clients) and all(f.done() for f in self.cancel_futures.values())
            if complete:
                try:
                    for f in self.cancel_futures.values():
                        if f.result().return_code not in (0,2):
                            self.report('FAILURE','Navigation cancellation rejected');return
                except Exception as exc:self.report('FAILURE',str(exc));return
            stationary=self.velocity is not None and now-self.odom_at<.5 and self.velocity[0]<.015 and self.velocity[1]<.04
            if complete and self.nav_pending is None and not any(self.nav_busy.values()) and stationary:
                if self.stopped_since is None:self.stopped_since=now
                if now-self.stopped_since>=.6:
                    if self.after_stop=='failure':self.report('FAILURE','Approach could not be completed; stopped')
                    elif self.after_stop in ('approach','verify'):
                        self.started=now;self.report('VERIFYING_DISTANCE','Stopped; verifying RGB-D working distance')
                        self.verify_distance()
                    else:self.begin_spin()
            else:self.stopped_since=None
            if self.phase=='STOPPING_NAV' and now-self.started>12:self.report('FAILURE','Could not confirm stopped navigation and fresh odometry')
        if self.phase not in ('SPINNING','CANCELLING'):return
        if self.phase=='SPINNING' and now-self.last_status>.3:self.publish_status('Rotating in place')
        if now-self.started>50 and self.reason is None:
            self.reason='spin timeout';self.report('CANCELLING',self.reason)
        if not self.spin_pending.done():return
        if self.spin_handle is None:
            try:
                self.spin_handle=self.spin_pending.result()
                if not self.spin_handle.accepted:
                    self.spin_pending=None;self.report('FAILURE','Spin goal rejected');return
                self.spin_result=self.spin_handle.get_result_async()
            except Exception as exc:
                self.spin_pending=None;self.report('FAILURE',str(exc));return
        if self.reason and self.spin_cancel is None:self.spin_cancel=self.spin_handle.cancel_goal_async()
        if not self.spin_result.done():return
        try:
            result=self.spin_result.result()
            success=result.status==GoalStatus.STATUS_SUCCEEDED and result.result.error_code==0
            detail=result.result.error_msg
        except Exception as exc:success=False;detail=str(exc)
        self.spin_pending=self.spin_handle=self.spin_result=self.spin_cancel=None
        if self.reason:self.report('STOPPED',self.reason+'; action terminated')
        elif success and abs(self.odom_angle)<math.radians(350):
            self.report('FAILURE','Spin action ended without a measured full turn; not resuming')
        elif success:
            self.report('DONE','360 degree spin complete')
            if self.target is not None:self.navigate(True)
        else:self.report('FAILURE','Spin failed: '+detail)


def main(args=None):
    rclpy.init(args=args);n=ObjectSpin()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:
        if rclpy.ok():
            n.stop(None,Trigger.Response());deadline=time.monotonic()+5
            while (n.spin_pending is not None or n.nav_pending is not None) and rclpy.ok() and time.monotonic()<deadline:rclpy.spin_once(n,timeout_sec=.05)
            if n.spin_pending is not None:n.get_logger().error('Spin cancellation not confirmed')
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
