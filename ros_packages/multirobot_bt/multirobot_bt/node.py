"""ROS adapter: execute externally allocated tasks; never allocate or plan routes."""
import asyncio,json,math,time,signal
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import String,Bool
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped,Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from lifecycle_msgs.srv import GetState
from tf2_ros import Buffer,TransformListener
from tf2_geometry_msgs import do_transform_pose_stamped
from .mission import Mission,ROBOTS
from .bt import build,Status
from .region_defaults import patrol_from_factory

class MultiBT(Node):
    def __init__(self):
        super().__init__('multirobot_bt');self.mission=Mission();self.tree=build()
        self.declare_parameter('factory_regions_file','')
        region_file=self.get_parameter('factory_regions_file').value
        if region_file:
            self.mission.assign_patrol(patrol_from_factory(region_file))
            self.get_logger().info('Loaded main regions: robot1=1, robot2=2, robot3=3; waiting for start')
        self.declare_parameter('frame_id','map');self.frame=self.get_parameter('frame_id').value
        self.declare_parameter('timeout_sec',900.);self.timeout=float(self.get_parameter('timeout_sec').value)
        self.tf=Buffer(node=self);self.listener=TransformListener(self.tf,self)
        self.nav={r:ActionClient(self,NavigateToPose,f'/{r}/navigate_to_pose') for r in ROBOTS}
        self.handles={};self.canceling=set();self.pending={};self.results={};self.odoms={};self.observed={};self.still_since=None;self.phase_since=time.monotonic();self.last_phase='WAITING'
        self.zeros={r:self.create_publisher(Twist,f'/{r}/cmd_vel',10) for r in ROBOTS}
        q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status=self.create_publisher(String,'/multi_bt/status',q)
        self.stop_pub=self.create_publisher(Bool,'/multi_bt/emergency_stop',q)
        self.stop_pub.publish(Bool(data=True))
        self.target_pub=self.create_publisher(PoseStamped,'/multi_bt/target',q)
        self.create_subscription(String,'/multi_bt/patrol_tasks',self.patrol_tasks,q)
        self.create_subscription(String,'/multi_bt/rescue_tasks',self.rescue_tasks,10)
        for r in ROBOTS:
            self.create_subscription(Bool,f'/{r}/perception/is_person',lambda m,r=r:self.detect(r,m),10)
            self.create_subscription(PoseStamped,f'/{r}/perception/person_pose',lambda m,r=r:self.observe(r,m),10)
            self.create_subscription(Odometry,f'/{r}/odom',lambda m,r=r:self.odoms.update({r:(m,time.monotonic())}),qos_profile_sensor_data)
        self.create_service(Trigger,'/multi_bt/start',self.start)
        self.create_service(Trigger,'/multi_bt/stop',self.stop)
        self.active={r:False for r in ROBOTS};self.lifecycle_pending={}
        self.lifecycle={r:self.create_client(GetState,f'/{r}/bt_navigator/get_state') for r in ROBOTS}
        self.create_timer(1.,self.check_navigation)
        self.create_timer(.1,self.tick)
        self.get_logger().info("BT ready: waiting for external patrol tasks and /multi_bt/start")
    def reject(self,e):self.get_logger().warning('Task rejected: '+str(e))
    def patrol_tasks(self,msg):
        try:self.mission.assign_patrol(json.loads(msg.data))
        except (ValueError,TypeError,KeyError) as e:self.reject(e)
    def rescue_tasks(self,msg):
        try:self.mission.assign_rescue(json.loads(msg.data))
        except (ValueError,TypeError,KeyError) as e:self.reject(e)
    def check_navigation(self):
        for r,client in self.lifecycle.items():
            if r in self.lifecycle_pending:continue
            if not client.service_is_ready():self.active[r]=False;continue
            future=client.call_async(GetState.Request());self.lifecycle_pending[r]=future
            def received(done,r=r):
                self.lifecycle_pending.pop(r,None)
                try:self.active[r]=done.result().current_state.id==3
                except Exception:self.active[r]=False
            future.add_done_callback(received)
    def fresh_odometry(self):return all(r in self.odoms and time.monotonic()-self.odoms[r][1]<2. for r in ROBOTS)
    def start(self,req,res):
        try:
            if not all(self.active.values()) or self.handles or self.pending or not self.fresh_odometry() or not all(c.server_is_ready() for c in self.nav.values()):raise ValueError('Wait for canceled goals, all odometry and Nav2 servers')
            self.mission.start();self.tree=build();self.results={};self.observed={};self.still_since=None
            res.success=True;res.message='Assigned patrol started'
        except ValueError as e:res.success=False;res.message=str(e)
        return res
    def stop(self,req,res):
        self.fail('Stopped by operator','STOPPED');res.success=True;res.message='Canceling all goals; wait for stationary odometry';return res
    def fail(self,reason,phase='ERROR'):
        self.get_logger().warning(phase+': '+reason)
        self.mission.fail(reason,phase);self.cancel_all()
    def cancel_all(self):
        self.stop_pub.publish(Bool(data=True))
        for r,h in tuple(self.handles.items()):
            if r not in self.canceling:self.canceling.add(r);h.cancel_goal_async()
        for p in self.zeros.values():p.publish(Twist())
    def detect(self,r,msg):
        if msg.data and self.mission.detect(r):self.still_since=None;self.cancel_all()
    def observe(self,r,msg):
        now=self.get_clock().now().nanoseconds/1e9;stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        if not msg.header.frame_id or not 0<=now-stamp<=2:return
        try:
            mapped=msg if msg.header.frame_id==self.frame else do_transform_pose_stamped(msg,self.tf.lookup_transform(self.frame,msg.header.frame_id,Time.from_msg(msg.header.stamp)))
            if all(math.isfinite(v) for v in (mapped.pose.position.x,mapped.pose.position.y)):self.observed[r]=(mapped,time.monotonic())
        except Exception:pass
    def send(self,r,point):
        g=NavigateToPose.Goal();g.pose.header.frame_id=self.frame;g.pose.header.stamp=self.get_clock().now().to_msg()
        p=g.pose.pose;p.position.x=point[0];p.position.y=point[1];p.orientation.z=math.sin(point[2]/2);p.orientation.w=math.cos(point[2]/2)
        self.results.pop(r,None);f=self.nav[r].send_goal_async(g);self.pending[r]=f
        def accepted(f):
            self.pending.pop(r,None)
            try:
                h=f.result()
                if not h.accepted:self.results[r]=GoalStatus.STATUS_ABORTED;return
                self.handles[r]=h
                if self.mission.phase not in ('PATROL','GATHER'):self.canceling.add(r);h.cancel_goal_async()
                h.get_result_async().add_done_callback(lambda done:self.finished(r,done))
            except Exception:self.results[r]=GoalStatus.STATUS_ABORTED
        f.add_done_callback(accepted)
    def finished(self,r,f):
        self.handles.pop(r,None);self.canceling.discard(r)
        try:self.results[r]=f.result().status
        except Exception:self.results[r]=GoalStatus.STATUS_ABORTED
    def stop_all(self):
        m=self.mission
        if m.phase in ('ERROR','STOPPED'):self.cancel_all();return Status.FAILURE
        if m.phase!='STOPPING':return Status.SUCCESS
        self.cancel_all()
        stationary=self.fresh_odometry() and not self.handles and not self.pending
        if stationary:
            stationary=all(abs(self.odoms[r][0].twist.twist.linear.x)<.025 and abs(self.odoms[r][0].twist.twist.angular.z)<.04 for r in ROBOTS)
        if not stationary:self.still_since=None;return Status.RUNNING
        if self.still_since is None:self.still_since=time.monotonic()
        obs=self.observed.get(m.source)
        if time.monotonic()-self.still_since<.8 or not obs or time.monotonic()-obs[1]>3:return Status.RUNNING
        pose=obs[0];p=pose.pose.position;m.stopped((p.x,p.y));self.target_pub.publish(pose);self.results={}
        return Status.SUCCESS
    def wait_assignment(self):
        if self.mission.phase in ('ERROR','STOPPED'):return Status.FAILURE
        return Status.RUNNING if self.mission.phase=='WAIT_RESCUE_TASKS' else Status.SUCCESS
    def gather(self):
        m=self.mission
        if m.phase=='COMPLETE':return Status.SUCCESS
        if m.phase!='GATHER':return Status.FAILURE
        if len(m.arrived)==3:m.phase='COMPLETE';self.cancel_all();return Status.SUCCESS
        r=m.order[len(m.arrived)]
        if r in self.pending or r in self.handles:return Status.RUNNING
        result=self.results.pop(r,None)
        if result is not None:
            if result!=GoalStatus.STATUS_SUCCEEDED:self.fail(r+': assigned rescue goal failed');return Status.FAILURE
            try:
                t=self.tf.lookup_transform(self.frame,r+'/base_link',Time())
                age=(self.get_clock().now().nanoseconds-(t.header.stamp.sec*10**9+t.header.stamp.nanosec))/1e9
                if not 0<=age<3:raise ValueError('stale arrival TF')
                if math.dist((t.transform.translation.x,t.transform.translation.y),m.slots[r][:2])>.4:raise ValueError('arrival mismatch')
            except Exception as e:self.fail(r+': arrival verification failed: '+str(e));return Status.FAILURE
            m.arrived.append(r);return Status.RUNNING
        self.send(r,m.slots[r]);return Status.RUNNING
    def patrol(self,r):
        m=self.mission
        if m.phase!='PATROL':return Status.FAILURE
        if r in self.pending or r in self.handles:return Status.RUNNING
        result=self.results.pop(r,None)
        if result is not None:
            if result!=GoalStatus.STATUS_SUCCEEDED:self.fail(r+': assigned patrol goal failed');return Status.FAILURE
            m.indices[r]=(m.indices[r]+1)%len(m.routes[r])
        self.send(r,m.routes[r][m.indices[r]]);return Status.RUNNING
    def tick(self):
        m=self.mission
        if m.phase!=self.last_phase:self.phase_since=time.monotonic();self.last_phase=m.phase
        if m.phase in ('PATROL','STOPPING','WAIT_RESCUE_TASKS','GATHER'):
            if not self.fresh_odometry() or not all(self.active.values()):self.fail('Robot odometry or active Nav2 unavailable')
            elif m.phase!='PATROL' and time.monotonic()-self.phase_since>self.timeout:self.fail('Mission phase timed out: '+m.phase)
        self.stop_pub.publish(Bool(data=m.phase not in ('PATROL','GATHER')))
        if m.phase!='WAITING':
            try:asyncio.run(self.tree.run(self,{}))
            except Exception as e:self.fail(str(e))
        payload=m.state();payload['route_counts']={r:len(v) for r,v in (m.routes or {}).items()}
        self.status.publish(String(data=json.dumps(payload)))

def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=MultiBT()
    requested=[False]
    signal.signal(signal.SIGINT,lambda *_:requested.__setitem__(0,True))
    try:
        while rclpy.ok() and not requested[0]:rclpy.spin_once(node,timeout_sec=.1)
    except KeyboardInterrupt:pass
    finally:
        signal.signal(signal.SIGINT,signal.SIG_IGN)
        if rclpy.ok():
            node.fail('Node shutdown','STOPPED')
            # Allow cancellation acknowledgments to arrive before destroying clients.
            end=time.monotonic()+2
            while (node.pending or node.handles) and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
