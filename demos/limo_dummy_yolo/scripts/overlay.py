"""Display inference status inside Webots; no ground-truth object access."""
import json,time
from controller import Supervisor
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
s=Supervisor();dt=int(s.getBasicTimeStep());rclpy.init();n=Node('webots_yolo_status');latest={};updated=0.;reaction={}
def receive(m):
 global latest,updated
 latest=json.loads(m.data);updated=time.monotonic()
sub=n.create_subscription(String,'/yolo/detections',receive,10)
def reaction_status(m):
 global reaction
 reaction=json.loads(m.data)
reaction_sub=n.create_subscription(String,'/object_spin/status',reaction_status,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
try:
 while s.step(dt)!=-1 and rclpy.ok():
  rclpy.spin_once(n,timeout_sec=0)
  detections=latest.get('detections',[])
  if time.monotonic()-updated>3:text='YOLO: waiting for camera';color=0xeeee00
  elif detections:text=f'YOLO HUMAN: {max(d["confidence"] for d in detections):.2f}';color=0x00ee44
  else:text='YOLO: no human in view';color=0xffffff
  s.setLabel(42,text,.01,.28,.045,color,0.,'Arial')
  phase=reaction.get('state','WAITING')
  progress=abs(reaction.get('measured_angle_radians',0))*180/3.141592653589793
  text=f'ROTATING: {progress:.0f} / 360 deg' if phase=='SPINNING' else f'ROBOT: {phase}'
  gap=reaction.get('front_gap_m')
  if phase in ('APPROACHING','VERIFYING_DISTANCE') and gap is not None:text=f'BOX APPROACH: {gap:.2f} m front gap'
  s.setLabel(43,text,.01,.34,.045,0xffcc00,0.,'Arial')
except KeyboardInterrupt:pass
finally:
 s.setLabel(42,'',0,0,.01,0,0)
 s.setLabel(43,'',0,0,.01,0,0)
 n.destroy_node()
 if rclpy.ok():rclpy.shutdown()
 s.getSelf().remove()
