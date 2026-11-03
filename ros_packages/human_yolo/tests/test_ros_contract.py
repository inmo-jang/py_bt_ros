"""Real ROS messages with a deterministic inference double; no simulator/assets."""
import json
import time
from types import SimpleNamespace as S
import numpy as np
import pytest
rclpy=pytest.importorskip('rclpy')
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PointStamped,PoseStamped


class FakeModel:
    names={0:'rescue_dummy'}
    def predict(self,pixels,**kwargs):
        boxes=[]
        if pixels[0,0,0]:
            boxes=[S(cls=np.array([0]),conf=np.array([.9]),xyxy=np.array([[16.,8.,48.,56.]]))]
        return [S(names=self.names,boxes=boxes)]


def test_detection_position_depth_failure_and_camera_timeout(monkeypatch):
    import ultralytics
    monkeypatch.setattr(ultralytics,'YOLO',lambda _:FakeModel())
    from human_yolo.detector import HumanDetector
    rclpy.init();detector=HumanDetector();probe=Node('human_yolo_contract_probe');received={};points=[]
    subs=[probe.create_subscription(Bool,'/perception/is_person',lambda m:received.update(person=m.data),10),
          probe.create_subscription(PoseStamped,'/perception/person_pose',lambda m:received.update(pose=m),10),
          probe.create_subscription(Bool,'/human/detected',lambda m:received.update(detected=m.data),10),
          probe.create_subscription(Bool,'/human/position_valid',lambda m:received.update(valid=m.data),10),
          probe.create_subscription(PointStamped,'/human/position',points.append,10),
          probe.create_subscription(String,'/human/detections',lambda m:received.update(details=json.loads(m.data)),10)]
    pubs=[probe.create_publisher(Image,'/camera/color/image_raw',10),
          probe.create_publisher(Image,'/camera/depth/image_raw',10),
          probe.create_publisher(CameraInfo,'/camera/color/camera_info',10)]
    def wait(predicate,seconds=4.):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            rclpy.spin_once(detector,timeout_sec=.01);rclpy.spin_once(probe,timeout_sec=.01)
            if predicate():return
        assert predicate(),received
    def send(positive=True,valid_depth=True):
        stamp=probe.get_clock().now().to_msg();frame='camera_color_optical_frame'
        i=CameraInfo();i.header.stamp=stamp;i.header.frame_id=frame;i.width=64;i.height=64
        i.k=[50.,0.,32.,0.,50.,32.,0.,0.,1.]
        d=Image();d.header=i.header;d.width=64;d.height=64;d.encoding='16UC1';d.step=128
        d.data=np.full((64,64),1200 if valid_depth else 0,dtype='<u2').tobytes()
        im=Image();im.header=i.header;im.width=64;im.height=64;im.encoding='rgb8';im.step=192
        im.data=np.full((64,64,3),255 if positive else 0,dtype=np.uint8).tobytes()
        pubs[2].publish(i);pubs[1].publish(d)
        for _ in range(3):rclpy.spin_once(detector,timeout_sec=.01)
        pubs[0].publish(im)
        wait(lambda:received.get('details',{}).get('stamp')=={'sec':stamp.sec,'nanosec':stamp.nanosec})
    try:
        wait(lambda:all(p.get_subscription_count() for p in pubs))
        send();wait(lambda:received.get('detected') and received.get('valid') and bool(points))
        assert points[-1].header.frame_id=='camera_color_optical_frame'
        assert points[-1].point.z==pytest.approx(1.2)
        wait(lambda:received.get('person') is True and 'pose' in received)
        assert received['pose'].pose.position.z==pytest.approx(1.2)
        send(valid_depth=False);wait(lambda:received.get('detected') and received.get('valid') is False)
        wait(lambda:received.get('person') is False)
        count=len(points)
        send(positive=False);wait(lambda:received.get('detected') is False and received.get('valid') is False)
        assert len(points)==count
        send();wait(lambda:received.get('detected') and received.get('valid'))
        wait(lambda:received.get('detected') is False and received.get('valid') is False,3.)
    finally:
        detector.close();detector.destroy_node();probe.destroy_node();rclpy.shutdown()
