"""Live ROS 2 inference using only RGB pixels (no simulator ground truth)."""
import json,threading,time,sys
# ROS dependencies provided by Ubuntu, without modifying the user YOLO venv.
sys.path.append('/usr/lib/python3/dist-packages')
from pathlib import Path
from collections import deque
from webots_ros2_limo.depth_geometry import sample_box_depth
import numpy as np
import torch
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from ultralytics import YOLO

ROOT=Path(__file__).resolve().parents[1]

class BoxDetector(Node):
    def __init__(self):
        super().__init__('limo_yolo_box')
        self.declare_parameter('model',str(ROOT/'models/best.pt'))
        self.declare_parameter('confidence',.50)
        self.declare_parameter('image_size',320)
        # NNPACK is unsupported on some CPUs and otherwise logs every convolution.
        torch.backends.nnpack.set_flags(False)
        self.model=YOLO(self.get_parameter('model').value);torch.set_num_threads(2)
        self.sub=self.create_subscription(Image,'/camera/color/image_raw',self.receive,qos_profile_sensor_data)
        self.depths=deque(maxlen=30);self.camera_info=None
        self.depth_sub=self.create_subscription(Image,'/camera/depth/image_raw',self.receive_depth,qos_profile_sensor_data)
        self.info_sub=self.create_subscription(CameraInfo,'/camera/color/camera_info',self.receive_info,10)
        self.pub=self.create_publisher(String,'/yolo/detections',10)
        self.image_pub=self.create_publisher(Image,'/yolo/annotated_image',1)
        self.latest=None;self.lock=threading.Lock();self.stop_event=threading.Event()
        self.thread=threading.Thread(target=self.worker,daemon=True);self.thread.start()
        self.previous=False
        self.get_logger().info('YOLO dummy detector ready: RGB inference + aligned depth; no velocity commands')

    def receive(self,msg):
        with self.lock:self.latest=msg

    def receive_depth(self,msg):
        with self.lock:self.depths.append(msg)

    def receive_info(self,msg):
        self.camera_info=msg

    def worker(self):
        while not self.stop_event.is_set():
            with self.lock:msg=self.latest;self.latest=None
            if msg is None:self.stop_event.wait(.02);continue
            try:
                if msg.encoding not in ('rgb8','bgr8'):continue
                rgb=np.frombuffer(msg.data,np.uint8).reshape(msg.height,msg.step)[:,:msg.width*3].reshape(msg.height,msg.width,3)
                bgr=np.ascontiguousarray(rgb[:,:,::-1] if msg.encoding=='rgb8' else rgb)
                start=time.monotonic()
                r=self.model.predict(bgr,imgsz=self.get_parameter('image_size').value,conf=self.get_parameter('confidence').value,device='cpu',verbose=False)[0]
                detections=[dict(class_id=int(b.cls.item()),class_name=r.names[int(b.cls.item())],confidence=float(b.conf.item()),xyxy=b.xyxy[0].tolist()) for b in r.boxes]
                with self.lock:depths=list(self.depths)
                stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
                if depths and self.camera_info is not None:
                    depth=min(depths,key=lambda d:abs(d.header.stamp.sec+d.header.stamp.nanosec/1e9-stamp))
                    delta=abs(depth.header.stamp.sec+depth.header.stamp.nanosec/1e9-stamp)
                    if delta<=.10 and depth.header.frame_id==msg.header.frame_id:
                        for detection in detections:
                            point=sample_box_depth(depth,detection['xyxy'],self.camera_info)
                            if point is not None:detection['position_camera']=point;detection['depth_dt']=delta
                payload={'stamp':{'sec':msg.header.stamp.sec,'nanosec':msg.header.stamp.nanosec},'frame_id':msg.header.frame_id,'detections':detections,'inference_ms':round((time.monotonic()-start)*1000,1)}
                self.pub.publish(String(data=json.dumps(payload)))
                annotated=r.plot();out=Image();out.header=msg.header;out.height=msg.height;out.width=msg.width;out.encoding='bgr8';out.step=msg.width*3;out.data=annotated.tobytes();self.image_pub.publish(out)
                detected=bool(detections)
                if detected and not self.previous:
                    self.get_logger().info(f'DUMMY DETECTED: confidence={max(d["confidence"] for d in detections):.3f}')
                self.previous=detected
            except Exception as exc:
                self.get_logger().error(str(exc));self.stop_event.wait(.5)
            self.stop_event.wait(.08)

    def close(self):
        self.stop_event.set();self.thread.join(timeout=10)


def main():
    rclpy.init();n=BoxDetector()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:
        n.close();n.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
