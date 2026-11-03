"""Human YOLO inference ROS node. Publishes perception only; no motion commands."""
from collections import deque
import json
from pathlib import Path
import threading
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import Bool, String
from .geometry import perception_result, seconds


def default_model():
    source=Path(__file__).resolve().parents[1]/'models/best.pt'
    if source.is_file():
        return str(source)
    from ament_index_python.packages import get_package_share_directory
    return str(Path(get_package_share_directory('human_yolo'))/'models/best.pt')


class HumanDetector(Node):
    def __init__(self):
        super().__init__('human_yolo')
        defaults={'model_path':default_model(),'target_class':'rescue_dummy','confidence':.5,
                  'image_size':320,'device':'cpu','cpu_threads':2,
                  'rgb_topic':'/camera/color/image_raw','depth_topic':'/camera/depth/image_raw',
                  'camera_info_topic':'/camera/color/camera_info','sync_tolerance':.10,
                  'input_timeout':1.,'min_depth':.15,'max_depth':3.}
        for key,value in defaults.items():
            self.declare_parameter(key,value)
        self.settings={key:self.get_parameter(key).value for key in defaults}
        cfg=self.settings
        if not 0<cfg['confidence']<=1 or cfg['image_size']<32 or cfg['cpu_threads']<1:
            raise ValueError('Invalid inference parameters')
        if not 0<=cfg['sync_tolerance']<=cfg['input_timeout'] or not 0<cfg['min_depth']<cfg['max_depth']:
            raise ValueError('Invalid depth/time parameters')
        if cfg['input_timeout']<=0 or not Path(cfg['model_path']).is_file():
            raise ValueError('Missing model or invalid input timeout')
        # Lazy import keeps geometry/tests independent of the inference runtime.
        import torch
        from ultralytics import YOLO
        torch.set_num_threads(cfg['cpu_threads'])
        torch.backends.nnpack.set_flags(False)
        self.model=YOLO(cfg['model_path'])
        if cfg['target_class'] not in self.model.names.values():
            raise ValueError('target_class does not exist in the supplied model')
        self.lock=threading.Lock();self.stop_event=threading.Event()
        self.latest=None;self.depths=deque(maxlen=40);self.info=None
        self.last_input=None
        self.detected_pub=self.create_publisher(Bool,'/human/detected',10)
        self.valid_pub=self.create_publisher(Bool,'/human/position_valid',10)
        self.position_pub=self.create_publisher(PointStamped,'/human/position',10)
        self.person_pub=self.create_publisher(Bool,'/perception/is_person',10)
        self.person_pose_pub=self.create_publisher(PoseStamped,'/perception/person_pose',10)
        self.details_pub=self.create_publisher(String,'/human/detections',10)
        self.rgb_sub=self.create_subscription(Image,cfg['rgb_topic'],self.receive_rgb,qos_profile_sensor_data)
        self.depth_sub=self.create_subscription(Image,cfg['depth_topic'],self.receive_depth,qos_profile_sensor_data)
        self.info_sub=self.create_subscription(CameraInfo,cfg['camera_info_topic'],self.receive_info,qos_profile_sensor_data)
        self.watchdog=self.create_timer(.2,self.expire)
        self.worker=threading.Thread(target=self.infer,daemon=True);self.worker.start()
        self.get_logger().info(f"Perception ready: {cfg['target_class']} -> /human/*")

    def receive_rgb(self,msg):
        with self.lock:
            self.latest=msg

    def receive_depth(self,msg):
        with self.lock:
            self.depths.append(msg)

    def receive_info(self,msg):
        with self.lock:
            self.info=msg

    def fresh(self,msg):
        age=self.get_clock().now().nanoseconds/1e9-seconds(msg.header.stamp)
        return 0<=age<=self.settings['input_timeout']

    def clear(self):
        if not self.context.ok():return
        self.detected_pub.publish(Bool(data=False));self.valid_pub.publish(Bool(data=False))
        self.person_pub.publish(Bool(data=False))

    def expire(self):
        with self.lock:
            if self.last_input is None or not self.fresh(self.last_input):
                self.clear()

    def infer(self):
        cfg=self.settings
        while not self.stop_event.is_set():
            with self.lock:
                msg=self.latest;self.latest=None
                depths=list(self.depths);info=self.info
            if msg is None:
                self.stop_event.wait(.02);continue
            try:
                if msg.encoding not in ('rgb8','bgr8') or msg.step<msg.width*3 or not self.fresh(msg):
                    with self.lock:self.clear()
                    continue
                rgb=np.frombuffer(msg.data,np.uint8).reshape(msg.height,msg.step)[:,:msg.width*3].reshape(msg.height,msg.width,3)
                bgr=np.ascontiguousarray(rgb[:,:,::-1] if msg.encoding=='rgb8' else rgb)
                start=time.monotonic()
                result=self.model.predict(bgr,imgsz=cfg['image_size'],conf=cfg['confidence'],device=cfg['device'],verbose=False)[0]
                detections=[{'class_name':result.names[int(b.cls.item())],'confidence':float(b.conf.item()),
                             'xyxy':b.xyxy[0].tolist()} for b in result.boxes]
                payload,point=perception_result(msg,detections,depths,info,cfg['target_class'],cfg['confidence'],
                                                cfg['sync_tolerance'],cfg['min_depth'],cfg['max_depth'])
                payload['inference_ms']=round((time.monotonic()-start)*1000,1)
                with self.lock:
                    if not self.fresh(msg):
                        self.clear();continue
                    self.last_input=msg
                    self.detected_pub.publish(Bool(data=payload['detected']))
                    self.valid_pub.publish(Bool(data=payload['position_valid']))
                    if point is not None:
                        out=PointStamped();out.header=msg.header
                        out.point.x=point['x'];out.point.y=point['y'];out.point.z=point['z']
                        self.position_pub.publish(out)
                        pose=PoseStamped();pose.header=msg.header;pose.pose.position=out.point;pose.pose.orientation.w=1.
                        self.person_pose_pub.publish(pose)
                    self.person_pub.publish(Bool(data=payload['position_valid']))
                    self.details_pub.publish(String(data=json.dumps(payload,allow_nan=False)))
            except Exception as exc:
                with self.lock:self.clear()
                self.get_logger().error(f'Frame rejected: {exc}')
                self.stop_event.wait(.1)

    def close(self):
        self.stop_event.set();self.worker.join()
        with self.lock:self.clear()


def main(args=None):
    rclpy.init(args=args)
    node=None
    try:
        node=HumanDetector();rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
