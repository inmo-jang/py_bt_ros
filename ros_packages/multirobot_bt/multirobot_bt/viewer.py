"""Simple multi-robot BT displayed using the unchanged team Pygame renderer."""
import json,signal
import pygame
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile,DurabilityPolicy
from std_msgs.msg import String
from .bt import build,Status
from modules.bt_visualiser import BTViewer

def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=Node('multi_bt_viewer');pygame.init()
    surface=pygame.display.set_mode((1200,700));viewer=BTViewer();tree=build();clock=pygame.time.Clock()
    pygame.display.set_caption('Multi Robot BT | Waiting for status')
    def receive(msg):
        try:phase=json.loads(msg.data)['phase']
        except (ValueError,KeyError,TypeError):return
        tree.reset();rescue,patrol=tree.children
        if phase=='PATROL':
            tree.status=patrol.status=Status.RUNNING;rescue.children[0].status=Status.FAILURE
            for child in patrol.children:child.status=Status.RUNNING
        elif phase!='WAITING':
            tree.status=rescue.status=Status.SUCCESS if phase=='COMPLETE' else Status.FAILURE if phase in ('ERROR','STOPPED') else Status.RUNNING
            rescue.children[0].status=Status.SUCCESS
            active={'STOPPING':0,'WAIT_RESCUE_TASKS':1,'GATHER':2}.get(phase,2)
            for i,child in enumerate(rescue.children[1:]):
                if i<active:child.status=Status.SUCCESS
                elif i==active:child.status=tree.status
        pygame.display.set_caption('Multi Robot BT | '+phase)
    node.create_subscription(String,'/multi_bt/status',receive,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.get_logger().info('Pygame ready: simple multi-robot BT')
    requested=[False];signal.signal(signal.SIGINT,lambda *_:requested.__setitem__(0,True))
    try:
        running=True
        while running and rclpy.ok() and not requested[0]:
            events=pygame.event.get();running=not any(e.type==pygame.QUIT for e in events) and viewer.handle_events(events)
            rclpy.spin_once(node,timeout_sec=0);viewer.render_tree(surface,tree);pygame.display.flip();clock.tick(20)
    except KeyboardInterrupt:pass
    finally:
        signal.signal(signal.SIGINT,signal.SIG_IGN);node.destroy_node();pygame.quit()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
