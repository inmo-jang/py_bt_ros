"""Additive three-robot simulator; tasks are supplied by an external allocator."""
import os,sys,tempfile
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction,ExecuteProcess,RegisterEventHandler,TimerAction,EmitEvent,IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController

def enabled(context,key):return LaunchConfiguration(key).perform(context).lower() in ('true','1','yes')

def runtime(context):
    share=Path(get_package_share_directory('multirobot_sim'))
    env=yaml.safe_load((share/'config/environment.yaml').read_text())
    repo=LaunchConfiguration('repo_path').perform(context)
    python=LaunchConfiguration('python_executable').perform(context)
    temporary=Path(tempfile.mkdtemp(prefix='multirobot_sim_'))
    webots=WebotsLauncher(world=str(share/'worlds'/env['world']),mode='realtime',ros2_supervisor=True)
    actions=[webots,webots._supervisor];stacks=[]
    stacks.extend([
        Node(package='nav2_map_server',executable='map_server',name='multi_map_server',parameters=[{'yaml_filename':str(share/'config'/env['map']),'use_sim_time':True}]),
        Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='multi_map_lifecycle',parameters=[{'use_sim_time':True,'autostart':True,'node_names':['multi_map_server']}])])
    for ns in ('robot1','robot2','robot3'):
        actions.append(WebotsController(robot_name=ns,namespace=ns,parameters=[{'robot_description':str(share/f'config/{ns}.urdf'),'use_sim_time':True,'set_robot_state_publisher':False}],output='screen'))
        for child,xyz in [('laser_link',(.103,0.,-.034)),('camera_link',(.096,.015,.03))]:
            actions.append(Node(package='tf2_ros',executable='static_transform_publisher',name=ns+'_'+child+'_tf',arguments=['--x',str(xyz[0]),'--y',str(xyz[1]),'--z',str(xyz[2]),'--frame-id',ns+'/base_link','--child-frame-id',ns+'/'+child]))
        params=yaml.safe_load((share/f'config/{ns}_nav2.yaml').read_text())
        params['amcl']['ros__parameters']['initial_pose']=env['starts'][ns]
        file=temporary/(ns+'.yaml');file.write_text(yaml.safe_dump({ns:params},sort_keys=False))
        nodes=[('amcl','nav2_amcl'),('controller_server','nav2_controller'),('smoother_server','nav2_smoother'),('planner_server','nav2_planner'),('behavior_server','nav2_behaviors'),('bt_navigator','nav2_bt_navigator')]
        for name,package in nodes:stacks.append(Node(package=package,executable=name,name=name,namespace=ns,parameters=[str(file),{'use_sim_time':True}],output='screen'))
        stacks.append(Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='lifecycle_manager',namespace=ns,parameters=[{'autostart':True,'use_sim_time':True,'node_names':[n for n,p in nodes],'bond_timeout':10.}]))
        if enabled(context,'yolo'):
            model=str(Path(get_package_share_directory('human_yolo'))/'models/best.pt')
            script="import sys,runpy;sys.path.append('/usr/lib/python3/dist-packages');import cv2,torch;cv2.setNumThreads(1);torch.set_num_interop_threads(1);import ultralytics.utils.torch_utils as tu;tu.NUM_THREADS=1;runpy.run_module('human_yolo.detector',run_name='__main__')"
            cmd=[LaunchConfiguration('yolo_python').perform(context),'-c',script,'--ros-args','-r','__ns:=/'+ns,'-p','use_sim_time:=true','-p','model_path:='+model,'-p','cpu_threads:=1','-p','confidence:=0.65','-p','input_timeout:=2.0']
            for key,value in [('rgb_topic','camera/color/image_raw'),('depth_topic','camera/depth/image_raw'),('camera_info_topic','camera/color/camera_info')]:cmd+=['-p',key+':=/'+ns+'/'+value]
            for topic in ['/human/detected','/human/position_valid','/human/position','/human/detections','/perception/is_person','/perception/person_pose']:cmd+=['-r',topic+':=/'+ns+topic]
            stacks.append(ExecuteProcess(cmd=cmd,additional_env={'OMP_NUM_THREADS':'1','MKL_NUM_THREADS':'1','OPENBLAS_NUM_THREADS':'1','YOLO_CONFIG_DIR':str(temporary/'yolo')},output='screen'))
    bt=Path(get_package_share_directory('multirobot_bt'))/'launch/multirobot_bt.launch.py'
    stacks.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(bt)),launch_arguments={'repo_path':repo,'python_executable':python,'use_sim_time':'true','pygame':LaunchConfiguration('pygame').perform(context),'factory_regions_file':str(Path(repo)/'scenarios/limo_factory_search/config_sim.yaml') if enabled(context,'use_main_regions') else ''}.items()))
    stacks.append(Node(package='rviz2',executable='rviz2',arguments=['-d',str(share/'config/multi.rviz')],parameters=[{'use_sim_time':True}],additional_env={'LIBGL_ALWAYS_SOFTWARE':'1'},condition=IfCondition(LaunchConfiguration('rviz'))))
    gate=Node(package='multirobot_sim',executable='ready',output='screen')
    def sensors_ready(event,context):
        if context.is_shutdown:return []
        return [TimerAction(period=2.,actions=stacks)] if event.returncode==0 else [EmitEvent(event=Shutdown(reason='Three-robot sensors failed'))]
    actions.extend([RegisterEventHandler(OnProcessExit(target_action=gate,on_exit=sensors_ready)),gate,RegisterEventHandler(OnProcessExit(target_action=webots,on_exit=[EmitEvent(event=Shutdown(reason='Webots closed'))]))])
    return actions

def generate_launch_description():
    candidates=[*Path(__file__).resolve().parents,Path.cwd()]
    repo=next((p for p in candidates if (p/'modules/base_bt_nodes.py').is_file()),Path.cwd())
    bt_python=repo.parent/'.venv-view/bin/python';yolo_python=Path.home()/'yolo_project/.venv/bin/python'
    return LaunchDescription([
        DeclareLaunchArgument('repo_path',default_value=str(repo)),
        DeclareLaunchArgument('python_executable',default_value=str(bt_python) if bt_python.exists() else sys.executable),
        DeclareLaunchArgument('yolo_python',default_value=str(yolo_python) if yolo_python.exists() else sys.executable),
        DeclareLaunchArgument('use_main_regions',default_value='true'),
        DeclareLaunchArgument('yolo',default_value='true'),DeclareLaunchArgument('rviz',default_value='true'),DeclareLaunchArgument('pygame',default_value='true'),
        OpaqueFunction(function=runtime)])
