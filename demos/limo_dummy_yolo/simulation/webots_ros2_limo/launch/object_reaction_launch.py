"""Attach YOLO + move/spin/resume reaction to an existing LIMO/Nav2 simulation."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_visualiser',default_value='true'),
        DeclareLaunchArgument('visualiser_python',default_value='/home/willy/limo_jazzy_ws/.venv-view/bin/python'),
        DeclareLaunchArgument('work_distance',default_value='0.30'),
        DeclareLaunchArgument('yolo_python', default_value='/home/willy/yolo_project/.venv/bin/python'),
        DeclareLaunchArgument('detector_script', default_value='/home/willy/limo_jazzy_ws/tools/box_training/detector.py'),
        ExecuteProcess(cmd=[LaunchConfiguration('yolo_python'),LaunchConfiguration('detector_script')],
            additional_env={'YOLO_CONFIG_DIR':'/home/willy/limo_jazzy_ws/box_yolo/config'},output='screen'),
        ExecuteProcess(cmd=[LaunchConfiguration('visualiser_python'),'/home/willy/limo_jazzy_ws/tools/box_training/visualise.py'],
            additional_env={'SDL_VIDEODRIVER':'x11','SDL_AUDIODRIVER':'dummy','SDL_VIDEO_WINDOW_POS':'100,80'},
            condition=IfCondition(LaunchConfiguration('use_visualiser')),output='screen'),
        Node(package='webots_ros2_limo',executable='object_spin',parameters=[{'use_sim_time':True,'work_distance':ParameterValue(LaunchConfiguration('work_distance'),value_type=float)}],output='screen'),
    ])
