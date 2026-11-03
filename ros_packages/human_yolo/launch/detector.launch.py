"""Attach perception to existing camera topics; never launch a simulator or BT."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share=FindPackageShare('human_yolo')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',default_value='false'),
        DeclareLaunchArgument('model_path',default_value=PathJoinSubstitution([share,'models','best.pt'])),
        DeclareLaunchArgument('params_file',default_value=PathJoinSubstitution([share,'config','detector.yaml'])),
        Node(package='human_yolo',executable='human_detector',name='human_yolo',output='screen',
             parameters=[LaunchConfiguration('params_file'),
                         {'model_path':LaunchConfiguration('model_path'),
                          'use_sim_time':ParameterValue(LaunchConfiguration('use_sim_time'),value_type=bool)}]),
    ])
