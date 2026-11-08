import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    repo=str(next((p for p in [*Path(__file__).resolve().parents,Path.cwd()] if (p/'modules/base_bt_nodes.py').is_file()),Path.cwd()))
    local_python=Path(repo).parent/'.venv-view/bin/python'
    python=str(local_python) if local_python.exists() else '/usr/bin/python3'
    return LaunchDescription([
        DeclareLaunchArgument('repo_path',default_value=repo),
        DeclareLaunchArgument('pygame',default_value='false'),
        DeclareLaunchArgument('factory_regions_file',default_value=''),
        DeclareLaunchArgument('python_executable',default_value=python),
        DeclareLaunchArgument('use_sim_time',default_value='false'),
        Node(package='multirobot_bt',executable='multi_bt',output='screen',prefix=[LaunchConfiguration('python_executable')],
             parameters=[{'use_sim_time':LaunchConfiguration('use_sim_time'),'factory_regions_file':ParameterValue(LaunchConfiguration('factory_regions_file'),value_type=str)}],
             additional_env={'PYTHONPATH':[LaunchConfiguration('repo_path'),':',os.environ.get('PYTHONPATH','')]}),
        Node(package='multirobot_bt',executable='bt_viewer',prefix=[LaunchConfiguration('python_executable')],condition=IfCondition(LaunchConfiguration('pygame')),parameters=[{'use_sim_time':LaunchConfiguration('use_sim_time')}],additional_env={'PYTHONPATH':[LaunchConfiguration('repo_path'),':',os.environ.get('PYTHONPATH','')],'SDL_AUDIODRIVER':'dummy'})])
