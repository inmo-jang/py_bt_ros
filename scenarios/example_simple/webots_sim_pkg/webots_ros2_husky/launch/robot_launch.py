import os
import re
import launch
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable  # ✅ 추가
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection
from launch_ros.actions import Node


# 월드 파일에서 인식할 로봇 타입 목록
ROBOTS_NAME_LIST = ['Fire_UGV']


def discover_robots_from_world(world_path):
    """월드 파일에서 Husky 기반 로봇들의 DEF 이름을 추출"""
    robot_names = []
    pattern = re.compile(r'^DEF\s+(\w+)\s+(' + '|'.join(ROBOTS_NAME_LIST) + r')\b', re.MULTILINE)
    
    with open(world_path, 'r') as f:
        content = f.read()
    
    for match in pattern.finditer(content):
        robot_names.append(match.group(1))
    
    return robot_names


def generate_launch_description():
    package_dir = get_package_share_directory('webots_ros2_husky')

    # ✅ 기존 값이 있으면 보존하면서 package_dir을 추가
    prev_extra = os.environ.get('WEBOTS_EXTRA_PROJECT_PATH', '')
    extra_path = package_dir if prev_extra == '' else (package_dir + os.pathsep + prev_extra)

    prev_proj = os.environ.get('WEBOTS_PROJECT_PATH', '')
    proj_path = package_dir if prev_proj == '' else (package_dir + os.pathsep + prev_proj)

    # ✅ Webots가 /tmp world를 열더라도 여기 경로에서 controllers를 찾게 됨
    set_webots_paths = [
        SetEnvironmentVariable('WEBOTS_EXTRA_PROJECT_PATH', extra_path),
        SetEnvironmentVariable('WEBOTS_PROJECT_PATH', proj_path),
    ]

    # Start a Webots simulation instance
    world_path = os.path.join(package_dir, 'worlds', 'fire_suppression.wbt')
    webots = WebotsLauncher(world=world_path)

    # 월드 파일에서 로봇 이름 동적 발견
    robot_names = discover_robots_from_world(world_path)
    print(f"[robot_launch] Discovered robots: {robot_names}")

    # Create the robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': '<robot name=""><link name=""/></robot>'
        }],
    )

    # ROS control spawners (모든 로봇이 공유하는 단일 controller_manager 사용)
    ros2_control_params = os.path.join(package_dir, 'resource', 'ros2control.yaml')
    controller_manager_timeout = ['--controller-manager-timeout', '50']

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_state_broadcaster'] + controller_manager_timeout,
    )
    diffdrive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['diffdrive_controller'] + controller_manager_timeout,
    )
    ros_control_spawners = [joint_state_broadcaster_spawner, diffdrive_controller_spawner]

    robot_description_path = os.path.join(package_dir, 'resource', 'husky.urdf')

    # ----------------------------------------
    # 로봇별 동적 생성: mappings, robot_driver
    # ----------------------------------------
    robot_drivers = []

    for robot_name in robot_names:
        # 각 로봇별 remapping
        mappings = [
            ('/diffdrive_controller/cmd_vel', f'/{robot_name}/cmd_vel'),
            ('/diffdrive_controller/odom', f'/{robot_name}/odom'),
        ]

        # 각 로봇별 WebotsController
        robot_driver = WebotsController(
            robot_name=robot_name,
            parameters=[
                {'robot_description': robot_description_path,
                 'set_robot_state_publisher': True},
                ros2_control_params
            ],
            remappings=mappings
        )
        robot_drivers.append(robot_driver)

    # LaunchDescription 구성
    launch_items = [
        *set_webots_paths,   # ✅ webots 앞에 들어가야 함
        webots,
        robot_state_publisher,
    ]

    # 모든 robot_driver 추가
    for robot_driver in robot_drivers:
        launch_items.append(robot_driver)

    # 마지막 로봇에만 controller spawner 연결 (공유 controller_manager 사용)
    if robot_drivers:
        waiting_node = WaitForControllerConnection(
            target_driver=robot_drivers[-1],
            nodes_to_start=ros_control_spawners
        )
        launch_items.append(waiting_node)

    launch_items.append(
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    )

    return LaunchDescription(launch_items)
    
if __name__ == '__main__':
    # VSCode 디버깅이나 직접 python3로 실행할 때 작동하는 부분
    from launch import LaunchService

    # 1. LaunchDescription 생성
    ld = generate_launch_description()

    # 2. LaunchService 초기화 및 설명(ld) 포함
    ls = LaunchService()
    ls.include_launch_description(ld)

    # 3. 실행
    ls.run()