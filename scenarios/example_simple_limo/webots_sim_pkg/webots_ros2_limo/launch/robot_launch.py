import os
import re
import yaml
import tempfile
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection
from launch_ros.actions import Node


# 월드 파일에서 인식할 로봇 타입 목록
ROBOTS_NAME_LIST = ['LimoFourDiff']


def create_namespaced_params_file(robot_name, template_path):
    """로봇 namespace에 맞게 YAML 키를 /{robot_name}/{node_name} 형태로 변환하여 임시 파일로 저장.

    ROS 2에서 --params-file의 YAML 키는 노드의 전체 경로(/namespace/node_name)와
    매칭되기 때문에, namespace를 부여한 경우 키를 namespace 포함 형태로 재작성해야 한다.
    """
    with open(template_path, 'r') as f:
        params = yaml.safe_load(f)

    namespaced = {f'/{robot_name}/{node_name}': node_params for node_name, node_params in params.items()}

    tmp = tempfile.NamedTemporaryFile(
        mode='w', suffix='.yaml', delete=False, prefix=f'ros2ctrl_{robot_name}_'
    )
    yaml.dump(namespaced, tmp)
    tmp.close()
    return tmp.name


def discover_robots_from_world(world_path):
    """월드 파일에서 LimoFourDiff 기반 로봇들의 DEF 이름을 추출.

    DEF가 없는 단일 LimoFourDiff도 감지하여 기본 이름으로 반환.
    """
    robot_names = []
    # DEF SomeName LimoFourDiff 패턴 매칭
    pattern = re.compile(r'^DEF\s+(\w+)\s+(' + '|'.join(ROBOTS_NAME_LIST) + r')\b', re.MULTILINE)

    with open(world_path, 'r') as f:
        content = f.read()

    for match in pattern.finditer(content):
        robot_names.append(match.group(1))

    return robot_names


def generate_launch_description():
    package_dir = get_package_share_directory('webots_ros2_limo')

    # ✅ Webots가 /tmp 월드를 열더라도 controllers/ 폴더를 찾을 수 있도록 경로 설정
    prev_extra = os.environ.get('WEBOTS_EXTRA_PROJECT_PATH', '')
    extra_path = package_dir if prev_extra == '' else (package_dir + os.pathsep + prev_extra)

    prev_proj = os.environ.get('WEBOTS_PROJECT_PATH', '')
    proj_path = package_dir if prev_proj == '' else (package_dir + os.pathsep + prev_proj)

    set_webots_paths = [
        SetEnvironmentVariable('WEBOTS_EXTRA_PROJECT_PATH', extra_path),
        SetEnvironmentVariable('WEBOTS_PROJECT_PATH', proj_path),
    ]

    # debug:=true 시 world_supervisor 등에서 visualisation topics publish
    debug_arg = DeclareLaunchArgument(
        'debug', default_value='false',
        description='Enable debug visualisation topics (fires in RViz, etc.)'
    )
    set_debug_env = SetEnvironmentVariable('DEBUG', LaunchConfiguration('debug'))

    # Start a Webots simulation instance
    world_path = os.path.join(package_dir, 'worlds', 'fire_suppression.wbt')
    webots = WebotsLauncher(world=world_path)

    # 월드 파일에서 로봇 이름 동적 발견
    robot_names = discover_robots_from_world(world_path)
    print(f"[robot_launch] Discovered robots: {robot_names}")

    # ROS control parameters
    ros2_control_params = os.path.join(package_dir, 'resource', 'ros2control.yaml')
    controller_manager_timeout = ['--controller-manager-timeout', '50']
    robot_description_path = os.path.join(package_dir, 'resource', 'limo.urdf')

    # ----------------------------------------
    # 로봇별 동적 생성: robot_driver + controller spawners + WaitForControllerConnection
    # ----------------------------------------
    robot_drivers = []
    ros_control_items = []
    robot_state_publishers = []

    for robot_name in robot_names:
        # 다중 로봇인 경우에만 namespace 적용
        use_namespace = len(robot_names) > 1
        ns = robot_name if use_namespace else None

        robot_state_publishers.append(
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                namespace=ns,
                output='screen',
                parameters=[{
                    'robot_description': '<robot name=""><link name=""/></robot>'
                }],
            )
        )

        # Topic remappings
        if use_namespace:
            # namespace 사용 시: webots_ros2_driver가 robot_name prefix를 붙이므로
            # 이중 네임스페이스 방지를 위해 remapping
            mappings = [
                ('diffdrive_controller/cmd_vel', f'/{robot_name}/cmd_vel'),
                ('diffdrive_controller/odom', f'/{robot_name}/odom'),
                (f'{robot_name}/laser', 'scan'),
            ]
        else:
            # 단일 로봇: robot_name 기반으로 remapping
            mappings = [
                ('/diffdrive_controller/cmd_vel', '/cmd_vel'),
                ('/diffdrive_controller/odom', '/odom'),
                (f'/{robot_name}/laser', '/scan'),
            ]

        # 다중 로봇 시 namespace에 맞는 params 파일 생성
        if use_namespace:
            namespaced_params = create_namespaced_params_file(robot_name, ros2_control_params)
            params_file = namespaced_params
        else:
            params_file = ros2_control_params

        robot_driver = WebotsController(
            robot_name=robot_name,
            namespace=ns,
            parameters=[
                {'robot_description': robot_description_path,
                 'set_robot_state_publisher': True},
                params_file
            ],
            remappings=mappings
        )
        robot_drivers.append(robot_driver)

        # Controller spawners
        if use_namespace:
            cm_prefix = f'{robot_name}/controller_manager'
        else:
            cm_prefix = None

        spawner_args_jsb = ['joint_state_broadcaster']
        spawner_args_ddc = ['diffdrive_controller']
        if cm_prefix:
            spawner_args_jsb += ['-c', cm_prefix]
            spawner_args_ddc += ['-c', cm_prefix]
        spawner_args_jsb += controller_manager_timeout
        spawner_args_ddc += controller_manager_timeout

        joint_state_broadcaster_spawner = Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            arguments=spawner_args_jsb,
        )
        diffdrive_controller_spawner = Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            arguments=spawner_args_ddc,
        )

        waiting_node = WaitForControllerConnection(
            target_driver=robot_driver,
            nodes_to_start=[joint_state_broadcaster_spawner, diffdrive_controller_spawner]
        )

        ros_control_items.append(robot_driver)
        ros_control_items.append(waiting_node)

    # LaunchDescription 구성
    launch_items = [
        debug_arg,
        set_debug_env,
        *set_webots_paths,
        webots,
        *robot_state_publishers,
        *ros_control_items,
    ]

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
    from launch import LaunchService

    ld = generate_launch_description()
    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
