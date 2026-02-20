#!/usr/bin/env python3
"""
World Supervisor
- Fire / Target / Base 등 world object를 관리
- 동적으로 object spawn/remove 지원
- pose publish
"""

from controller import Supervisor
import math
import os
import random
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty
from std_msgs.msg import Float64, UInt8  # ✅ UInt8 추가 (Target status용)
from std_msgs.msg import UInt16MultiArray  # Target을 Summary 형태로 전달하기 위해 추가
from std_msgs.msg import Float64MultiArray  # Custom fire spawn용
from ament_index_python.packages import get_package_share_directory


# -----------------------------
# Object Category 정의
# -----------------------------
class ObjectCategory:
    FIRE = "Fire"
    TARGET = "Target"
    WATER = "Water"
    BASE = "Base"


# PROTO 템플릿 (spawn 시 사용)
PROTO_TEMPLATES = {
  ObjectCategory.FIRE: """
DEF {def_name} Fire {{
  translation {x} {y} {z}
  radius {radius}
}}
""",
    ObjectCategory.TARGET: """
DEF {def_name} Target {{
  translation {x} {y} {z}
  rotation 0 0 1 -1.57
  status {status}
  textureUrl [ "{texture_url}" ]
}}
"""
}


# -----------------------------
# util
# -----------------------------
def axis_angle_to_quaternion(axis_x, axis_y, axis_z, angle):
    """axis-angle -> quaternion 변환"""
    norm = math.sqrt(axis_x * axis_x + axis_y * axis_y + axis_z * axis_z)
    if norm < 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    axis_x, axis_y, axis_z = axis_x / norm, axis_y / norm, axis_z / norm
    sin_half = math.sin(angle * 0.5)
    return (axis_x * sin_half, axis_y * sin_half, axis_z * sin_half, math.cos(angle * 0.5))


# -----------------------------
# 동적 Object 관리 클래스
# -----------------------------
class DynamicObjectManager:
    def __init__(self, supervisor: Supervisor, category: str, def_prefix: str):
        """카테고리별(Webots DEF prefix 기반) 객체 spawn/remove를 관리"""
        self.supervisor = supervisor
        self.category = category
        self.def_prefix = def_prefix
        self.active_objects = {}
        self.next_index = 1

    # 월드에 이미 존재하는 객체를 찾아서 목록에 등록 (초기화 시 실행)
    def scan_existing_objects(self):
        """월드에 이미 존재하는 객체(DEF prefix 매칭)를 스캔"""
        root = self.supervisor.getRoot()
        children_field = root.getField("children")
        children_count = children_field.getCount()

        max_found_index = 0
        for i in range(children_count):
            child_node = children_field.getMFNode(i)
            if child_node is None:
                continue

            def_name = child_node.getDef()
            if def_name and def_name.startswith(self.def_prefix):
                try:
                    index_str = def_name[len(self.def_prefix):]
                    index = int(index_str)
                    self.active_objects[def_name] = child_node
                    max_found_index = max(max_found_index, index)
                except ValueError:
                    pass
        
        self.next_index = max_found_index + 1   # 다음 인덱스 설정 -> 겹치지 않도록 (Fire_1,2,3이 있는 상태에서 2가 없어져도 spawn시 Fire_4로 생성되도록)
        return list(self.active_objects.keys())

    # 특정 객체의 PROTO 필드(status) 값을 읽어옴 (Target의 초기 상태 확인용)
    def get_object_status(self, def_name: str) -> int:
        """PROTO field 'status'를 읽어 반환 (field가 없으면 0)"""
        node = self.active_objects.get(def_name)
        if node:
            field = node.getField("status")
            if field:
                return field.getSFInt32()
        return 0

    # 새로운 객체를 월드에 생성 (PROTO 템플릿 사용)
    def spawn_object(self, position_x: float, position_y: float, position_z: float, **kwargs) -> str:
        """PROTO 템플릿을 사용해 객체를 동적으로 생성"""
        def_name = f"{self.def_prefix}{self.next_index}"
        self.next_index += 1

        template = PROTO_TEMPLATES.get(self.category)
        if template is None:
            return None

        try:
            proto_string = template.format(
                def_name=def_name,
                x=position_x,
                y=position_y,
                z=position_z,
                **kwargs
            )
        except KeyError as e:
            print(f"Error formatting template: Missing key {e}")
            return None

        root = self.supervisor.getRoot()
        children_field = root.getField("children")
        children_field.importMFNodeFromString(-1, proto_string)

        new_node = self.supervisor.getFromDef(def_name)
        if new_node:
            self.active_objects[def_name] = new_node
            return def_name
        return None

    def remove_object(self, def_name: str) -> bool:
        """객체를 월드에서 제거"""
        if def_name not in self.active_objects:
            return False
        webots_node = self.active_objects[def_name]
        if webots_node:
            webots_node.remove()
        del self.active_objects[def_name]
        return True

    def get_active_object_names(self):
        """현재 active 상태인 DEF 리스트 반환"""
        return list(self.active_objects.keys())

    def get_webots_node(self, def_name: str):
        """DEF에 해당하는 Webots node 반환"""
        return self.active_objects.get(def_name)


# -----------------------------
# ROS Node
# -----------------------------
class WorldSupervisor(Node):
    def __init__(self, supervisor: Supervisor):
        """WorldSupervisor 초기화 및 publisher/service 구성"""
        super().__init__("world_supervisor")

        self.supervisor = supervisor
        self.timestep = int(self.supervisor.getBasicTimeStep())

        # 월드 파일 경로를 기준으로 icons 폴더 경로 저장
        # Webots ROS2에서는 월드 파일이 /tmp에 복사되므로 패키지 경로 사용
        try:
            pkg_share = get_package_share_directory("webots_ros2_husky")
            self.icons_dir = os.path.join(pkg_share, "worlds", "icons")
        except Exception:
            # fallback: 기존 방식
            world_path = self.supervisor.getWorldPath()
            self.icons_dir = os.path.join(os.path.dirname(world_path), "icons")

        self.frame_id = "world" #"webots_world"
        self.publish_rate = 30.0  # 20.0

        self.fire_manager = DynamicObjectManager(supervisor, ObjectCategory.FIRE, "Fire_")
        self.target_manager = DynamicObjectManager(supervisor, ObjectCategory.TARGET, "Target_")
        self.water_manager = DynamicObjectManager(supervisor, ObjectCategory.WATER, "Water_")

        self.base_node = None
        self.base_def_name = "Base"

        # Target status: 0=UNCHECKED, 1=CHECKED, 2=COMPLETED
        self.target_status = {}              # { Target_1: 0/1/2 }
        self.target_status_publishers = {}   # { Target_1: Publisher(UInt8) }
        self.target_check_services = {}      # { Target_1: Service(/check) }

        self._scan_initial_objects()

        self.fire_publishers = {}
        self.fire_radius_publishers = {}
        self.target_publishers = {}
        self.water_publishers = {}
        self.base_publisher = None

        self._create_initial_publishers()

        self.remove_services = {}  # suppress/complete 같은 “삭제” 서비스들 관리
        self.spawn_services = {}

        self.target_summary_publishers = self.create_publisher(UInt16MultiArray, "/world/target/summary", 1)  # Target Summary
        self.fire_total_spawned = len(self.fire_manager.get_active_object_names())
        self.fire_summary_publishers = self.create_publisher(UInt16MultiArray, "/world/fire/summary", 1)  # Fire Summary

        # ===== Fire 자동 스폰 및 확산 설정 =====
        self.fire_auto_spawn_enabled = False      # 자동 스폰 활성화 여부
        self.fire_auto_spawn_interval = 20.0     # 자동 스폰 간격 (초)
        self.fire_spawn_range = 20.0             # 스폰 가능 범위 (+/-)
        self.fire_max_count = 20                 # 최대 Fire 개수 제한

        self.fire_spread_enabled = True          # 확산 활성화 여부
        self.fire_spread_interval = 5.0         # 확산 시도 간격 (초)
        self.fire_spread_probability = 0.8       # 확산 확률 (0.0 ~ 1.0)
        self.fire_spread_distance_min = 2.0      # 확산 최소 거리
        self.fire_spread_distance_max = 5.0      # 확산 최대 거리

        self._create_spawn_services()
        self._create_initial_remove_services()

        # Fire 자동 스폰 타이머
        if self.fire_auto_spawn_enabled:
            self.fire_auto_spawn_timer = self.create_timer(self.fire_auto_spawn_interval, self._auto_spawn_fire)
            self.get_logger().info(f"Fire auto spawn enabled: every {self.fire_auto_spawn_interval}s")

        # Fire 확산 타이머
        if self.fire_spread_enabled:
            self.fire_spread_timer = self.create_timer(self.fire_spread_interval, self._spread_fire_from_existing)
            self.get_logger().info(f"Fire spread enabled: every {self.fire_spread_interval}s, prob={self.fire_spread_probability}")

        self.last_publish_time = self.get_clock().now()
        self.get_logger().info("WorldSupervisor ready")

    # 프로그램 시작 시 월드에 미리 배치된 객체들을 인식
    def _scan_initial_objects(self):
        """월드 초기 객체(Fire/Target/Water/Base) 스캔"""
        self.fire_manager.scan_existing_objects()
        self.target_manager.scan_existing_objects()
        self.water_manager.scan_existing_objects()
        self.base_node = self.supervisor.getFromDef(self.base_def_name)

    # 인식된 초기 객체들에 대해 Publisher 및 상태 초기화 수행
    def _create_initial_publishers(self):
        """초기 존재하는 객체들에 대한 publisher 생성"""
        for def_name in self.fire_manager.get_active_object_names():
            self._create_fire_publisher(def_name)

        for def_name in self.target_manager.get_active_object_names():
            self._create_target_publisher(def_name)
            
            # PROTO field에서 초기 상태 읽어오기
            initial_status = self.target_manager.get_object_status(def_name)
            self.target_status[def_name] = initial_status
            
            # 초기 상태에 맞춰 텍스처 설정 (1=Checked=White, 0=Unchecked=Red)
            self._set_target_texture(def_name, initial_status == 1)
            
            self._create_target_status_publisher(def_name)   # ✅ status pub 추가

        for def_name in self.water_manager.get_active_object_names():
            self._create_water_publisher(def_name)

        if self.base_node:
            self.base_publisher = self.create_publisher(PoseStamped, "/world/base/pose", 1)

    def _create_fire_publisher(self, def_name: str):
        """Fire pose/radius 퍼블리셔 생성"""
        topic_name = f"/world/fire/{def_name}/pose"
        self.fire_publishers[def_name] = self.create_publisher(PoseStamped, topic_name, 1)

        radius_topic_name = f"/world/fire/{def_name}/radius"
        self.fire_radius_publishers[def_name] = self.create_publisher(Float64, radius_topic_name, 1)

    def _create_target_publisher(self, def_name: str):
        """Target pose 퍼블리셔 생성"""
        topic_name = f"/world/target/{def_name}/pose"
        self.target_publishers[def_name] = self.create_publisher(PoseStamped, topic_name, 1)

    def _create_target_status_publisher(self, def_name: str):
        """Target status 퍼블리셔 생성"""
        topic_name = f"/world/target/{def_name}/status"
        self.target_status_publishers[def_name] = self.create_publisher(UInt8, topic_name, 1)

    def _create_water_publisher(self, def_name: str):
        """Water pose 퍼블리셔 생성"""
        topic_name = f"/world/water/{def_name}/pose"
        self.water_publishers[def_name] = self.create_publisher(PoseStamped, topic_name, 1)

    def _create_spawn_services(self):   # 직접 터미널에서 호출하는 방식으로 구현
        """spawn 서비스 및 topic 구독 생성"""
        self.create_service(Empty, "/world/fire/spawn", self._handle_spawn_fire)
        self.create_service(Empty, "/world/target/spawn", self._handle_spawn_target)
        # Fire 지정 생성 / x, y, radius
        self.create_subscription(Float64MultiArray, "/world/fire/spawn_custom", self._handle_spawn_fire_custom, 1)
        # Target 지정 생성 / x, y
        self.create_subscription(Float64MultiArray, "/world/target/spawn_custom", self._handle_spawn_target_custom, 1)

    def _handle_spawn_fire(self, request, response):
        """Fire를 랜덤 위치/크기로 생성"""
        range_limit = 12.5  # +/-12.5
        rand_x = round(random.uniform(-range_limit, range_limit), 2)
        rand_y = round(random.uniform(-range_limit, range_limit), 2)
        default_z = 0.0

        rand_radius = round(random.uniform(0.5, 3.0), 2)

        def_name = self.fire_manager.spawn_object(rand_x, rand_y, default_z, radius=rand_radius)

        if def_name:
            self._create_fire_publisher(def_name)
            self._create_suppress_service_for_fire(def_name)
            self.get_logger().info(f"Spawned {def_name} at ({rand_x}, {rand_y}) with radius {rand_radius}")
            self.fire_total_spawned += 1
        else:
            self.get_logger().error("Failed to spawn Fire")

        return response

    def _handle_spawn_fire_custom(self, msg):
        """Fire를 지정된 위치/크기로 생성 / msg.data = [x, y, radius]"""
        if len(msg.data) < 3:
            self.get_logger().error("spawn_custom requires [x, y, radius]")
            return

        x, y, radius = msg.data[0], msg.data[1], msg.data[2]
        def_name = self.fire_manager.spawn_object(x, y, 0.0, radius=radius)

        if def_name:
            self._create_fire_publisher(def_name)
            self._create_suppress_service_for_fire(def_name)
            self.get_logger().info(f"Spawned {def_name} at ({x}, {y}) with radius {radius}")
            self.fire_total_spawned += 1
        else:
            self.get_logger().error("Failed to spawn Fire")

    def _handle_spawn_target(self, request, response):
        """Target을 랜덤 위치에 생성"""
        range_limit = 10.0
        rand_x = round(random.uniform(-range_limit, range_limit), 2)
        rand_y = round(random.uniform(-range_limit, range_limit), 2)
        default_z = 0.03
        
        # Spawn new target (Always Unchecked=0, Red)
        icon_path = os.path.join(self.icons_dir, "target_unchecked.png") # Red
        def_name = self.target_manager.spawn_object(rand_x, rand_y, default_z, status=0, texture_url=icon_path)

        if def_name:
            self._create_target_publisher(def_name)

            # target의 status/pub/service 생성
            self.target_status[def_name] = 0
            self._create_target_status_publisher(def_name)
            self._create_check_service_for_target(def_name)

            # complete 서비스 생성
            self._create_complete_service_for_target(def_name)

            self.get_logger().info(f"Spawned {def_name}")
        return response

    def _handle_spawn_target_custom(self, msg):
        """Target을 지정된 위치에 생성 / msg.data = [x, y]"""
        if len(msg.data) < 2:
            self.get_logger().error("spawn_custom requires [x, y]")
            return

        x, y = msg.data[0], msg.data[1]
        default_z = 0.03

        # Spawn new target (Always Unchecked=0, Red)
        icon_path = os.path.join(self.icons_dir, "target_unchecked.png") # Red
        def_name = self.target_manager.spawn_object(x, y, default_z, status=0, texture_url=icon_path)

        if def_name:
            self._create_target_publisher(def_name)

            # target의 status/pub/service 생성
            self.target_status[def_name] = 0
            self._create_target_status_publisher(def_name)
            self._create_check_service_for_target(def_name)

            # complete 서비스 생성
            self._create_complete_service_for_target(def_name)

            self.get_logger().info(f"Spawned {def_name} at ({x}, {y})")

    def _create_initial_remove_services(self):
        """초기 존재 객체들의 suppress/complete/check 서비스 생성"""
        for def_name in self.fire_manager.get_active_object_names():
            self._create_suppress_service_for_fire(def_name)

        for def_name in self.target_manager.get_active_object_names():
            self._create_complete_service_for_target(def_name)
            self._create_check_service_for_target(def_name)  # ✅ check srv 추가

    # ===== Fire 자동 스폰 =====
    def _auto_spawn_fire(self):
        """랜덤 위치에 Fire 자동 생성"""
        current_count = len(self.fire_manager.get_active_object_names())
        if current_count >= self.fire_max_count:  # 최대 개수 제한
            self.get_logger().info(f"Fire count ({current_count}) reached max ({self.fire_max_count}), skipping auto spawn")
            return

        rand_x = round(random.uniform(-self.fire_spawn_range, self.fire_spawn_range), 2)
        rand_y = round(random.uniform(-self.fire_spawn_range, self.fire_spawn_range), 2)
        rand_radius = round(random.uniform(0.5, 2.5), 2)

        def_name = self.fire_manager.spawn_object(rand_x, rand_y, 0.0, radius=rand_radius)
        if def_name:
            self._create_fire_publisher(def_name)
            self._create_suppress_service_for_fire(def_name)
            self.fire_total_spawned += 1
            self.get_logger().info(f"[Auto] Spawned {def_name} at ({rand_x}, {rand_y}) radius={rand_radius}")

    # ===== Fire 확산 =====
    def _spread_fire_from_existing(self):
        """기존 Fire 주변으로 확산"""
        current_count = len(self.fire_manager.get_active_object_names())
        if current_count >= self.fire_max_count:
            self.get_logger().debug(f"Fire count ({current_count}) reached max, skipping spread")
            return

        active_fires = list(self.fire_manager.active_objects.keys())
        if not active_fires:
            return

        # 랜덤하게 하나의 Fire 선택하여 확산 시도
        source_fire = random.choice(active_fires)
        if random.random() > self.fire_spread_probability:
            self.get_logger().debug(f"Spread skipped (prob check failed) for {source_fire}")
            return

        node = self.fire_manager.get_webots_node(source_fire)
        if not node:
            return

        pos = node.getField("translation").getSFVec3f()
        
        # 확산 방향 및 거리 계산
        angle = random.uniform(0, 2 * math.pi)
        distance = random.uniform(self.fire_spread_distance_min, self.fire_spread_distance_max)
        new_x = round(pos[0] + distance * math.cos(angle), 2)
        new_y = round(pos[1] + distance * math.sin(angle), 2)

        # 맵 범위 체크
        if abs(new_x) > self.fire_spawn_range or abs(new_y) > self.fire_spawn_range:
            self.get_logger().debug(f"Spread position ({new_x}, {new_y}) out of range, skipping")
            return

        # 기존 Fire와 너무 가까운지 체크 (겹침 방지)
        for other_name in active_fires:
            other_node = self.fire_manager.get_webots_node(other_name)
            if other_node:
                other_pos = other_node.getField("translation").getSFVec3f()
                dist = math.sqrt((new_x - other_pos[0])**2 + (new_y - other_pos[1])**2)
                if dist < 2.0:  # 최소 2m 간격
                    self.get_logger().debug(f"Too close to {other_name}, skipping spread")
                    return

        # 새 Fire 생성 (소스 Fire보다 약간 작게)
        source_radius = self._read_fire_radius(node)
        new_radius = round(random.uniform(0.5, max(0.5, source_radius * 0.8)), 2)

        def_name = self.fire_manager.spawn_object(new_x, new_y, 0.0, radius=new_radius)
        if def_name:
            self._create_fire_publisher(def_name)
            self._create_suppress_service_for_fire(def_name)
            self.fire_total_spawned += 1
            self.get_logger().info(f"[Spread] {source_fire} -> {def_name} at ({new_x}, {new_y}) radius={new_radius}")

    def _create_suppress_service_for_fire(self, def_name: str):
        """Fire suppress 서비스 생성"""
        service_name = f"/world/fire/{def_name}/suppress"
        service = self.create_service(Empty, service_name, self._make_fire_suppress_callback(def_name))
        self.remove_services[def_name] = service

    def _create_complete_service_for_target(self, def_name: str):
        """Target complete(삭제) 서비스 생성"""
        service_name = f"/world/target/{def_name}/complete"
        service = self.create_service(Empty, service_name, self._make_target_complete_callback(def_name))
        self.remove_services[def_name] = service

    def _create_check_service_for_target(self, def_name: str):
        """Target check 서비스 생성 (Mavic가 호출 예정)"""
        # TODO(ML-BT): Mavic의 CheckTarget 노드에서 이 서비스를 호출하도록 구현
        service_name = f"/world/target/{def_name}/check"
        srv = self.create_service(Empty, service_name, self._make_target_check_callback(def_name))
        self.target_check_services[def_name] = srv

    # Target Check 서비스(UAV가 호출) 처리용 콜백 함수 생성
    def _make_target_check_callback(self, def_name: str):
        """check 서비스 콜 시 UNCHECKED -> CHECKED로 변경 및 텍스처 변경"""
        def callback(request, response):
            node = self.target_manager.get_webots_node(def_name)
            if not node:
                return response  # 이미 삭제된 경우 무시

            prev = self.target_status.get(def_name, 0)
            if prev == 0:
                self.target_status[def_name] = 1
                self._set_target_texture(def_name, True)
                self.get_logger().info(f"{def_name} checked")
            return response
        return callback

    def _set_target_texture(self, def_name: str, is_checked: bool):
        """Target의 텍스처를 상태에 따라 변경 (Red/White)"""
        node = self.target_manager.get_webots_node(def_name)
        if not node:
            return

        texture_name = "target_checked.png" if is_checked else "target_unchecked.png"
        
        # PROTO field 'textureUrl' 변경 시도 (우선)
        updated = False
        try:
            texture_field = node.getField("textureUrl")
            if texture_field:
                path = os.path.join(self.icons_dir, texture_name)
                texture_field.setMFString(0, path)
                updated = True
        except Exception:
            pass
        
        # Fallback: 기존 children...texture 방식
        if not updated:
            try:
                children = node.getField("children")
                if children and children.getCount() > 0:
                    shape = children.getMFNode(0)
                    appearance = shape.getField("appearance").getSFNode()
                    texture = appearance.getField("texture").getSFNode()
                    url_field = texture.getField("url")
                    path = os.path.join(self.icons_dir, texture_name)
                    url_field.setMFString(0, path)
            except Exception as e:
                self.get_logger().warn(f"Failed to set texture for {def_name}: {e}")

    def _make_fire_suppress_callback(self, def_name: str):
        """suppress 서비스 콜 시 Fire 제거 + 리소스 정리 (서비스 destroy는 지연)"""
        def callback(request, response):
            success = self.fire_manager.remove_object(def_name)
            if not success:
                return response

            # pub 정리
            if def_name in self.fire_publishers:
                self.destroy_publisher(self.fire_publishers[def_name])
                del self.fire_publishers[def_name]
            if def_name in self.fire_radius_publishers:
                self.destroy_publisher(self.fire_radius_publishers[def_name])
                del self.fire_radius_publishers[def_name]

            self.get_logger().info(f"{def_name} suppressed")

            cleanup_timer = None

            def cleanup_services():
                nonlocal cleanup_timer
                if def_name in self.remove_services:
                    self.destroy_service(self.remove_services[def_name])
                    del self.remove_services[def_name]
                if cleanup_timer:
                    cleanup_timer.cancel()

            cleanup_timer = self.create_timer(0.1, cleanup_services)
            return response
        return callback

    def _make_target_complete_callback(self, def_name: str):
        """complete 서비스 콜 시 Target을 COMPLETED 처리 후 월드에서 삭제"""
        def callback(request, response):
            # COMPLETED 상태 기록(디버깅용) 후 바로 삭제
            self.target_status[def_name] = 2

            success = self.target_manager.remove_object(def_name)
            if success:
                # pose pub 정리
                if def_name in self.target_publishers:
                    self.destroy_publisher(self.target_publishers[def_name])
                    del self.target_publishers[def_name]

                # status pub 정리
                if def_name in self.target_status_publishers:
                    self.destroy_publisher(self.target_status_publishers[def_name])
                    del self.target_status_publishers[def_name]

                # status dict 정리
                if def_name in self.target_status:
                    del self.target_status[def_name]

                self.get_logger().info(f"{def_name} completed and removed")

                # 서비스 정리를 타이머로 지연 (콜백 완료 후 정리)
                cleanup_timer = None

                def cleanup_services():
                    nonlocal cleanup_timer
                    if def_name in self.target_check_services:
                        self.destroy_service(self.target_check_services[def_name])
                        del self.target_check_services[def_name]
                    if def_name in self.remove_services:
                        self.destroy_service(self.remove_services[def_name])
                        del self.remove_services[def_name]
                    # 타이머 한 번 실행 후 취소
                    if cleanup_timer:
                        cleanup_timer.cancel()

                cleanup_timer = self.create_timer(0.1, cleanup_services)

            return response
        return callback

    def _read_fire_radius(self, webots_node):
        """Webots Fire PROTO node에서 radius 필드를 읽음"""
        try:
            radius_field = webots_node.getField("radius")
            if radius_field:
                return radius_field.getSFFloat()
        except Exception:
            pass
        return 2.0

    def _read_pose(self, webots_node):
        """Webots node의 translation/rotation을 PoseStamped로 변환"""
        trans = webots_node.getField("translation").getSFVec3f()
        rot = webots_node.getField("rotation").getSFRotation()
        quat = axis_angle_to_quaternion(rot[0], rot[1], rot[2], rot[3])
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = trans[0]
        msg.pose.position.y = trans[1]
        msg.pose.position.z = trans[2]
        msg.pose.orientation.x = quat[0]
        msg.pose.orientation.y = quat[1]
        msg.pose.orientation.z = quat[2]
        msg.pose.orientation.w = quat[3]
        return msg

    def _publish_target_summary(self):
        """모든 Target의 상태를 UInt16MultiArray로 요약하여 퍼블리시"""
        active_targets = self.target_manager.get_active_object_names()
        total_targets = len(active_targets)
        unchecked = 0
        checked = 0
        completed = 0

        for t in active_targets:
            status = int(self.target_status.get(t, 0))
            if status == 0:
                unchecked += 1
            elif status == 1:
                checked += 1
            else:   # completed일 때 여기 들어오는 것 방지
                completed += 1
        
        msg = UInt16MultiArray()
        msg.data = [total_targets, unchecked, checked, completed]
        self.target_summary_publishers.publish(msg)

    def _publish_fire_summary(self):
        """모든 Fire의 상태를 UInt16MultiArray로 요약하여 퍼블리시"""
        active_fires = len(self.fire_manager.get_active_object_names())
        total_fires = int(self.fire_total_spawned)
        suppressed = max(0, total_fires - active_fires)

        msg = UInt16MultiArray()
        msg.data = [total_fires, int(active_fires), int(suppressed)]
        self.fire_summary_publishers.publish(msg)

    def publish_if_needed(self):
        """publish_rate에 맞춰 pose/radius/status를 주기적으로 publish"""
        now = self.get_clock().now()
        if (now - self.last_publish_time).nanoseconds < 1e9 / self.publish_rate:
            return
        self.last_publish_time = now

        for def_name, pub in list(self.fire_publishers.items()):
            node = self.fire_manager.get_webots_node(def_name)
            if node:
                pub.publish(self._read_pose(node))
                r_pub = self.fire_radius_publishers.get(def_name)
                if r_pub:
                    msg = Float64()
                    msg.data = float(self._read_fire_radius(node))
                    r_pub.publish(msg)

        for def_name, pub in list(self.target_publishers.items()):
            node = self.target_manager.get_webots_node(def_name)
            if node:
                pub.publish(self._read_pose(node))

                s_pub = self.target_status_publishers.get(def_name)
                if s_pub:
                    msg = UInt8()
                    msg.data = int(self.target_status.get(def_name, 0))
                    s_pub.publish(msg)

        for def_name, pub in list(self.water_publishers.items()):
            node = self.water_manager.get_webots_node(def_name)
            if node:
                pub.publish(self._read_pose(node))

        if self.base_node and self.base_publisher:
            self.base_publisher.publish(self._read_pose(self.base_node))

        self._publish_target_summary()
        self._publish_fire_summary()


def main():
    """Webots step 루프에서 ROS spin + publish를 수행"""
    supervisor = Supervisor()
    rclpy.init()
    node = WorldSupervisor(supervisor)
    try:
        while supervisor.step(node.timestep) != -1:
            rclpy.spin_once(node, timeout_sec=0.0)
            node.publish_if_needed()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
