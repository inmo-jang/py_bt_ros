# Copyright 1996-2023 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""ROS2 Mavic 2 Pro driver."""

import math
import sys
import rclpy
from controller import Robot
from geometry_msgs.msg import Twist


K_VERTICAL_THRUST = 68.5    # with this thrust, the drone lifts.
K_VERTICAL_P = 3.0          # P constant of the vertical PID.
K_VERTICAL_D = 2.0          # D constant of the vertical PID.
K_ROLL_P = 50.0             # P constant of the roll PID.
K_PITCH_P = 30.0            # P constant of the pitch PID.
K_YAW_P = 2.0
K_X_VELOCITY_P = 1
K_Y_VELOCITY_P = 1
K_X_VELOCITY_I = 0.01
K_Y_VELOCITY_I = 0.01
LIFT_HEIGHT = 5.0
ASCENT_RATE = 1.0           # 상승 속도 (m/s) - 점진적 상승용

print("=====mavic_driver.py loaded=====")

def clamp(value, value_min, value_max):
    return min(max(value, value_min), value_max)


class MavicDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        self.__timestep = int(self.__robot.getBasicTimeStep())

        # Sensors
        self.__gps = self.__robot.getDevice('gps')
        self.__gyro = self.__robot.getDevice('gyro')
        self.__imu = self.__robot.getDevice('inertial unit')

        # Propellers
        self.__propellers = [
            self.__robot.getDevice('front right propeller'),
            self.__robot.getDevice('front left propeller'),
            self.__robot.getDevice('rear right propeller'),
            self.__robot.getDevice('rear left propeller')
        ]
        for propeller in self.__propellers:
            propeller.setPosition(float('inf'))
            propeller.setVelocity(0)

        # State
        self.__target_twist = Twist()
        self.__vertical_ref = 0.0  # 점진적 상승을 위해 0에서 시작
        self.__linear_x_integral = 0
        self.__linear_y_integral = 0
        self.__previous_x = None
        self.__previous_y = None
        self.__previous_z = None

        # ROS interface
        rclpy.init(args=None)
        self.__node = rclpy.create_node('mavic_driver')
        robot_name = self.__robot.getName()
        self.__node.create_subscription(Twist, f'/{robot_name}/cmd_vel', self.__cmd_vel_callback, 1)

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        roll_ref = 0
        pitch_ref = 0

        # Read sensors
        roll, pitch, yaw = self.__imu.getRollPitchYaw()
        x, y, vertical = self.__gps.getValues()
        roll_velocity, pitch_velocity, twist_yaw = self.__gyro.getValues()
        
        if self.__previous_x is None:
            self.__previous_x = x
            self.__previous_y = y
            self.__previous_z = vertical
            return

        dt = self.__timestep / 1000.0
        vx_global = (x - self.__previous_x) / dt
        vy_global = (y - self.__previous_y) / dt
        vz_global = (vertical - self.__previous_z) / dt

        self.__previous_x = x
        self.__previous_y = y
        self.__previous_z = vertical

        # Calculate velocity in body frame
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        velocity_x = vx_global * cos_yaw + vy_global * sin_yaw
        velocity_y = -vx_global * sin_yaw + vy_global * cos_yaw

        # Allow high level control once the drone is lifted
        if vertical > 0.2:
            # 점진적 상승: 목표 고도까지 서서히 올림
            if self.__vertical_ref < LIFT_HEIGHT:
                self.__vertical_ref = min(self.__vertical_ref + ASCENT_RATE * dt, LIFT_HEIGHT)
            
            # High level controller (linear velocity)
            linear_y_error = self.__target_twist.linear.y - velocity_y
            linear_x_error = self.__target_twist.linear.x - velocity_x
            self.__linear_x_integral += linear_x_error
            self.__linear_y_integral += linear_y_error
            roll_ref = K_Y_VELOCITY_P * linear_y_error + K_Y_VELOCITY_I * self.__linear_y_integral
            pitch_ref = - K_X_VELOCITY_P * linear_x_error - K_X_VELOCITY_I * self.__linear_x_integral
            self.__vertical_ref = clamp(
                self.__vertical_ref + self.__target_twist.linear.z * dt,
                max(vertical - 0.5, LIFT_HEIGHT),
                vertical + 0.5
            )
        else:
            # 이륙 전: vertical_ref를 현재 고도 + 약간 위로 설정하여 부드럽게 이륙
            self.__vertical_ref = vertical + 0.5
        vertical_input = K_VERTICAL_P * (self.__vertical_ref - vertical) - K_VERTICAL_D * vz_global

        # Low level controller (roll, pitch, yaw)
        yaw_ref = self.__target_twist.angular.z

        roll_input = K_ROLL_P * clamp(roll, -1, 1) + roll_velocity + roll_ref
        pitch_input = K_PITCH_P * clamp(pitch, -1, 1) + pitch_velocity + pitch_ref
        yaw_input = K_YAW_P * (yaw_ref - twist_yaw)

        m1 = K_VERTICAL_THRUST + vertical_input + yaw_input + pitch_input + roll_input
        m2 = K_VERTICAL_THRUST + vertical_input - yaw_input + pitch_input - roll_input
        m3 = K_VERTICAL_THRUST + vertical_input - yaw_input - pitch_input + roll_input
        m4 = K_VERTICAL_THRUST + vertical_input + yaw_input - pitch_input - roll_input

        # Apply control
        self.__propellers[0].setVelocity(-m1)
        self.__propellers[1].setVelocity(m2)
        self.__propellers[2].setVelocity(m3)
        self.__propellers[3].setVelocity(-m4)


# ===== Standalone Webots Controller Execution =====
def main():
    """메인 실행 함수 - Webots에서 직접 컨트롤러로 실행될 때 사용"""
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())
    
    # GPS, Gyro, IMU 활성화
    gps = robot.getDevice('gps')
    gyro = robot.getDevice('gyro')
    imu = robot.getDevice('inertial unit')
    
    gps.enable(timestep)
    gyro.enable(timestep)
    imu.enable(timestep)
    
    # 프로펠러 초기화
    propellers = [
        robot.getDevice('front right propeller'),
        robot.getDevice('front left propeller'),
        robot.getDevice('rear right propeller'),
        robot.getDevice('rear left propeller')
    ]
    for propeller in propellers:
        propeller.setPosition(float('inf'))
        propeller.setVelocity(0)
    
    # ROS2 초기화
    rclpy.init(args=None)
    node = rclpy.create_node('mavic_driver')
    robot_name = robot.getName()
    
    # 상태 변수
    target_twist = Twist()
    vertical_ref = 0.0  # 점진적 상승을 위해 0에서 시작
    linear_x_integral = 0
    linear_y_integral = 0
    previous_x = None
    previous_y = None
    previous_z = None
    
    def cmd_vel_callback(twist):
        nonlocal target_twist
        target_twist = twist
    
    node.create_subscription(Twist, f'/{robot_name}/cmd_vel', cmd_vel_callback, 1)
    
    print(f"=====Mavic driver started for {robot_name}=====")

    # WBT 파일의 controllerArgs로부터 목표 고도 설정
    target_lift_height = LIFT_HEIGHT
    if len(sys.argv) > 1:
        try:
            target_lift_height = float(sys.argv[1])
            print(f"Set target lift height to {target_lift_height}m from controllerArgs")
        except ValueError:
            print(f"Invalid lift height argument: {sys.argv[1]}, using default {LIFT_HEIGHT}m")
    else:
        print(f"No controllerArgs provided, using default lift height {LIFT_HEIGHT}m")
    
    # 메인 루프

    while robot.step(timestep) != -1:
        rclpy.spin_once(node, timeout_sec=0)
        
        roll_ref = 0
        pitch_ref = 0
        
        # 센서 읽기
        roll, pitch, yaw = imu.getRollPitchYaw()
        x, y, vertical = gps.getValues()
        roll_velocity, pitch_velocity, twist_yaw = gyro.getValues()
        
        if previous_x is None:
            previous_x = x
            previous_y = y
            previous_z = vertical
            continue
        
        dt = timestep / 1000.0
        vx_global = (x - previous_x) / dt
        vy_global = (y - previous_y) / dt
        vz_global = (vertical - previous_z) / dt
        
        previous_x = x
        previous_y = y
        previous_z = vertical
        
        # Body frame velocity
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        velocity_x = vx_global * cos_yaw + vy_global * sin_yaw
        velocity_y = -vx_global * sin_yaw + vy_global * cos_yaw
        
        if vertical > 0.2:
            # 점진적 상승: 목표 고도까지 서서히 올림
            if vertical_ref < target_lift_height:
                vertical_ref = min(vertical_ref + ASCENT_RATE * dt, target_lift_height)
            
            linear_y_error = target_twist.linear.y - velocity_y
            linear_x_error = target_twist.linear.x - velocity_x
            linear_x_integral += linear_x_error
            linear_y_integral += linear_y_error
            roll_ref = K_Y_VELOCITY_P * linear_y_error + K_Y_VELOCITY_I * linear_y_integral
            pitch_ref = - K_X_VELOCITY_P * linear_x_error - K_X_VELOCITY_I * linear_x_integral
            vertical_ref = clamp(
                vertical_ref + target_twist.linear.z * dt,
                max(vertical - 0.5, target_lift_height),
                vertical + 0.5
            )
        else:
            # 이륙 전: vertical_ref를 현재 고도 + 약간 위로 설정하여 부드럽게 이륙
            vertical_ref = vertical + 0.5
        
        vertical_input = K_VERTICAL_P * (vertical_ref - vertical) - K_VERTICAL_D * vz_global
        
        # Low level control
        yaw_ref = target_twist.angular.z
        
        roll_input = K_ROLL_P * clamp(roll, -1, 1) + roll_velocity + roll_ref
        pitch_input = K_PITCH_P * clamp(pitch, -1, 1) + pitch_velocity + pitch_ref
        yaw_input = K_YAW_P * (yaw_ref - twist_yaw)
        
        m1 = K_VERTICAL_THRUST + vertical_input + yaw_input + pitch_input + roll_input
        m2 = K_VERTICAL_THRUST + vertical_input - yaw_input + pitch_input - roll_input
        m3 = K_VERTICAL_THRUST + vertical_input - yaw_input - pitch_input + roll_input
        m4 = K_VERTICAL_THRUST + vertical_input + yaw_input - pitch_input - roll_input
        
        # 프로펠러 제어
        propellers[0].setVelocity(-m1)
        propellers[1].setVelocity(m2)
        propellers[2].setVelocity(m3)
        propellers[3].setVelocity(-m4)
    
    rclpy.shutdown()


if __name__ == "__main__":
    main()
