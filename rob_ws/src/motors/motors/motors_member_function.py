# Copyright 2016 Open Source Robotics Foundation, Inc.
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

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import os
import sys
import time
import math
import RPi.GPIO as GPIO
from smbus2 import SMBus, i2c_msg

__ADC_BAT_ADDR = 0
__SERVO_ADDR   = 21
__MOTOR_ADDR   = 31
__SERVO_ADDR_CMD  = 40

__motor_speed = [0, 0, 0, 0]
__servo_angle = [0, 0, 0, 0, 0, 0]
__servo_pulse = [0, 0, 0, 0, 0, 0]
__i2c = 1
__i2c_addr = 0x7A

GPIO.setwarnings(True)
GPIO.setmode(GPIO.BOARD)

def setMotor(index, speed):
    speed_limit = 30
    if index < 1 or index > 4:
        raise AttributeError("Invalid motor num: %d"%index)
    if index == 2 or index == 4:
        speed = speed
    else:
        speed = -speed
    index = index - 1
    speed = speed_limit if speed > speed_limit else speed
    speed = -speed_limit if speed < -speed_limit else speed
    reg = __MOTOR_ADDR + index
    
    with SMBus(__i2c) as bus:
        try:
            msg = i2c_msg.write(__i2c_addr, [reg, speed.to_bytes(1, 'little', signed=True)[0]])
            bus.i2c_rdwr(msg)
            __motor_speed[index] = speed
            
        except:
            msg = i2c_msg.write(__i2c_addr, [reg, speed.to_bytes(1, 'little', signed=True)[0]])
            bus.i2c_rdwr(msg)
            __motor_speed[index] = speed
           
    return __motor_speed[index]

def calculate_motor_speeds(twist):
    # Robot dimensions in meters
    L = 0.059  # Half the distance between front and back wheels
    W = 0.067  # Half the distance between left and right wheels
    
    # Convert twist velocities to motor speeds
    Vx = twist.linear.x
    Vy = twist.linear.y
    Vz = twist.angular.z
    
    # Calculations based on the meccanum wheel formulae
    V_FL = Vx - Vy - (L + W) * Vz
    V_FR = Vx + Vy + (L + W) * Vz
    V_RL = Vx + Vy - (L + W) * Vz
    V_RR = Vx - Vy + (L + W) * Vz
    
    # Normalize the speeds to fit within -100 to 100 range
    # This requires knowing the max possible value of V_FL, V_FR, V_RL, V_RR based on robot specs
    # For simplicity, let's assume a direct scaling for this example
    max_wheel_speed = max(abs(V_FL), abs(V_FR), abs(V_RL), abs(V_RR), 1) # Avoid division by zero
    scale = 100.0 / max_wheel_speed

    setMotor(1,int(V_FL * scale))
    setMotor(2,int(V_FR * scale))
    setMotor(3,int(V_RL * scale))
    setMotor(4,int(V_RR * scale))

def getMotor(index):
    if index < 1 or index > 4:
        raise AttributeError("Invalid motor num: %d"%index)
    index = index - 1
    return __motor_speed[index]

class MecanumChassis:
    # A = 67  # mm
    # B = 59  # mm
    # WHEEL_DIAMETER = 65  # mm

    def __init__(self, a=67, b=59, wheel_diameter=65):
        self.a = a
        self.b = b
        self.wheel_diameter = wheel_diameter
        self.velocity = 0
        self.direction = 0
        self.angular_rate = 0

    def reset_motors(self):
        for i in range(1, 5):
            setMotor(i, 0)
            
        self.velocity = 0
        self.direction = 0
        self.angular_rate = 0

    def set_velocity(self, velocity, direction, angular_rate, fake=False):
        """
        Use polar coordinates to control moving
        motor1 v1|  ?  |v2 motor2
                 |     |
        motor3 v3|     |v4 motor4
        :param velocity: mm/s
        :param direction: Moving direction 0~360deg, 180deg<--- ? ---> 0deg
        :param angular_rate:  The speed at which the chassis rotates
        :param fake:
        :return:
        """
        rad_per_deg = math.pi / 180
        vx = velocity * math.cos(direction * rad_per_deg)
        vy = velocity * math.sin(direction * rad_per_deg)
        vp = -angular_rate * (self.a + self.b)
        v1 = int(vy + vx - vp) 
        v2 = int(vy - vx + vp)
        v3 = int(vy - vx - vp)
        v4 = int(vy + vx + vp)
        if fake:
            return
        setMotor(1, v1) 
        setMotor(2, v2)
        setMotor(3, v3)
        setMotor(4, v4)
        self.velocity = velocity
        self.direction = direction
        self.angular_rate = angular_rate

    def translation(self, velocity_x, velocity_y, fake=False):
        velocity = math.sqrt(velocity_x ** 2 + velocity_y ** 2)
        if velocity_x == 0:
            direction = 90 if velocity_y >= 0 else 270  # pi/2 90deg, (pi * 3) / 2  270deg
        else:
            if velocity_y == 0:
                direction = 0 if velocity_x > 0 else 180
            else:
                direction = math.atan(velocity_y / velocity_x)  # ?=arctan(y/x) (x!=0)
                direction = direction * 180 / math.pi
                if velocity_x < 0:
                    direction += 180
                else:
                    if velocity_y < 0:
                        direction += 360
        if fake:
            return velocity, direction
        else:
            return self.set_velocity(velocity, direction, 0)

    def twist_control(self, linear_x, linear_y, angular_z):
        # Calculate scalar velocity
        scalar_velocity = math.sqrt(linear_x**2 + linear_y**2)
        
        # Calculate direction in radians and convert to degrees
        # Note: math.atan2 returns the angle in radians, so we convert it to degrees.
        # atan2(y, x) is used instead of atan(y/x) to handle x=0 and to determine the correct quadrant of the angle.
        direction_degrees = math.atan2(linear_y, linear_x) * (180.0 / math.pi)
        
        # Use angular_z directly, assuming it's already in the desired units
        # If angular_z is in radians per second and you need degrees per second, convert it
        angular_rate_degrees = angular_z * (180.0 / math.pi)
        result = self.set_velocity(scalar_velocity,direction_degrees,angular_rate_degrees)
        return result
        


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MecanumDriveSubscriber(Node):

    def __init__(self):
        super().__init__('mecanum_drive_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',  # Topic name
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        setMotor(1,60)
        time.sleep(0.5)
        setMotor(1,0)
        self.get_logger().info("Motor subscriber running")

    def listener_callback(self, msg):
        linear_x = msg.linear.x  # Forward/Backward
        linear_y = msg.linear.y  # Left/Right
        angular_z = msg.angular.z  # Rotation
        # Process the incoming velocity commands
        self.get_logger().info(f'Received command: linear_x={linear_x}, linear_y={linear_y}, angular_z={angular_z}')
        calculate_motor_speeds(msg)
        return


def main(args=None):
    rclpy.init(args=args)
    mecanum_drive_subscriber = MecanumDriveSubscriber()
    rclpy.spin(mecanum_drive_subscriber)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically when the garbage collector destroys the node object)
    mecanum_drive_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
