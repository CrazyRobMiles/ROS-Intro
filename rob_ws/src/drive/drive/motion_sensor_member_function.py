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
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty
import serial
import json

class MotionSensor(Node):

    def get_port(self, portNames):
        result = None
        for portName in portNames:
            try:
                self.get_logger().info("Motion sensor Node string: connecting to " + portName)
                result = serial.Serial(portName, 115200)
                return result
            except :
                self.get_logger().info("    Connect failed to " + portName)
        return result

    def __init__(self):
        super().__init__('motion_sensor')
        ports = ["/dev/ttyACM0","/dev/ttyACM1","/dev/ttyACM2"]

        self.serial_port = self.get_port(ports)

        if self.serial_port==None:
            self.get_logger().info("Failed to open serial port ")
            raise Exception("Port not found")


        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # Poll at 10 Hz

        self.get_logger().info("Motion sensor Node Running")

        # Save terminal settings
        self.settings = termios.tcgetattr(sys.stdin)

    def timer_callback(self):
        if self.serial_port.in_waiting:
            data_str = self.serial_port.readline().decode('utf-8').strip()
            try:
                data_json = json.loads(data_str)
                x = float(data_json.get('x', 0))  # Convert to float
                y = float(data_json.get('y', 0))  # Convert to float

                twist = Twist()
                twist.linear.x = x
                twist.linear.y = y
                # Assuming z, angular x, y, and z are 0 or set them as needed
                self.publisher_.publish(twist)
                self.get_logger().info(f'Publishing: {twist}')
            except json.JSONDecodeError:
                self.get_logger().error('Could not decode JSON from serial data')
            except ValueError as e:
                self.get_logger().error(f'Error converting x or y to float: {e}')

def main(args=None):
    rclpy.init(args=args)
    try:
        teleop_node = MotionSensor()
    except Exception:
        rclpy.shutdown()
        return
 
    try:
        rclpy.spin(teleop_node)
    except KeyboardInterrupt:
        pass  # Allow program to exit gracefully on a keyboard interrupt
    finally:
        # Restore terminal settings here if needed
        teleop_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

