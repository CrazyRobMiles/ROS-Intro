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

class MotionSensor(Node):

    def get_port(self, portNames):
        result = None
        for portName in portNames:
            try:
                self.get_logger().info("Motion sensor Node string: connecting to " + portName)
                result = serial.Serial(portName, 115200)
            except :
                self.get_logger().info("    Connect failed to " + portName)
        return result

    def __init__(self):
        super().__init__('motion_sensor')
        ports = ["/dev/ttyACM0","/dev/ttyACM1","/dev/ttyACM2"]


        self.port = self.get_port(ports)

        if self.port==None:
            self.get_logger().info("Failed to open serial port ")
            raise Exception("Port not found")


        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        self.get_logger().info("Motion sensor Node Running")

        # Save terminal settings
        self.settings = termios.tcgetattr(sys.stdin)

    def timer_callback(self):
        key = self.get_key()

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        r, _, _ = select.select([sys.stdin], [], [], 0.1)
        if r:
            key = sys.stdin.read(1)
        else:
            key = ''
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def publish_twist(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.publisher_.publish(msg)

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

