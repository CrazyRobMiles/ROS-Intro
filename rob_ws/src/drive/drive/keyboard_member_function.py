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

class KeyboardTeleop(Node):

    def __init__(self):
        super().__init__('keyboard_teleop')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # Poll at 10 Hz
        self.get_logger().info("Keyboard Teleop Node Running: Use WASD keys to control the robot")
        # Save terminal settings
        self.settings = termios.tcgetattr(sys.stdin)

    def timer_callback(self):
        key = self.get_key()
        if key == 'w':
            self.get_logger().info("w - forward")
            self.publish_twist(1.0, 0.0)  # Forward
        elif key == 's':
            self.get_logger().info("s - backward")
            self.publish_twist(-1.0, 0.0)  # Backward
        elif key == 'a':
            self.get_logger().info("a - left turn")
            self.publish_twist(0.0, 1.0)  # Left Turn
        elif key == 'd':
            self.get_logger().info("a - right turn")
            self.publish_twist(0.0, -1.0)  # Right Turn
        elif key == ' ':
            self.get_logger().info("space - stop")
            self.publish_twist(0.0, 0.0)  # Stop
        elif key == '\x03':  # CTRL-C
            self.destroy_node()
            rclpy.shutdown()

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
    self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    teleop_node = KeyboardTeleop()

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

