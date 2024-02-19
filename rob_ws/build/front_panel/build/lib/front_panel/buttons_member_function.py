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

import random

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class ButtonReader():

   def __init__(self):
        self.buttonCount = 0    
   
   def scan_buttons(self):
        if self.buttonCount >1000:
            return ""
        
        self.buttonCount = self.buttonCount+1

        buttonNames = ["","","","","","","","start","stop","help","shutdown"];

        r = random.choice(buttonNames);
        return r

class ButtonsPublisher(Node):

    def start_publisher(self):
        self.publisher_ = self.create_publisher(String, 'buttons', 10)

    def start_timer(self):
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def __init__(self):
        super().__init__('button_publisher')
        self.buttonReader = ButtonReader()
        self.start_publisher()
        self.start_timer()

    def timer_callback(self):

        button = self.buttonReader.scan_buttons()

        if button!="":
            msg = String()
            msg.data = button
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    buttons_publisher = ButtonsPublisher()

    rclpy.spin(buttons_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    buttons_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
