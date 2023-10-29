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
import sys
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('pmove')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 1)
        self.subscriber_= self.create_subscription(String, 'cmd_text', self.publish_message, 1)

    def publish_message(self, msg):
        message = Twist()
        msg = msg.data
        if(msg=="turn_right"):
        	message.angular.z = -1.5
        elif(msg=="turn_left"):
        	message.angular.z = 1.5
        elif(msg=="move_forward"):
        	message.linear.x = 1.0
        elif(msg=="move_backward"):
        	message.linear.x = -1.0
        else:
        	msg = 'Unknown command'
        self.get_logger().info('Next moving: %s' % (msg))
        self.publisher_.publish(message)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
