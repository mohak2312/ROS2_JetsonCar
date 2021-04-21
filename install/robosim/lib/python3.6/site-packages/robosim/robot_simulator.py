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
from adafruit_servokit import ServoKit
import time


class RobotSimulator(Node):

    def __init__(self):
        super().__init__('rc_jcar')
        self.kit=ServoKit(channels=16)
        self.kit.servo[0].angle = 90            # Nutral position
        time.sleep(1)
        self.kit.continuous_servo[1].throttle = -0.44
        self.subscription = self.create_subscription(
            Twist,
            'jCar/cmd_vel',
            self.move_callback,
            10)
        self.subscription  # prevent unused variable warning

    def move_callback(self, cmd):
        if(-0.49<cmd.linear.x<-0.21 and 0<=cmd.angular.z<=180):
            self.kit.servo[0].angle =cmd.angular.z           # Nutral position
            time.sleep(1)
            self.kit.continuous_servo[1].throttle = cmd.linear.x
            #time.sleep(1)
            self.get_logger().info('Speed: "%s" ----- Turn: "%s"' % (cmd.linear.x,cmd.angular.z))
        else:
            self.get_logger().info('....WARNING....:Over Limit.....Speed: "%s" ----- Turn: "%s"' % (cmd.linear.x,cmd.angular.z))

def main(args=None):
    rclpy.init(args=args)

    rc_jcar = RobotSimulator()

    rclpy.spin(rc_jcar)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rc_jcar.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
