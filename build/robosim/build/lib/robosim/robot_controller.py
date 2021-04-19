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
from pynput import keyboard
from goto import with_goto

#motion combinations
forward=set({keyboard.Key.up})
reverse=set({keyboard.Key.down})
left=set({keyboard.Key.left})
right=set({keyboard.Key.right})
speed_up = set({keyboard.Key.ctrl,keyboard.Key.up})
speed_down = set({ keyboard.Key.ctrl,keyboard.Key.down})
auto_speed_hold= set({keyboard.Key.space, keyboard.Key.ctrl,keyboard.Key.up})
turn_left_forward= set({keyboard.Key.left,keyboard.Key.up})
turn_right_forward= set({keyboard.Key.right,keyboard.Key.up})
turn_left_reverse= set({keyboard.Key.left, keyboard.Key.down})
turn_right_reverse= set({keyboard.Key.right,keyboard.Key.down })

class RobotController(Node):
    def __init__(self):
        super().__init__('controller')
        self.publisher_ = self.create_publisher(Twist, 'jCar/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.KeyboardReader)
        self.i = 0.0
        self.linear_x= 0.0
        self.linear_y= 0.0
        self.angular_z= 0.0
        self.current=set()
        self.last_linear_x = -0.4

    def KeyboardReader(self):
        @with_goto
        def on_press(key):
            #self.get_logger().info("Starting 2")
            self.angular_z= 0.0 # nutral position
            self.linear_x = self.last_linear_x # hold last speed
            self.current.add(key)
            if self.current == forward:
                self.linear_x = -0.2 #constant forward speed
                goto .end
            if self.current == reverse:
                self.linear_x = -0.48 #constant reverse speed
                goto .end
            if self.current == left:
                self.angular_z= -1.0
                goto .end
            if self.current == right:
                self.angular_z= 1.0
                goto .end
            if self.current == speed_up:
                if self.linear_x < 0.0:    #max forward speed limit
                    self.linear_x += 0.001
                self.last_linear_x =self.linear_x 
                goto .end
            if self.current == speed_down:
                if self.linear_x > -0.5:   #max reverse speed limit
                    self.linear_x -= 0.001
                self.last_linear_x =self.linear_x 
                goto .end
            if self.current == auto_speed_hold:
                last_linear_x = self.linear_x 
                goto .end
            if self.current == turn_left_forward:
                self.linear_x = -0.2 #constant forward speed
                self.angular_z= -1.0
                goto .end
            if self.current == turn_right_forward:
                self.linear_x = -0.2 #constant forward speed
                self.angular_z= 1.0
                goto .end
            if self.current == turn_left_reverse:
                self.linear_x = -0.48 #constant reverse speed
                self.angular_z= -1.0
                goto .end
            if self.current == turn_right_reverse:
                self.linear_x = -0.48 #constant reverse speed
                self.angular_z= 1.0
                goto .end
            if key == keyboard.Key.esc:
                # Stop listener
                return False
            self.last_linear_x= -0.4 # nutral position if no key press
            label .end
            cmd = Twist()
            cmd.linear.x = self.linear_x
            cmd.linear.y = self.linear_y
            cmd.angular.z = self.angular_z
            self.publisher_.publish(cmd)
            self.get_logger().info('Publishing: "%s"' % cmd)
        def on_release(key):
            try:
                self.current.remove(key)
            except KeyError:
                pass
            
        self.get_logger().info("Starting 0")
        # Collect events until released
        with keyboard.Listener(
                on_press=on_press,on_release=on_release) as listener:
            self.get_logger().info("Starting 1")
            listener.join()

def main(args=None):
    rclpy.init(args=args)

    controller = RobotController()

    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
