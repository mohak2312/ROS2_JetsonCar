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
import time

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
        self.linear_x= -0.4
        self.linear_y= 0.0
        self.angular_z= 0.0
        self.current=set()
        self.last_linear_x = -0.4
        self.reverse_flag=True
        self.forward_flag=True
        self.auto_hold_flag=False
        self.get_logger().info('|--------------- Start Talking (-_-) ---------------|')

    def KeyboardReader(self):

        def send_cmd(speed,turn):
            cmd = Twist()
            cmd.linear.x = speed
            cmd.angular.z = turn
            self.publisher_.publish(cmd)
            self.get_logger().info('Speed: "%s" ----- Turn: "%s"' % (cmd.linear.x,cmd.angular.z))
        
        @with_goto
        def on_press(key):
            
            self.current.add(key)
            
            if self.current == forward:
                self.reverse_flag=True
                if self.auto_hold_flag:
                    send_cmd(self.linear_x,90.0)
                else:
                    if self.linear_x < -0.27:
                        self.linear_x = -0.27
                    elif self.linear_x < -0.25:
                        self.linear_x += 0.005
                    send_cmd(self.linear_x,90.0)
                goto .end

            if self.current == reverse:
                if  self.reverse_flag:
                    send_cmd(-0.47,90.0)
                    self.linear_x = -0.4
                    #time.sleep(1)
                else:
                    if self.linear_x > -0.48:
                        self.linear_x -= 0.005 
                        send_cmd(self.linear_x,90.0)
                    else:
                        send_cmd(self.linear_x,90.0)
                self.reverse_flag=False
                goto .end
            
            if self.current == speed_up:
                self.linear_x += 0.005
                send_cmd(self.linear_x,90.0)
                self.auto_hold_flag=False
                goto .end

            if self.current == speed_down:
                self.linear_x -= 0.005
                send_cmd(self.linear_x,90.0)
                self.auto_hold_flag=False
                goto .end

            if self.current == auto_speed_hold:
                send_cmd(self.linear_x,90.0)
                self.auto_hold_flag=True
                goto .end

            if self.current == turn_left_forward:
                self.reverse_flag=True
                #send_cmd(-0.27,180.0)
                if self.linear_x < -0.27:
                    self.linear_x = -0.27
                elif self.linear_x < -0.25:
                    self.linear_x += 0.005
                send_cmd(self.linear_x,180.0)
                goto .end
            
            if self.current == turn_right_forward:
                self.reverse_flag=True
                #send_cmd(-0.27,0.0)
                if self.linear_x < -0.27:
                    self.linear_x = -0.27
                elif self.linear_x < -0.25:
                    self.linear_x += 0.005
                send_cmd(self.linear_x,0.0)
                goto .end

            if self.current == turn_left_reverse:
                if  self.reverse_flag:
                    send_cmd(-0.47,90.0)
                    self.linear_x = -0.4
                    #time.sleep(1)
                else:
                    if self.linear_x > -0.48:
                        self.linear_x -= 0.005 
                        send_cmd(self.linear_x,180.0)
                    else:
                        send_cmd(self.linear_x,180.0)
                self.reverse_flag=False
                goto .end
            
            if self.current == turn_right_reverse:
                if  self.reverse_flag:
                    send_cmd(-0.47,90.0)
                    self.linear_x = -0.4
                    #time.sleep(1)
                else:
                    if self.linear_x > -0.48:
                        self.linear_x -= 0.005 
                        send_cmd(self.linear_x,0.0)
                    else:
                        send_cmd(self.linear_x,0.0)
                self.reverse_flag=False
                goto .end


            label .end

            # you need to store last value for both to use when you realse button

        def on_release(key):
            try:
                self.current.remove(key)
                if key == keyboard.Key.up or key == keyboard.Key.down:
                    send_cmd(-0.4,90.0)
                    self.linear_x = -0.4
                
                if key == keyboard.Key.left or key == keyboard.Key.right:
                    send_cmd(self.linear_x, 90.0)
            except KeyError:
                pass
            
        
        # Collect events until released
        with keyboard.Listener(
                on_press=on_press,on_release=on_release) as listener:
            
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
