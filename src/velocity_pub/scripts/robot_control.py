#!/usr/bin/python3
import math
import threading
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

vel_msg = Twist()  # robot velosity
mode_selection = 4 # 1:opposite phase, 2:in-phase, 3:pivot turn 4: none

class Commander(Node):

    def __init__(self):
        super().__init__('commander')
        timer_period = 0.02

        self.vel = np.array([0,0,0,0], float) #left_front, right_front, left_rear, right_rear

        self.pub_vel = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        global vel_msg, mode_selection

        linear_x = vel_msg.linear.x
        linear_y = vel_msg.linear.y
        
        # Maximum linear velocities
        max_forward_back = 7.5
        max_right_left = 7.5

        # Normalize input velocities
        linear_x = max(min(linear_x, max_forward_back), -max_forward_back)
        linear_y = max(min(linear_y, max_right_left), -max_right_left)

        # Calculate wheel velocities
        vfl = linear_x - linear_y  # Front Left Wheel
        vfr = linear_x + linear_y  # Front Right Wheel
        vrl = linear_x - linear_y  # Back Left Wheel
        vrr = linear_x + linear_y  # Back Right Wheel

        # Determine the maximum absolute velocity for normalization
        max_velocity = max(abs(vfl), abs(vfr), abs(vrl), abs(vrr), 1)  # Ensure at least 1 for scaling

        # Normalize wheel velocities to the range [-1, 1]
        self.vel = [
            vfl,
            vfr,
            vrl,
            vrr,
        ]

        vel_array = Float64MultiArray(data=self.vel)
        self.pub_vel.publish(vel_array)

class Joy_subscriber(Node):

    def __init__(self):
        super().__init__('joy_subscriber')
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, data):
        global vel_msg, mode_selection

        if(data.buttons[0] == 1):   # in-phase # A button of Xbox 360 controller
            mode_selection = 2
        elif(data.buttons[4] == 1): # opposite phase # LB button of Xbox 360 controller
            mode_selection = 1
        elif(data.buttons[5] == 1): # pivot turn # RB button of Xbox 360 controller
            mode_selection = 3
        else:
            mode_selection = 4

        vel_msg.linear.x = data.axes[1]*7.5
        vel_msg.linear.y = data.axes[0]*7.5
        vel_msg.angular.z = data.axes[3]*10

if __name__ == '__main__':
    rclpy.init(args=None)
    
    commander = Commander()
    joy_subscriber = Joy_subscriber()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(commander)
    executor.add_node(joy_subscriber)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    rate = commander.create_rate(2)
    try:
        while rclpy.ok():
            rate.sleep()
    except KeyboardInterrupt:
        pass
    
    rclpy.shutdown()
    executor_thread.join()

