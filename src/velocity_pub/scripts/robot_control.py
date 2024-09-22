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
        max_forward_back = 20
        max_right_left = 20

        # Normalize input velocities
        linear_x = max(min(linear_x, max_forward_back), -max_forward_back)
        linear_y = max(min(linear_y, max_right_left), -max_right_left)

        # Calculate wheel velocities
        vfl = linear_x - linear_y  # Front Left Wheel
        vfr = linear_x + linear_y  # Front Right Wheel
        vrl = linear_x - linear_y  # Back Left Wheel
        vrr = linear_x + linear_y  # Back Right Wheel

        #Set the velocity array
        self.vel = [vfl, vfr, vrl, vrr]

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

        vel_msg.linear.x = data.axes[1]*20
        vel_msg.linear.y = data.axes[0]*20

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

