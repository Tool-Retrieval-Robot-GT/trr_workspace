#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class listener(Node):
    def __init__(self):
        super().__init__("listener")

        self.subscription = self.create_subscription(Twist, "/diff_cont/cmd_vel_unstamped", self.callbackFunc, 10)

    def callbackFunc(self, message):
        self.get_logger().info("Cmd vel out: ")
        print(message)

def main(args=None):
    rclpy.init(args=args)
    Listener = listener()
    rclpy.spin(Listener)
    rclpy.shutdown()

if __name__ == '__main__':
    main()