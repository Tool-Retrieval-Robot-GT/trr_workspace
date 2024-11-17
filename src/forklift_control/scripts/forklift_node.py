#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial

arduino = serial.Serial('/dev/ttyACM0', 9600)

class fork_control(Node):
    def __init__(self):
        super().__init__("listener")
        self.homed = False

        self.subscription = self.create_subscription(Float32, "/fork_pos", self.callbackFunc, 10)

    def homingProcedure(self):
        self.get_logger().info(f"Homing forklift")
        data = "h"
        arduino.write(data.encode())
        while True:
            response = arduino.readline().decode().strip()
            if response == "done":
                self.homed = True
                self.get_logger().info("Forklift homed successfully.")
                break
        
    def posProcedure(self, pos):
        self.get_logger().info(f"Setting forklift position to: {pos}")
        data = f"p {pos}"
        arduino.write(data.encode())
        while True:
            response = arduino.readline().decode().strip()
            if response == "done":
                self.get_logger().info("Forklift moved successfully.")
                break

    def callbackFunc(self, message):
        if not self.homed:
            self.homingProcedure()
        self.posProcedure(message.data)

def main(args=None):
    rclpy.init(args=args)
    Listener = fork_control()
    rclpy.spin(Listener)
    rclpy.shutdown()

if __name__ == '__main__':
    main()