#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial

class ForkliftNode(Node):

    forklift_position_topic_name = "pos_fork"

    serial_port = "/dev/ttyUSB0"
    serial_baudrate = 9600

    def __init__(self):
        super().__init__("listener")
        
        self.read_params()
        try:
            self.arduino = serial.Serial(
                self.serial_port, self.serial_baudrate, timeout=0.5
            )
        except serial.SerialException:
            self.get_logger().error(
                f"Could not open serial port {self.serial_port}"
            )
            exit()

        self.init_sub()

        self.homed = False

    def read_params(self):
        self.declare_parameter("serial_port", self.serial_port)
        self.declare_parameter("serial_baudrate", self.serial_baudrate)
        self.declare_parameter("forklift_position_topic_name", self.forklift_position_topic_name)

        self.serial_port = (
            self.get_parameter("serial_port")
            .get_parameter_value()
            .string_value
        )
        self.serial_baudrate = (
            self.get_parameter("serial_baudrate")
            .get_parameter_value()
            .integer_value
        )
        self.forklift_position_topic_name = (
            self.get_parameter("forklift_position_topic_name")
            .get_parameter_value()
            .string_value
        )
    
    def init_sub(self):
        self.subscription = self.create_subscription(Float32, "/fork_pos", self.callbackFunc, 10)

    def homingProcedure(self):
        self.get_logger().info(f"Homing forklift")
        data = "h"
        self.arduino.write(data.encode())
        while True:
            response = self.arduino.readline().decode().strip()
            if response == "done":
                self.homed = True
                self.get_logger().info("Forklift homed successfully.")
                break
        
    def posProcedure(self, pos):
        self.get_logger().info(f"Setting forklift position to: {pos}")
        data = f"p {pos}"
        self.arduino.write(data.encode())
        while True:
            response = self.arduino.readline().decode().strip()
            if response == "done":
                self.get_logger().info("Forklift moved successfully.")
                break

    def callbackFunc(self, message):
        if not self.homed:
            self.homingProcedure()
        self.posProcedure(message.data)

def main(args=None):
    rclpy.init(args=args)
    Listener = ForkliftNode()
    rclpy.spin(Listener)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
