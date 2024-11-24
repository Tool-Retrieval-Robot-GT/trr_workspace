#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class ForkliftTeleop(Node):
    def __init__(self):
        super().__init__('fork_position_publisher')
        self.publisher_ = self.create_publisher(Float32, '/fork_pos', 10)
        self.get_logger().info("Fork position publisher node has been started.")

    def publish_position(self, value: float):
        msg = Float32()
        msg.data = value
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published value: {value}")


def main(args=None):
    rclpy.init(args=args)
    node = ForkliftTeleop()

    try:
        while rclpy.ok():
            try:
                user_input = input("Enter a float value between 0.0 and 1.0: ")
                value = float(user_input)

                if 0.0 <= value <= 1.0:
                    node.publish_position(value)
                else:
                    print("Please enter a value between 0.0 and 1.0.")
            except ValueError:
                print("Invalid input. Please enter a numeric value.")
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
