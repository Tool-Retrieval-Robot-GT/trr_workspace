#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import String

comLine = serial.Serial('/dev/ttyUSB0') # Change when device is connected


class motorDriverComm(Node):
    def __init__(self):
        super().__init__('motorDriverControl')

        # Create a publisher to update log
        self.publisher = self.create_publisher(String, 'Motor Communication', 10)

        # Create a timer
        self.timer = self.create_timer(0.01, self.timer_callback)

    def handleInput(self):
        print("Type 0 for automatic control with PID or 1 for automatic user control")
        chosenOption = int(input())
        comLine.write(chosenOption)
        if(chosenOption == 0):
            print("Please input a speed to set (up to 255):")
            chosenSpeed = int(input())
            comLine.write(chosenSpeed)
            print("Type 1 at any time to break out of automatic control")
            while(int(input()) != 1):
                continue
            
        elif(chosenOption == 1):
            # Get the motor from the user
            print("Please type the associated number to the motor you wish to control:")
            print("1. Front Left\n2. Back Left\n3. Front Right\n 4. Back Right")
            chosenMotor = int(input())
            comLine.write(chosenMotor)
            print(chosenMotor)

            # Get the function to call on the motor
            print("Please type the associated number to the function you wish to call:")
            print("1. Move foreward\n2. Move backward\n3. Control Speed")
            chosenFunction = int(input())
            print(chosenFunction)
            #comLine.write(chosenFunction)

            # If the user wants to set the speed
            if(chosenFunction == 3):
                print("Please type the speed you wish to set the motor to (Between range of 0 and 255):")
                motorSpeed = int(input())
                if(motorSpeed > 255):
                    print("Invalid input. Please specify a number between 0 and 255")
                else:
                    comLine.write(motorSpeed)

def main(args=None):
    rclpy.init(args=args)
    motorCommSystem = motorDriverComm
    while rclpy.ok():
        motorDriverComm.handleInput(motorCommSystem)
        print("Here")
        #rclpy.spin(motorDriverComm)
        print("here")
    motorDriverComm.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()