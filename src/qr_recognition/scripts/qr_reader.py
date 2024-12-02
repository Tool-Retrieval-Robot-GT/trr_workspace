#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image
from pyzbar.pyzbar import decode
from geometry_msgs.msg import Twist
import time
from std_msgs.msg import Float32

class QRReader(Node):
    def __init__(self):
        #Name the object
        super().__init__('camera')

        #Sets the camera to capture from
        try:
            self.capture = cv2.VideoCapture(0)
        except:
            self.get_logger().info('Could not open camera')


        # Get camera width and height
        self.get_logger().info('Getting camera information...')
        self.cameraWidth = int(self.capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.cameraHeight = int(self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.cameraCenter = [self.cameraWidth / 2, self.cameraHeight / 2]
        self.QRCodeCenter = [0, 0]
        self.QRWidth = 0
        self.QRHeight = 0
        self.QRToCameraHeightRatio = 0
        self.QRToCameraWidthRatio = 0
        self.get_logger().info('Got camera information:')

        #Create a bridge to allow images to go between cv2 and ROS
        self.bridge = CvBridge()

        # Publishers to communicate with the appropriate topics as action is required:
        # Publisher:            Topic:              Description:
        # imagePublisher        frames              Takes the image from the camera and passes it down the node
        # movementPublisher     cmd_vel             Sends movement commands to the motor arduino
        # forkliftPublisher     fork_pos            Send commands to change the forklift height
        self.imagePublisher = self.create_publisher(Image, '/frames', 10)
        self.movementPublisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.forkliftPublisher = self.create_publisher(Float32, '/fork_pos', 10)

        # Timer initializes at zero seconds so it doesn't publish when not needed
        self.timeToSend = 0
        self.timer = self.create_timer(self.timeToSend, self.timerCallback)

        # This subscriber will get the image from the camera
        self.subscription = self.create_subscription(Image, '/frames', self.imgCallback, 10)
        self.subscription

        # Variables used around the class
        self.imageFound = False
        self.isHomed = False
        self.isCentered = False
        self.atBin = False
        self.decodeMsg = []
        self.internalTimer = time.time()
        self.endTimer = time.time() + 1


        self.forkMsg = Float32()
        self.velocityMsg = Twist()
        self.moveForwardMsg = Twist()

        # 0.43 is a good height for the testing table this project is utilizing
        self.forkMsg.data = 0.43
        self.velocityMsg.angular.z = 0.0
        self.moveForwardMsg.linear.x = 0.1

    # This function reads the image and image data from the camera and decodes it
    def imgCallback(self, data):
        self.get_logger().info('Getting frame')
        currentImage = self.bridge.imgmsg_to_cv2(data) # Converts the ros2 image to an opencv image
        self.decodeMsg = decode(currentImage) # Decodes the image and stores the data

        # This code will need to be erased during implementation
        cv2.imshow("Camera", currentImage)
        cv2.waitKey(3)

    # This function will publish the image data to external nodes
    def timerCallback(self):

        # Send the current camera frame to the frames topic to determine if action is needed
        ret, frame = self.capture.read()
        if(ret == True):
            self.imagePublisher.publish(self.bridge.cv2_to_imgmsg(frame))

        if(self.decodeMsg != []):
            self.imageFound = True
            self.timeToSend = 0.5 # Send data every 0.5 seconds

            # Store the QR information:
            for object in self.decodeMsg:
                QRLeft, QRTop, self.QRWidth, self.QRHeight = object.rect # QRWidth and QRHeight are needed in another function

            self.QRToCameraWidthRatio = (int)(self.cameraWidth / self.QRWidth)
            self.QRToCameraHeightRatio = (int)(self.cameraHeight / self.QRHeight)

            # The resulting rectangular coordinates are relative to the camera frame
            # To get the center of the QR code explicitly you need to start from it's bounds and then add the length or width divided by 2 
            self.QRCodeCenter = [QRLeft + (self.QRWidth / 2), QRTop + (self.QRHeight / 2) + 15] # The plus 15 is the get the forklift under the the QR Code

            # Force the code to center first
            if (self.isCentered == False):
                if(self.isQRCodeCentered() == True):
                    self.isCentered = True
            # Once centered drive to the bin
            elif(self.isCentered == True and self.atBin == False):
                if(self.driveToQRCode() == True):
                    self.atBin = True
            # Once at bin pick it up
            elif(self.isCentered == True and self.atBin == True):
                # Obtain the bin
                self.raiseForkLift()

                # Reset the appropriate values
                self.isCentered = False
                self.atBin = False
                self.imageFound = False
                self.timeToSend = 0

        # elif(not self.decodeMsg and self.imageFound == True):
        #     self.imageFound = False
        #     self.timeToSend = 0

    """
    This function will make adjustments to center the QR code relative to the webcam.
    """
    def isQRCodeCentered(self):
        forkliftPositionCorrect = False
        robotPositionCorrect = False
        if((self.QRCodeCenter[1] < self.cameraCenter[1] + 5) and (self.QRCodeCenter[1] > self.cameraCenter[1] - 5)):
            forkliftPositionCorrect = True
        if((self.QRCodeCenter[0] < self.cameraCenter[0] + 10) and (self.QRCodeCenter[0] > self.cameraCenter[0] - 10)):
            robotPositionCorrect = True
        # If the QR Code is not centered then center it
        if(forkliftPositionCorrect == True and robotPositionCorrect == True):
            return True
        else:
            if(self.isHomed == False):
                self.forkliftPublisher.publish(self.forkMsg)
                time.sleep(15) # Give the forklift time to get it's start up information
                self.isHomed = True

            """
            [0, 0]
            + - - - - - - - - - +
            |  [L, T]           |
            |     + - - +       |
            |     | Q R |       |
            |     + - - +       |
            |                   |
            |                   |
            + - - - - - - - - - + 
                           [Width, Height]
            """

            # If the forklift isn't in the right place then attempt to fix it
            if(forkliftPositionCorrect == False):
                heightDifference = self.cameraCenter[1] - self.QRCodeCenter[1]
                if(heightDifference > 0 and self.forkMsg.data > 0.0):
                    print("Moving fork down")
                    # Move forklift down
                    self.forkMsg.data -= 0.005
                    # For the odd number decrements
                    if(self.forkMsg.data < 0.0):
                        self.forkMsg.data = 0.0
                elif(heightDifference < 0 and self.forkMsg.data < 1.0):
                    print("Moving fork up")
                    # Move forklift up
                    self.forkMsg += 0.05
                    #For the odd number increments
                    if(self.forkMsg.data > 1.0):
                        self.forkMsg.data = 1.0
                self.forkliftPublisher.publish(self.forkMsg)

            # If the robot is not centered then attempt to fix it
            # Skip for now
            """
            if(robotPositionCorrect == False):
                widthDifference = self.cameraCenter[0] - self.QRCodeCenter[0]
                if(widthDifference > 0):
                    # Move the robot right
                    self.velocityMsg.angular.z = 0.2

                elif(widthDifference < 0):
                    #Move the robot left
                    self.velocityMsg.angular.z = -0.2

                self.movementPublisher.publish(self.velocityMsg)
                time.sleep(1) # Move the robot for one second before stopping it
                self.velocityMsg.angular.z = 0.0
                self.movementPublisher.publish(self.velocityMsg)
            """

            return False

    """
    This function will move the robot closer to the QR Code until it is taking up the majority of the screen
    in which the robot will begin to move forward a little bit more before stopping.
    """
    def driveToQRCode(self):
        stopMessage = Twist()
        stopMessage.linear.x = 0.0
        print("Height Ratio:")
        print(self.QRToCameraHeightRatio)
        print("Width Ratio:")
        print(self.QRToCameraWidthRatio)

        if not (self.QRToCameraHeightRatio < 2 and self.QRToCameraWidthRatio < 2):
            # Move forward for half a second
            self.movementPublisher.publish(self.moveForwardMsg)
            time.sleep(0.25)  # Give it some time to move
            self.movementPublisher.publish(stopMessage)
            return False
        else:
            # QR Code can become unreadable when too close so send it forward an approximate distance
            self.movementPublisher(self.moveForwardMsg)
            time.sleep(0.25)
            self.movementPublisher.publish(stopMessage)
            return True

    """
    This function will raise the forklift so the bin can be taken away
    """
    def raiseForkLift(self):
        self.forkLiftMsg.data = 0.7
        # Raise the bin to max height
        self.forkliftPublisher.publish(self.forkLiftMsg)

def main(args=None):
    rclpy.init(args=args)
    currentReader = QRReader()
    try:
        rclpy.spin(currentReader)
    except:
        currentReader.get_logger().info('Shutting down')
    currentReader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()