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


        # 0.42 is a good height for the testing table this project is utilizing
        self.forkMsg = Float32()
        self.velocityMsg = Twist()
        self.moveForewardMsg = Twist()

        self.forkMsg.data = 0.5
        self.velocityMsg.angular.z = 0.0
        self.moveForewardMsg.linear.x = 0.25

        self.lastForkMsg = 0.42
        self.lastVelocity = 0.0

    # This function reads the image and image data from the camera and decodes it
    def imgCallback(self, data):
        self.get_logger().info('Getting frame')
        currentImage = self.bridge.imgmsg_to_cv2(data) # Converts the ros2 image to an opencv image
        self.decodeMsg = decode(currentImage) # Decodes the image and stores the data
        #self.decodeMsg = decodedMsg # <--- Delete this

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
            self.timeToSend = 0.05 # Send data every 0.05 seconds

            # Store the QR information:
            for object in self.decodeMsg:
                QRLeft, QRTop, self.QRWidth, self.QRHeight = object.rect # QRWidth and QRHeight are needed in another function

            # The resulting rectangular coordinates are relative to the camera frame
            # To get the center of the QR code explicitly you need to start from it's bounds and then add the length or width divided by 2 
            self.QRCodeCenter = [QRLeft + (self.QRWidth / 2), QRTop + (self.QRHeight / 2) + 20] # The plus 40 is the get the forklift under the the QR Code

            # Do something with that information
            if not self.isCentered:
                if(self.isQRCodeCentered() == True):
                    self.isCentered = True
            
            if(self.isCentered == True and self.atBin == False):
                if(self.driveToQRCode() == True):
                    self.atBin == True

            if(self.isCentered and self.atBin):
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
        # If the QR Code is not centered then center it
        if(self.cameraCenter[0] == self.QRCodeCenter[0] and self.cameraCenter[1] == self.QRCodeCenter[1]):
            return True
        else:
            if(self.isHomed == False):
                self.forkliftPublisher.publish(self.forkMsg)
                time.sleep(10) # Give it time to home
                self.isHomed = True

            # Determine which axis needs to change
            topVal = self.cameraCenter[1] - self.QRCodeCenter[1]
            leftVal = self.cameraCenter[0] - self.QRCodeCenter[0]

            # Adjust the height of the forklift
            if(self.cameraCenter[1] != self.QRCodeCenter[1]):
                if (topVal > 0):
                    # Move the camera (forklift) up
                    self.forkMsg.data += 0.05
                elif (topVal < 0):
                    # Move the camera (forklift) down
                    self.forkMsg.data -= 0.05

                self.forkliftPublisher.publish(self.forkMsg)

            # Adjust the robot to be more centered
            if(self.cameraCenter[0] != self.QRCodeCenter[0]):
                if(leftVal > 0):
                    # Move robot left
                    self.velocityMsg.angular.z -= 0.4
                elif(leftVal < 0):
                    # Move robot right
                    self.velocityMsg.angular.z += 0.4

                self.movementPublisher.publish(self.velocityMsg)

            # Tell the node it isn't centered yet
            return False

    """
    This function will move the robot closer to the QR Code until it is taking up the majority of the screen
    in which the robot will begin to move forward a little bit more before stopping.
    """
    def driveToQRCode(self):
        if not (self.QRWidth <=  3 * self.cameraWidth / 4 and self.QRHeight <= 3 * self.cameraHeight / 4):
            self.movementPublisher.publish(self.moveForewardMsg)
            return False
        else:
            # Once the QR code is taking up most of the frame go foreward an appoximate distance
            endTime = time.time() + 1 # Run for 1 second
            while(currentTime < endTime):
                currentTime = time.time()
            return True

    """
    This function will raise the forklift so the bin can be taken away
    """
    def raiseForkLift(self):
        forkLiftMsg = Float32()
        forkLiftMsg.data = 0.7
        # Raise the bin to max height
        self.forkliftPublisher.publish(forkLiftMsg)

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
