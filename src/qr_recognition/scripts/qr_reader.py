#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
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
        self.capture = cv2.VideoCapture(0)
        if not (self.capture.isOpened()):
            self.get_logger().info('Could not open camera')

        # Get camera width and height
        self.cameraWidth = int(self.capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.cameraHeight = int(self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.cameraCenter = [self.cameraWidth / 2, self.cameraHeight / 2]

        #Create a bridge to allow images to go between cv2 and ROS
        self.bridge = CvBridge()

        #This publisher will give the data from the image
        #self.publisher = self.create_publisher(Image, 'frames', 10)
        self.movementPublisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.forkliftPublisher = self.create_publisher(Float32, '/fork_pos', 10)
        self.timer = self.create_timer(0.01, self.timerCallback)

        #This subscriber will get the image from the camera
        self.subscription = self.create_subscription(Image, 'frames', self.imgCallback, 10)
        self.imageFound = False
        self.decodeMsg
        self.subscription
        self.bridge = CvBridge()

    # This function reads the image and image data from the camera and decodes it
    def imgCallback(self, data):
        self.get_logger().info('Getting frame')
        currentImage = self.bridge.imgmsg_to_cv2(data) # Converts the ros2 image to an opencv image
        self.decodeMsg = decode(currentImage) # Decodes the image

        if(self.decodeMsg):
            self.imageFound = True
            # Store the QR information:
            self.QRLeft = self.decodeMsg.rect[0]
            self.QRTop = self.decodeMsg.rect[1]
            self.QRWidth = self.decodeMsg.rect[2]
            self.QRHeight = self.decodeMsg.rect[3]
            if(self.isQRCodeCentered() == True):
                self.driveToQRCode()
                self.raiseForkLift()
        elif(not self.decodeMsg and self.imageFound == True):
            self.imageFound = False
        cv2.waitKey(3)

    # This function will publish the image data to external nodes
    def timerCallback(self):
        ret, frame = self.capture.read()
        #If the camera sees anything it will return true
        if ret == True:
            self.movementPublisher.publish(self.bridge.cv2_to_imgmsg(frame))
        self.get_logger().info('Publishing frame')

    def isQRCodeCentered(self):
        QRCodeCenter = [self.QRWidth / 2, self.QRHeight / 2]

        # Check if the QR code and camera are centered based on the top left corner of the QR code
        # If it's not centered then center it
        if(self.QRTop == self.cameraCenter[1] - QRCodeCenter[1] and self.QRLeft == self.cameraCenter[0] - QRCodeCenter[0]):
            return True
        else:
            # Determine which axis needs to change
            topVal = self.cameraCenter[1] - QRCodeCenter[1]
            leftVal = self.cameraCenter[0] - QRCodeCenter[0]
            driveMsg = Twist()
            raiseMsg = Float32()
            if(self.QRTop != topVal):
                if (self.QRTop > topVal):
                    # Move Up
                    raiseMsg.data = 0.05
                    self.forkliftPublisher.publish(raiseMsg)
                elif (self.QRTop < topVal):
                    # Move down
                    raiseMsg.data = -0.05
                    self.forkliftPublisher.publish(raiseMsg)
            elif(self.QRLeft != leftVal):
                if(self.QRLeft > leftVal):
                    # Move left
                    driveMsg.linear.x = -0.5
                    self.movementPublisher.publish(driveMsg)
                elif(self.QRLeft < leftVal):
                    # Move Right
                    driveMsg.linear.x = 0.5
                    self.movementPublisher.publish(driveMsg)
            return False

    def driveToQRCode(self):
        moveForewardMsg = Twist()
        moveForewardMsg.linear.x = 0.25
        while(self.QRWidth <=  3 * self.cameraWidth / 4 and self.QRHeight <= 3 * self.cameraHeight / 4):
            self.movementPublisher.publish(moveForewardMsg)
        # Once the QR code is taking up most of the frame go foreward an appoximate distance
        endTime = time.time() + 3 # Run for 3 seconds
        while(currentTime < endTime):
            currentTime = time.time()

    def raiseForkLift(self):
        forkLiftMsg = Float32()
        forkLiftMsg.data = 1.0
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