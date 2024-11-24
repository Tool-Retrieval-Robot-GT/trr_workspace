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

        self.get_logger().info('Here')

        #Sets the camera to capture from
        try:
            self.capture = cv2.VideoCapture(0)
        except:
            self.get_logger().info('Could not open camera')

        self.get_logger().info('Here')

        # Get camera width and height
        self.get_logger().info('Getting camera information...')
        self.cameraWidth = int(self.capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.cameraHeight = int(self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.cameraCenter = [self.cameraWidth / 2, self.cameraHeight / 2]
        self.QRCodeCenter = [0, 0]
        self.get_logger().info('Got camera information:')

        #Create a bridge to allow images to go between cv2 and ROS
        self.bridge = CvBridge()

        self.get_logger().info('Here')

        # Publishers to communicate with the appropriate topics as action is required
        self.publisher = self.create_publisher(Image, 'frames', 10)
        self.movementPublisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.forkliftPublisher = self.create_publisher(Float32, '/fork_pos', 10)
        # Timer initializes at zero seconds so it doesn't publish
        self.timeToSend = 0
        self.timer = self.create_timer(0.05, self.timerCallback)

        # This subscriber will get the image from the camera
        self.subscription = self.create_subscription(Image, 'frames', self.imgCallback, 10)
        self.imageFound = False
        self.decodeMsg = 0
        self.subscription
        self.bridge = CvBridge()

    # This function reads the image and image data from the camera and decodes it
    def imgCallback(self, data):
        self.get_logger().info('In function')
        self.get_logger().info('Getting frame')
        currentImage = self.bridge.imgmsg_to_cv2(data) # Converts the ros2 image to an opencv image
        decodedMsg = decode(currentImage) # Decodes the image
        self.decodeMsg = decodedMsg
        cv2.imshow("Camera", currentImage)
        cv2.waitKey(3)
        self.get_logger().info('Out of function')

    # This function will publish the image data to external nodes
    def timerCallback(self):
        ret, frame = self.capture.read()
        if(ret == True and self.decodeMsg > 0):
            # Publish the frame for debugging purposes
            self.movementPublisher.publish(self.bridge.cv2_to_imgmsg(frame))

            self.imageFound = True
            self.timeToSend = 0.05 # Send data every half second

            # Store the QR information:
            QRLeft = self.decodeMsg.rect[0]
            QRTop = self.decodeMsg.rect[1]
            QRWidth = self.decodeMsg.rect[2]
            QRHeight = self.decodeMsg.rect[3]

            # The resulting polygon coordinates are relative to the camera frame
            # To get the center of the QR code explicitly you need to start from it's bounds and then add the length or width divided by 2 
            self.QRCodeCenter = [QRLeft + (QRWidth / 2), QRTop + (QRHeight / 2)]

            # Do something with that information
            if(self.isQRCodeCentered() == True):
                self.driveToQRCode()
                self.raiseForkLift()
        elif(not self.decodeMsg and self.imageFound == True):
            self.imageFound = False
            self.timeToSend = 0
    """
    This function will make adjustments to center the QR code relative to the webcam.
    """
    def isQRCodeCentered(self):
        # If the QR Code is not centered then center it
        if(self.cameraCenter[0] == self.QRCodeCenter[0] and self.cameraCenter[1] == self.QRCodeCenter[1]):
            return True
        else:
            # Determine which axis needs to change
            topVal = self.cameraCenter[1] - self.QRCodeCenter[1]
            leftVal = self.cameraCenter[0] - self.QRCodeCenter[0]

            # Messages to publish
            driveMsg = Twist()
            raiseMsg = Float32()

            # Adjust the height of the forklift
            if(self.QRTop != topVal):
                if (topVal > 0):
                    # Move the camera (forklift) up
                    raiseMsg.data = 0.05
                    self.forkliftPublisher.publish(raiseMsg)
                elif (topVal < 0):
                    # Move the camera (forklift) down
                    raiseMsg.data = -0.05
                    self.forkliftPublisher.publish(raiseMsg)

            # Adjust the robot to be more centered
            elif(self.QRLeft != leftVal):
                if(self.QRLeft > leftVal):
                    # Move robot left
                    driveMsg.linear.x = -0.4
                    self.movementPublisher.publish(driveMsg)
                elif(self.QRLeft < leftVal):
                    # Move robot right
                    driveMsg.linear.x = 0.4
                    self.movementPublisher.publish(driveMsg)

            # Tell the node it isn't centered yet
            return False

    """
    This function will move the robot closer to the QR Code until it is taking up the majority of the screen
    in which the robot will begin to move forward a little bit more before stopping.
    """
    def driveToQRCode(self):
        moveForewardMsg = Twist()
        moveForewardMsg.linear.x = 0.25
        while(self.QRWidth <=  3 * self.cameraWidth / 4 and self.QRHeight <= 3 * self.cameraHeight / 4):
            self.movementPublisher.publish(moveForewardMsg)
        # Once the QR code is taking up most of the frame go foreward an appoximate distance
        endTime = time.time() + 3 # Run for 3 seconds
        while(currentTime < endTime):
            currentTime = time.time()

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
        print('Here')
    except:
        currentReader.get_logger().info('Shutting down')
    currentReader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
