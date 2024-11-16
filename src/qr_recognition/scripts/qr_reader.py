#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
import cv2
from sensor_msgs.msg import Image
from pyzbar.pyzbar import decode

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
        self.publisher = self.create_publisher(Image, 'frames', 10)
        self.timer = self.create_timer(0.01, self.timerCallback)

        #This subscriber will get the image from the camera
        self.subscription = self.create_subscription(Image, 'frames', self.imgCallback, 10)
        self.decodeMsg
        self.subscription
        self.bridge = CvBridge()

    # This function reads the image and image data from the camera and decodes it
    def imgCallback(self, data):
        self.get_logger().info('Getting frame')
        currentImage = self.bridge.imgmsg_to_cv2(data) # Converts the ros2 image to an opencv image
        self.decodeMsg = decode(currentImage) # Decodes the image

        # Store the QR information:
        self.QRLeftCorner = self.decodeMsg.rect[0]
        self.QRTopCorner = self.decodeMsg.rect[1]
        self.QRWidth = self.decodeMsg.rect[2]
        self.QRHeight = self.decodeMsg.rect[3]

        cv2.imshow("Camera", currentImage)
        cv2.waitKey(3)

    # This function will publish the image data to external nodes
    def timerCallback(self):
        ret, frame = self.capture.read()
        #If the camera sees anything it will return true
        if ret == True:
            self.publisher.publish(self.bridge.cv2_to_imgmsg(frame))
        self.get_logger().info('Publishing frame')

    def isQRCodeCentered(self):
        QRCodeCenter = [self.QRWidth / 2, self.QRHeight / 2]

        if(self.cameraCenter[0] == QRCodeCenter[0] and self.cameraCenter[1] == QRCodeCenter[1]):
            return True
        else:
            return False

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