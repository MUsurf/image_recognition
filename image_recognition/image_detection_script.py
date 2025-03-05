import cv2 as cv
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Script to get a direction from a orange oblong
# Uses opencv to process the image and get a "direction"
# Currently only supports a single direction averaging the oblong

# TODO: Support multi segment oblong.

# Get capture from camera
cap = cv.VideoCapture(0)

# Set up node stuff
class imageDetectionPkg(Node): 
    def __init__(self):
        super().__init__('image_detection_node')

        # Create a publisher for the detected angle
        self.publisher_= self.create_publisher(Float64, 'angle', 10)

        # Create a subscriber to the image topic
        self.subscription = self.create_subscription(
            Image, 'image_topic', self.imageDetection, 10)

        # Initialize CvBridge for image conversion
        self.bridge = CvBridge()

        self.get_logger().info('image detection node has been started')

    def imageDetection(self, msg):
        """Callback function for processing incoming images."""

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"Error converting image: {e}")
            return

        # Flip frame horizontally
        frame = cv.flip(frame, 1)

        # Convert BGR to HSV
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

        # Define range of blue color in HSV
        lower_color = np.array([0, 150, 150])
        upper_color = np.array([30, 255, 255])

        # Threshold the HSV image to get only blue colors
        mask = cv.inRange(hsv, lower_color, upper_color)

        kernel = np.ones((15, 15), np.uint8)
        mask_smoothed = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)

        # Look for any contours
        contours, _ = cv.findContours(mask_smoothed, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        # If any contours:
        if contours:
            # Get largest contour
            largest_contour = max(contours, key=cv.contourArea)

            # Get a bounding rectangle
            x, y, w, h = cv.boundingRect(largest_contour)
            
            # Fit line to the largest contour
            [vx, vy, x, y] = cv.fitLine(largest_contour, cv.DIST_L2, 0, 0.01, 0.01)

            # Calculate the angle
            angle = np.arctan2(vy, vx) * 180 / np.pi # Convert to degrees

            # Publish the angle
            msg = Float64()
            msg.data = angle
            self.publisher_.publish(msg)


def main(args = None):
    rclpy.init(args=args)
    node = imageDetectionPkg()
    rclpy.spin(node)
    rclpy.shutdown()