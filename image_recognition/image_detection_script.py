import cv2 as cv
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from image_recognition.msg import ThreePairCoords

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
        self.publisher_= self.create_publisher(ThreePairCoords, 'positions', 10)

        # Create a subscriber to the image topic
        self.subscription = self.create_subscription(
            Image, 'image_topic', self.imageDetection, 10)

        # Initialize CvBridge for image conversion
        self.bridge = CvBridge()

        self.get_logger().info('image detection node has been started')

    def imageDetection(self, msg):
        """Callback function for processing incoming images."""

        try:
            frame = self.bridge.imgmsg_to_cv2(msg)
        except Exception as e:
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
        # Get the largest contour found as this is likely the wanted shape
            largest_contour = max(contours, key=cv.contourArea)
            
            try:
                # Get the convex hull of a point set to detect angle point
                hull = cv.convexHull(largest_contour,returnPoints = False)
                # Get all the examples of convexity
                defects = cv.convexityDefects(largest_contour,hull)
                
                # We want to keep the furthest point so track params
                maxDefect = [0,0,0,0]
                
                # If there are defects
                if(defects is not None):
                    # For all defects (convex points)
                    for i in range(defects.shape[0]):
                        # Get relevant points [start, end, far, distance]
                        # start: index (of start of line of convexity),
                        # end: index (of end of line with convexity),
                        # far: convex point,
                        # distance: distance of far from line of convexity
                        test = defects[i,0]

                        # If distance is greater than current max
                        if(test[3] > maxDefect[3]):
                            ## far = tuple(largest_contour[test[2]][0])
                            # Update max to new largest
                            maxDefect = test 
                    
                    
                    # Get the relevant points for the start, end, and far point
                    start = tuple(largest_contour[maxDefect[0]][0])
                    end = tuple(largest_contour[maxDefect[1]][0])
                    far = tuple(largest_contour[maxDefect[2]][0])
                    
                    # Publish the points
                    msg = ThreePairCoords()
                    msg.endPoint1[0] = start[0]
                    msg.endPoint1[1] = start[1]
                    msg.endPoint2[0] = end[0]
                    msg.endPoint2[1] = end[1]
                    msg.midPoint[0] = far[0]
                    msg.midPoint[1] = far[1]
                    self.publisher_.publish(msg)

            # If error from opencv then skip this frame
            except cv.error:
                self.get_logger().error("Error processing contours")
                return

            #### OLD CODE ####

            # # Get a bounding rectangle
            # x, y, w, h = cv.boundingRect(largest_contour)
            
            # # Fit line to the largest contour
            # [vx, vy, x, y] = cv.fitLine(largest_contour, cv.DIST_L2, 0, 0.01, 0.01)

            # # Calculate the angle
            # angle = np.arctan2(vy, vx) * 180 / np.pi # Convert to degrees

            # # Convert it from numpy array
            # angle = float(angle)

            # # Publish the angle
            # msg = Float64()
            # msg.data = angle
            # self.publisher_.publish(msg)


def main(args = None):
    rclpy.init(args=args)
    node = imageDetectionPkg()
    rclpy.spin(node)
    rclpy.shutdown()