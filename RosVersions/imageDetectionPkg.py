import cv2 as cv
import numpy as np
import rclpy 
from rclpy.node import Node
from std_msgs.msg import Float64
if __debug__:
    from compass import *

# Script to get a direction from a orange oblong
# Uses opencv to process the image and get a "direction"
# Currently only supports a single direction averaging the oblong

# Adds a pygame compass to indicate direction if ran `python -O orange_oblong_detection.py`

# TODO: Support multi segment oblong.

# Get capture from camera
cap = cv.VideoCapture(0)

#set up node stuff
class imageDetectionPkg(Node): 
    def __init__(self):
        super().__init__('image_detection_node')
        #create the publisher stuff so that it returns a float representing an angle
        self.publisher_= self.create_publisher(Float64, 'angle', 10)
        self.get_logger().info('image detection node has been started')
    
    
    def imageDetection(self):
    
            # Get frame
            frameReadSuccess , frame = cap.read()
            if(frameReadSuccess): #check if the frame read was a success
                # Flip horizontally
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

                # if any contours:
                if len(contours) > 0:
                    # cnt = contours[0]  
                    # Get Largest Contour
                    largest_contour = max(contours, key=cv.contourArea)

                    # Get a bounding rectangle
                    x, y, w, h = cv.boundingRect(largest_contour)
                    # Draw the rectangle in green
                    cv.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)


                    # rect = cv.minAreaRect(largest_contour)
                    # box = cv.boxPoints(rect)
                    # box = np.int0(box)
                    # cv.drawContours(frame, [box], 0, (0, 0, 255), 2)

                    # Get a closer polygon to match multi-segments
                    epsilon = 0.02*cv.arcLength(largest_contour,True)
                    approx = cv.approxPolyDP(largest_contour,epsilon,True)
                    # Draw the polygon in red
                    cv.drawContours(frame, [approx], -1, (255, 0, 0), 2)

                    # Get bounds for calculating line
                    camera_width = int(cap.get(cv.CAP_PROP_FRAME_WIDTH)) * 20
                    camera_height = int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))

                    # * Honestly I forget 
                    rows,cols = frame.shape[:2]
                    # Get a fit line for the largest contour
                    # This is a generic line, does not check for segments
                    [vx,vy,x,y] = cv.fitLine(largest_contour, cv.DIST_L2,0,0.01,0.01)
                    # * Print line coords to ensure sanity
                    print(f"y is {y}, x is {x}, vy is {vy}, vx is {vx}")

                    # Get the slope
                    m = vy/vx
                    # Make sure slope isn't too extreme
                    #   ! Without this there are crashes from extreme slopes
                    m = min(max(m, -25), 25)


                    #get the angle and convert to a ros2 compatible message
                    angle = calculate_angle(vx, vy)
                    msg = Float64()
                    msg.data = angle
                    self.publisher_.publish(msg)


                    # # Calculate left and right end points of line
                    # lefty = int((-x*m) + y)
                    # righty = int(((cols-x)*m)+y)

                    # # Ensure lefty & righty are ints
                    # #   Should not be needed
                    # lefty = lefty if isinstance(lefty, int) else 1
                    # righty = righty if isinstance(righty, int) else 1

                    # # Ensure sides are within normal bounds
                    # lefty = max(lefty, -camera_width)
                    # lefty = min(lefty, camera_width)
                    # righty = max(righty, -camera_width)
                    # righty = min(righty, camera_width)

                    # # * I do not know again
                    # colMin1 = cols-1 or 0
                    # colMin1 = colMin1 if isinstance(colMin1, int) else 1

                    # print()

                    # Actually draw the line
                    # cv.line(frame,(colMin1,righty),(0,lefty),(0,255,0),2)

                    # return ((colMin1, righty), (0, lefty))

                # Show the edits
                # cv.imshow('Contours', frame)
                # cv.imshow('Mask', mask)
                # cv.imshow('Smoothed', mask_smoothed)
                
            # Exit
            # cv.destroyAllWindows()
            cap.release()


def main(args = None):
    rclpy.init(args=args)

    node = imageDetectionPkg()

    rclpy.spin(node)

    rclpy.shutdown()