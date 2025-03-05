import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv

#!!!!!!!!!! THIS IS COPILOT CODE FOR TESTING !!!!!!!!!!!!#
class ImagePublisher(Node):
    def __init__(self):
        #copilot based code 
        super().__init__('image_publisher') #name of node
        self.publisher_ = self.create_publisher(Image, '/image_topic', 10) #topic name
        self.timer = self.create_timer(0.1, self.publish_image) #publish ever 0.1 seconds
        self.bridge = CvBridge()
        
    def publishImg(self):
        straight_img = './test_images/mask.png'
        cv_image = cv.imread()

        ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        self.publisher_.publish(ros_image)

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()