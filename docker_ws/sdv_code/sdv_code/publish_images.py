import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import time

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, '/carla/ego_vehicle/rgb_front/image', 10)
        self.bridge = CvBridge()
        self.image_directory = "/home/ws/src/sdv_code/images/"  # Set your image folder path
        self.index = 0
        self.timer = self.create_timer(3.0, self.publish_image)

    def publish_image(self):
        self.get_logger().info(f"Publishing")
        #Aqui tu codigo para la practica 7 
def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
