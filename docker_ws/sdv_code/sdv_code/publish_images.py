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
        self.image_list = self.load_images(self.image_directory)
        self.index = 0

        if not self.image_list:
            self.get_logger().error("No images found in the specified directory!")
            rclpy.shutdown()
            return
        
        self.timer = self.create_timer(3.0, self.publish_image)

    def load_images(self, directory):
        valid_extensions = ('.jpg', '.jpeg', '.png')
        return [os.path.join(directory, f) for f in sorted(os.listdir(directory)) if f.lower().endswith(valid_extensions)]

    def publish_image(self):
        if self.image_list:
            image_path = self.image_list[self.index]
            self.get_logger().info(f"Publishing: {image_path}")
            cv_image = cv2.imread(image_path)

            if cv_image is None:
                self.get_logger().error(f"Failed to read image: {image_path}")
                return

            ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            self.publisher_.publish(ros_image)

            self.index = (self.index + 1) % len(self.image_list)

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
