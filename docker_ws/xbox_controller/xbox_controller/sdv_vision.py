import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2


class ObjectLocator(Node):
    def __init__(self):
        super().__init__('object_locator')

        # Subscriptions for segmentation and depth images
        self.create_subscription(Image, '/carla/semantic_segmentation_camera/image', self.segmentation_callback, 10)
        self.create_subscription(Image, '/carla/depth_camera/image', self.depth_callback, 10)

        self.bridge = CvBridge()
        self.segmentation_image = None
        self.depth_image = None

    def segmentation_callback(self, msg):
        """Process segmentation images."""
        # Convert ROS image to OpenCV format
        self.segmentation_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.segmentation_image = cv2.cvtColor(self.segmentation_image, cv2.COLOR_BGR2RGB)

        # Example: Detect object with ID 10 (e.g., cars in CARLA)
        object_id = 10
        mask = (self.segmentation_image[:, :, 0] == object_id)

        # Visualize the segmentation mask
        mask_visualization = np.zeros_like(self.segmentation_image)
        mask_visualization[mask] = [255, 0, 0]  # Highlight detected object in red
        cv2.imshow("Segmentation Mask", mask_visualization)
        cv2.waitKey(1)

        # Check if depth image is available
        if self.depth_image is not None:
            self.predict_3d_location(mask)

    def depth_callback(self, msg):
        """Process depth images."""
        # Convert ROS image to OpenCV format
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def predict_3d_location(self, mask):
        """Predict 3D location of the detected object."""
        if self.depth_image is None or self.segmentation_image is None:
            return

        # Get pixel coordinates of the object
        indices = np.argwhere(mask)
        if len(indices) == 0:
            return

        # Get the center of the object in pixel coordinates
        pixel_y, pixel_x = np.mean(indices, axis=0).astype(int)

        # Retrieve depth value (distance in meters)
        depth = self.depth_image[pixel_y, pixel_x]

        # Project pixel coordinates to 3D world coordinates
        # Assuming intrinsic parameters of the camera
        fov = 90  # Example field of view
        width = self.segmentation_image.shape[1]
        height = self.segmentation_image.shape[0]
        focal_length = width / (2 * np.tan(np.radians(fov / 2)))

        # Convert pixel to camera coordinates
        x = (pixel_x - width / 2) * depth / focal_length
        y = (pixel_y - height / 2) * depth / focal_length
        z = depth

        self.get_logger().info(f"Object 3D Location: X={x:.2f}, Y={y:.2f}, Z={z:.2f}")


def main(args=None):
    rclpy.init(args=args)
    object_locator = ObjectLocator()
    rclpy.spin(object_locator)
    object_locator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
