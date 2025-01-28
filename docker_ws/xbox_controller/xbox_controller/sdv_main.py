import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from carla_msgs.msg import CarlaEgoVehicleControl
from cv_bridge import CvBridge
import cv2
import numpy as np


class LaneFollowingNode(Node):
    def __init__(self):
        super().__init__('lane_following_node')
        self.bridge = CvBridge()

        # ROS 2 Subscribers and Publishers
        self.image_sub = self.create_subscription(
            Image, '/carla/ego_vehicle/camera/rgb/front/image', self.image_callback, 10
        )
        self.control_pub = self.create_publisher(
            CarlaEgoVehicleControl, '/carla/ego_vehicle/vehicle_control_cmd', 10
        )

        # Control parameters
        self.throttle = 0.4  # Constant throttle
        self.max_steering_angle = 1.0  # Maximum steering angle in radians

    def image_callback(self, msg):
        """
        Callback for receiving camera images. Performs lane detection, calculates
        control commands, and sends them to the vehicle.
        """
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Perform lane detection
        trajectory, overlay_image = self.detect_lanes(cv_image)

        # Send control commands based on the trajectory
        self.follow_trajectory(trajectory)

        # Display the lane detection result
        cv2.imshow("Lane Detection", overlay_image)
        cv2.waitKey(1)

    def detect_lanes(self, image):
        """
        Detect lanes in the image and calculate the trajectory.

        Args:
            image (numpy.ndarray): Input image from the camera.

        Returns:
            trajectory (numpy.ndarray): Nx2 array of x, y trajectory points.
            overlay_image (numpy.ndarray): Image with lane detection overlay.
        """
        # Convert the image to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define thresholds for white lane markings
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 25, 255])
        mask = cv2.inRange(hsv, lower_white, upper_white)

        # Detect edges using Canny
        edges = cv2.Canny(mask, 50, 150)

        # Hough Line Transform to detect lane lines
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 50, minLineLength=100, maxLineGap=50)

        overlay_image = image.copy()

        if lines is not None:
            left_lines = []
            right_lines = []

            # Separate lines into left and right based on their slope
            for line in lines:
                x1, y1, x2, y2 = line[0]
                slope = (y2 - y1) / (x2 - x1 + 1e-6)  # Avoid division by zero
                if slope < -0.5:  # Left lane
                    left_lines.append(line[0])
                elif slope > 0.5:  # Right lane
                    right_lines.append(line[0])

            # Draw lines on the overlay image
            for x1, y1, x2, y2 in left_lines + right_lines:
                color = (255, 0, 0) if (x1, y1, x2, y2) in left_lines else (0, 0, 255)
                cv2.line(overlay_image, (x1, y1), (x2, y2), color, 3)

            # Calculate the trajectory as the midpoint of left and right lanes
            trajectory = self.calculate_trajectory(left_lines, right_lines, image.shape)
        else:
            trajectory = np.array([])

        return trajectory, overlay_image

    def calculate_trajectory(self, left_lines, right_lines, img_shape):
        """
        Calculate the trajectory based on detected lane lines.

        Args:
            left_lines (list): List of left lane lines.
            right_lines (list): List of right lane lines.
            img_shape (tuple): Shape of the input image.

        Returns:
            numpy.ndarray: Nx2 array of trajectory points.
        """
        h, w = img_shape[:2]
        x = np.linspace(0, h, num=20)
        if left_lines and right_lines:
            left_poly = np.polyfit([y1 for _, y1, _, _ in left_lines], [x1 for x1, _, _, _ in left_lines], 1)
            right_poly = np.polyfit([y1 for _, y1, _, _ in right_lines], [x1 for x1, _, _, _ in right_lines], 1)

            left_x = np.polyval(left_poly, x)
            right_x = np.polyval(right_poly, x)

            # Midpoint trajectory
            y = x
            mid_x = (left_x + right_x) / 2
            trajectory = np.vstack((mid_x, y)).T
        else:
            trajectory = np.zeros((20, 2))  # Fallback if no lines detected

        return trajectory

    def follow_trajectory(self, trajectory):
        """
        Follow the computed trajectory by sending control commands.

        Args:
            trajectory (numpy.ndarray): Nx2 array of x, y trajectory points.
        """
        if trajectory.size == 0:
            self.get_logger().warning("No trajectory found!")
            return

        # Use the first trajectory point to calculate steering angle
        target_x, target_y = trajectory[0]
        angle_to_target = np.arctan2(target_y, target_x)

        # Generate and publish control commands
        control_msg = CarlaEgoVehicleControl()
        control_msg.throttle = self.throttle
        control_msg.steer = np.clip(angle_to_target, -self.max_steering_angle, self.max_steering_angle)
        control_msg.brake = 0.0

        self.control_pub.publish(control_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LaneFollowingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
