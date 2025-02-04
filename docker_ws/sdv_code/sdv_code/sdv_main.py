import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from carla_msgs.msg import CarlaEgoVehicleControl
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class VehicleControlNode(Node):
    def __init__(self):
        super().__init__('vehicle_control_node')
        self.subscription = self.create_subscription(
            Image,
            '/carla/ego_vehicle/rgb_front/image',
            self.listener_callback,
            100)
        self.bridge = CvBridge()
        self.control_pub = self.create_publisher(CarlaEgoVehicleControl, '/carla/ego_vehicle/vehicle_control_cmd_manual', 10)
        
        # Vehicle parameters
        self.wheelbase = 2.5  # meters
        self.max_steering_angle = np.radians(30)  # in radians
    def compute_ackermann_steering(self, speed, radius):
        """
        Compute the steering angle using Ackermann steering geometry.
        """
        if radius == 0:
            return 0.0
        return np.arctan(self.wheelbase / radius)

    def compute_icr(self, velocity, steering_angle):
        """
        Compute the Instantaneous Center of Rotation (ICR) given a velocity and steering angle.
        """
        if steering_angle == 0:
            return float('inf')  # Infinite radius means straight motion
        return self.wheelbase / np.tan(steering_angle)

    def pure_pursuit_control(self, target_point, lookahead_distance):
        """
        Pure Pursuit Algorithm for path tracking.
        """
        dx = target_point[0]
        dy = target_point[1]
        L = np.sqrt(dx**2 + dy**2)
        steering_angle = np.arctan2(2 * self.wheelbase * dy, L**2)
        return np.clip(steering_angle, -self.max_steering_angle, self.max_steering_angle)

    def calculate_optimal_steering_angle(self, trajectory):
        """
        Compute the optimal steering angle given a trajectory.
        """
        if trajectory.size == 0:
            return 0.0
        return self.pure_pursuit_control(trajectory[0], lookahead_distance=5.0)
    def listener_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV format
            self.get_logger().info(f"Image encoding: {msg.encoding}")

            if msg.encoding == '8UC3':
                cv_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            else:
                cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            # Perform lane detection
            # trajectory, overlay_image = self.detect_lanes(cv_image)
            # Placeholder for processing trajectory
            trajectory = np.array([[5.0, 2.0], [10.0, 3.0]])
            optimal_steering = self.calculate_optimal_steering_angle(trajectory)
            
            # Generate and publish control commands
            control_msg = CarlaEgoVehicleControl()
            control_msg.throttle = 0.3  # Example value
            control_msg.steer = optimal_steering
            control_msg.brake = 0.0

            self.control_pub.publish(control_msg)

        except CvBridgeError as e:
            self.get_logger().error(f'Error converting image: {e}')
            return

       

def main(args=None):
    rclpy.init(args=args)
    node = VehicleControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
