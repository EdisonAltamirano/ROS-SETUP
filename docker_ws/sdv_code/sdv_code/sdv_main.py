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
        #Aqui el codigo de la practica6
        return 0

    def compute_icr(self, velocity, steering_angle):
        """
        Compute the Instantaneous Center of Rotation (ICR) given a velocity and steering angle.
        """
        #Aqui el codigo de la practica6
        return 0

    def pure_pursuit_control(self, target_point):
        """
        Pure Pursuit Algorithm for path tracking.
        """
        #Aqui el codigo de la practica6
        return 0

    def calculate_optimal_steering_angle(self, trajectory):
        """
        Compute the optimal steering angle given a trajectory.
        """
        if trajectory.size == 0:
            return 0.0
        return self.pure_pursuit_control(trajectory[0])
    def listener_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV format
            self.get_logger().info(f"Nodo sdv")
            # Placeholder for processing trajectory
            trajectory = np.array([[5.0, 2.0], [10.0, 3.0]])
            optimal_steering = self.calculate_optimal_steering_angle(trajectory)
            
            # Generate and publish control commands
            #Aqui el codigo de la practica6

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
