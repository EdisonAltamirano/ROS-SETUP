#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node

# Mensajes para los markers y la transformación
from visualization_msgs.msg import Marker
from geometry_msgs.msg import TransformStamped, Quaternion, PointStamped

# tf2: para broadcaster y listener
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
import tf2_geometry_msgs  # para do_transform_point

def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> Quaternion:
    """
    Convierte ángulos Euler (roll, pitch, yaw) a un quaternion.
    """
    qx = math.sin(roll/2.0) * math.cos(pitch/2.0) * math.cos(yaw/2.0) - math.cos(roll/2.0) * math.sin(pitch/2.0) * math.sin(yaw/2.0)
    qy = math.cos(roll/2.0) * math.sin(pitch/2.0) * math.cos(yaw/2.0) + math.sin(roll/2.0) * math.cos(pitch/2.0) * math.sin(yaw/2.0)
    qz = math.cos(roll/2.0) * math.cos(pitch/2.0) * math.sin(yaw/2.0) - math.sin(roll/2.0) * math.sin(pitch/2.0) * math.cos(yaw/2.0)
    qw = math.cos(roll/2.0) * math.cos(pitch/2.0) * math.cos(yaw/2.0) + math.sin(roll/2.0) * math.sin(pitch/2.0) * math.sin(yaw/2.0)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

class Practica4TfMarkers(Node):
    def __init__(self):
        super().__init__('practica4_tf_markers')

        # --- Publicadores de markers ---
        # Marker para la posición del robot (en "map")
        self.pub_robot_marker = self.create_publisher(Marker, 'robot_marker', 10)
        # Marker para los obstáculos en el marco "base_link"
        self.pub_obs_base = self.create_publisher(Marker, 'obstacles_marker_base', 10)
        # Marker para los obstáculos transformados al marco "map"
        self.pub_obs_map = self.create_publisher(Marker, 'obstacles_marker_map', 10)

        # --- Transform Broadcaster (para publicar el TF del robot) ---
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- Transform Listener ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Timers ---
        # Publica los markers cada 1 segundo
        self.timer = self.create_timer(1.0, self.timer_callback)
        # Publica la transformación a 10 Hz
        self.tf_timer = self.create_timer(0.1, self.broadcast_transform)

        # --- Datos de los obstáculos (definidos en "base_link") ---
        # Cada obstáculo es un diccionario con id y coordenadas (x, y) en base_link
        self.obstacles = [
            {"id": 1, "x": 2.0, "y": 2.0},
            {"id": 2, "x": 4.0, "y": 4.0},
            {"id": 3, "x": 7.0, "y": 7.0}
        ]

        # --- Datos del robot ---
        # Posición y orientación del robot en el marco "map"
        self.robot_x = 5.0
        self.robot_y = 6.0
        self.robot_yaw_deg = 40.0  # en grados
        self.robot_yaw = math.radians(self.robot_yaw_deg)

    def broadcast_transform(self):
        """
        Publica la transformación (TF) del robot: de "map" a "base_link".
        """
        #Aqui el codigo de la practica4 
        self.get_logger().debug("Broadcasting transform: map -> base_link")

    def timer_callback(self):
        current_time = self.get_clock().now().to_msg()

        # --- Publicar Marker del robot (posición y orientación) en "map" ---
        #Aqui el codigo de la practica4 

        # --- Para cada obstáculo, publicar:
        # 1. Un marker en "base_link" (tal como se detectó)
        # 2. Un marker transformado a "map" usando tf2 (con listener) ---
        #Aqui el codigo de la practica4 
        
def main(args=None):
    rclpy.init(args=args)
    node = Practica4TfMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
