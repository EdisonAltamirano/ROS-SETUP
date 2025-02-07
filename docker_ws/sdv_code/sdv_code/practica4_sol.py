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
        self.robot_x = 4.0
        self.robot_y = 4.0
        self.robot_yaw_deg = 0.0  # en grados
        self.robot_yaw = math.radians(self.robot_yaw_deg)

    def broadcast_transform(self):
        """
        Publica la transformación (TF) del robot: de "map" a "base_link".
        """
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.robot_x
        t.transform.translation.y = self.robot_y
        t.transform.translation.z = 0.0
        t.transform.rotation = quaternion_from_euler(0.0, 0.0, self.robot_yaw)
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().debug("Broadcasting transform: map -> base_link")

    def timer_callback(self):
        current_time = self.get_clock().now().to_msg()

        # --- Publicar Marker del robot (posición y orientación) en "map" ---
        robot_marker = Marker()
        robot_marker.header.frame_id = "map"
        robot_marker.header.stamp = current_time
        robot_marker.ns = "robot"
        robot_marker.id = 0
        robot_marker.type = Marker.ARROW  # Representa la orientación
        robot_marker.action = Marker.ADD
        robot_marker.pose.position.x = self.robot_x
        robot_marker.pose.position.y = self.robot_y
        robot_marker.pose.position.z = 0.0
        robot_marker.pose.orientation = quaternion_from_euler(0.0, 0.0, self.robot_yaw)
        robot_marker.scale.x = 1.0   # Longitud de la flecha
        robot_marker.scale.y = 0.2   # Ancho
        robot_marker.scale.z = 0.2   # Alto
        robot_marker.color.r = 0.0
        robot_marker.color.g = 1.0
        robot_marker.color.b = 0.0
        robot_marker.color.a = 1.0
        self.pub_robot_marker.publish(robot_marker)

        # --- Para cada obstáculo, publicar:
        # 1. Un marker en "base_link" (tal como se detectó)
        # 2. Un marker transformado a "map" usando tf2 (con listener) ---
        for obs in self.obstacles:
            obs_id = obs["id"]

            # Marker en "base_link" (sin transformación)
            marker_base = Marker()
            marker_base.header.frame_id = "base_link"
            marker_base.header.stamp = current_time
            marker_base.ns = "obstacles"
            marker_base.id = obs_id
            marker_base.type = Marker.SPHERE
            marker_base.action = Marker.ADD
            marker_base.pose.position.x = obs["x"]
            marker_base.pose.position.y = obs["y"]
            marker_base.pose.position.z = 0.0
            marker_base.pose.orientation.w = 1.0
            marker_base.scale.x = 0.5
            marker_base.scale.y = 0.5
            marker_base.scale.z = 0.5
            marker_base.color.r = 1.0
            marker_base.color.g = 0.0
            marker_base.color.b = 0.0
            marker_base.color.a = 1.0
            self.pub_obs_base.publish(marker_base)

            # Crear un PointStamped para el obstáculo en "base_link"
            point_in_base = PointStamped()
            point_in_base.header.frame_id = "base_link"
            point_in_base.header.stamp = current_time
            point_in_base.point.x = obs["x"]
            point_in_base.point.y = obs["y"]
            point_in_base.point.z = 0.0

            try:
                # Espera y obtiene la transformación de "base_link" a "map"
                transform = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
                # Transforma el punto del obstáculo a "map"
                point_in_map = tf2_geometry_msgs.do_transform_point(point_in_base, transform)

                # Marker para el obstáculo en "map"
                marker_map = Marker()
                marker_map.header.frame_id = "map"
                marker_map.header.stamp = current_time
                marker_map.ns = "obstacles"
                marker_map.id = obs_id + 100  # ID diferente para distinguir
                marker_map.type = Marker.SPHERE
                marker_map.action = Marker.ADD
                marker_map.pose.position.x = point_in_map.point.x
                marker_map.pose.position.y = point_in_map.point.y
                marker_map.pose.position.z = point_in_map.point.z
                marker_map.pose.orientation.w = 1.0
                marker_map.scale.x = 0.5
                marker_map.scale.y = 0.5
                marker_map.scale.z = 0.5
                marker_map.color.r = 0.0
                marker_map.color.g = 0.0
                marker_map.color.b = 1.0
                marker_map.color.a = 1.0
                self.pub_obs_map.publish(marker_map)

            except Exception as e:
                self.get_logger().warn(f'No se pudo transformar el obstáculo {obs_id}: {e}')

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