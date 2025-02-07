#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sdv_msg.msg import MyMessage
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String

class CustomMsgPublisher(Node):
    def __init__(self):
        super().__init__('custom_msg_publisher')
        self.publisher_ = self.create_publisher(MyMessage, 'custom_topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = MyMessage()
        msg.tipo = "Hola, ROS2!"
        # Configurar un PointStamped (ejemplo)
        point = PointStamped()
        point.header.stamp = self.get_clock().now().to_msg()
        point.header.frame_id = "map"
        point.point.x = 1.0
        point.point.y = 2.0
        point.point.z = 0.0
        msg.distance = point

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publicado: {msg.tipo} con point ({point.point.x}, {point.point.y}, {point.point.z})')

def main(args=None):
    rclpy.init(args=args)
    node = CustomMsgPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
