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
        #Aqui tu codigo de la Practica 2
        self.get_logger().info(f'Publicado:')

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
