#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HelloWorldPublisher(Node):
    def __init__(self):
        super().__init__('hello_world_publisher')
        # Crea un publicador para el mensaje String en el tópico '/hello_topic'
        self.publisher_ = self.create_publisher(String, 'hello_topic', 10)
        # Publicar cada 1 segundo
        timer_period = 1.0  
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
    def timer_callback(self):
        msg = String()
        msg.data = 'Hello world'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publicando: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = HelloWorldPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
