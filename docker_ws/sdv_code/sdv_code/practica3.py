#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker

class Practica3Publisher(Node):
    def __init__(self):
        super().__init__('practica3')
        # Publicador de mensajes tipo Marker en el tópico 'visualization_marker'
        self.publisher_ = self.create_publisher(Marker, 'visualization_marker', 10)
        # Timer para publicar cada segundo
        self.timer = self.create_timer(1.0, self.publish_markers)
        
        # Diccionario de marcadores:
        # clave: id del objeto
        # valor: tupla con (r, g, b, a) para el color del marcador
        # Aqui tu codigo para la practica3

    def publish_markers(self):
        # Aqui tu codigo para la practica3
        self.get_logger().info(f'Publicado ')

def main(args=None):
    rclpy.init(args=args)
    node = Practica3Publisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
