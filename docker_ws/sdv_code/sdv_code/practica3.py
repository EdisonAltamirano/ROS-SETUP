
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
        self.markers_dict = {
            1: (1.0, 0.0, 0.0, 1.0),  # Rojo
            2: (0.0, 1.0, 0.0, 1.0),  # Verde
            3: (0.0, 0.0, 1.0, 1.0)   # Azul
        }

    def publish_markers(self):
        for marker_id, color in self.markers_dict.items():
            marker = Marker()
            # Configuración del header
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            # Namespace y ID para diferenciar cada marcador
            marker.ns = "practica3"
            marker.id = marker_id
            # Tipo de marcador: CUBE (cubo)
            marker.type = Marker.CUBE
            # Acción: agregar o modificar el marcador
            marker.action = Marker.ADD
            
            # Ubicación: se separa en el eje X según el id (solo para ejemplo)
            marker.pose.position.x = float(marker_id) * 1.5
            marker.pose.position.y = 0.0
            marker.pose.position.z = 0.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            
            # Escala del marcador (tamaño del cubo)
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0
            
            # Asigna el color extraído del diccionario
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = color[3]
            
            # Lifetime: 0 indica que el marcador se mantiene indefinidamente
            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 0

            # Publica el marcador
            self.publisher_.publish(marker)
            self.get_logger().info(f'Publicado marker id: {marker_id} con color: {color}')

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
