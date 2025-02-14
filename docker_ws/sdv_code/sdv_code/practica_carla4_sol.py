#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from builtin_interfaces.msg import Time
# Se asume que se cuenta con estas dependencias:
import carla
from agents.navigation.global_route_planner import GlobalRoutePlanner
import carla_common.transforms as trans
class CarlaToRosWaypointConverter(Node):
    def __init__(self):
        super().__init__('carla_waypoint_publisher')
        # Declaración de parámetros: host, port, timeout y role_name
        self.declare_parameter('host', '172.19.0.2')
        self.declare_parameter('port', 2000)
        self.declare_parameter('timeout', 10)
        self.declare_parameter('role_name', 'ego_vehicle')
        self.role_name = self.get_parameter('role_name').value
        # Publicador de la ruta (tipo nav_msgs/Path) en el tópico /carla/<role_name>/waypoints
        self.waypoint_publisher = self.create_publisher(
            Path, f'/carla/{self.role_name}/waypoints', 10
        )
        # Variables internas
        self.ego_vehicle = None
        self.ego_vehicle_location = None
        self.goal = None
        self.current_route = None
        self.WAYPOINT_DISTANCE = 2.0
        # Conexión a CARLA y obtención del mundo y mapa
        self.connect_to_carla()
        self.map = self.world.get_map()
        # En lugar de suscribirnos a un tópico para obtener el goal, definimos por defecto el siguiente:
        # header:
        #   stamp:
        #     sec: 1738961382
        #     nanosec: 459879610
        #   frame_id: map
        # point:
        #   x: 334.6995544433594
        #   y: -270.9805603027344
        #   z: -0.00800323486328125
        default_goal = PoseStamped()
        default_goal.header.frame_id = "map"
        default_goal.header.stamp = Time(sec=1738961382, nanosec=459879610)
        default_goal.pose.position.x = 334.6995544433594
        default_goal.pose.position.y = -270.9805603027344
        default_goal.pose.position.z = -0.00800323486328125
        # Se asigna una orientación por defecto (sin rotación)
        default_goal.pose.orientation.w = 1.0
        # Llamamos al callback on_goal con el objetivo por defecto.
        self.on_goal(default_goal)
        # Se crea un timer para buscar el vehículo “ego” cada 1 segundo.
        self.create_timer(1.0, self.find_ego_vehicle_actor)
        self.get_logger().info("Nodo CarlaToRosWaypointConverter iniciado.")
    def connect_to_carla(self):
        host = self.get_parameter('host').value
        port = self.get_parameter('port').value
        timeout = self.get_parameter('timeout').value
        self.get_logger().info(f"Conectando a CARLA en {host}:{port}...")
        try:
            client = carla.Client(host, port)
            client.set_timeout(timeout)
            self.world = client.get_world()
            self.get_logger().info("Conectado a CARLA.")
        except Exception as e:
            self.get_logger().error(f"Error al conectar a CARLA: {e}")
            raise e
    def on_goal(self, goal_msg: PoseStamped):
        """
        Función callback que se invoca al establecer un goal.
        Convierte la pose recibida a una transformación de CARLA, asigna el nuevo objetivo y
        dispara el replanificado (reroute).
        """
        self.get_logger().info("Se ha establecido un goal, recalculando ruta...")
        # Conversión de la pose de ROS a la transformación propia de CARLA
        carla_goal = trans.ros_pose_to_carla_transform(goal_msg.pose)
        self.goal = carla_goal
        self.reroute()
    def reroute(self):
        """
        Recalcula la ruta desde la posición actual del vehículo “ego” hasta el objetivo.
        Si no se dispone de ego_vehicle o goal, se limpia la ruta.
        Luego se publica la ruta (waypoints).
        """
        if self.ego_vehicle is None or self.goal is None:
            self.current_route = None
            self.publish_waypoints()
        else:
            self.current_route = self.calculate_route(self.goal)
            self.publish_waypoints()
    def calculate_route(self, goal):
        """
        Calcula la ruta utilizando el GlobalRoutePlanner desde la ubicación actual del vehículo
        hasta la ubicación del goal.
        """
        self.get_logger().info(
            f"Calculando ruta hacia x={goal.location.x}, y={goal.location.y}, z={goal.location.z}"
        )
        grp = GlobalRoutePlanner(self.map, sampling_resolution=1)
        start_location = self.ego_vehicle.get_location()
        # Trazado de la ruta entre la posición actual y la posición objetivo.
        route = grp.trace_route(
            start_location,
            carla.Location(goal.location.x, goal.location.y, goal.location.z)
        )
        return route
    def publish_waypoints(self):
        """
        Publica la ruta (mensaje de tipo nav_msgs/Path) con los waypoints calculados.
        Cada waypoint se convierte a un PoseStamped (utilizando la función de transformación).
        """
        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        if self.current_route is not None:
            for wp, _ in self.current_route:
                ps = PoseStamped()
                ps.header.frame_id = "map"
                ps.header.stamp = self.get_clock().now().to_msg()
                ps.pose = trans.carla_transform_to_ros_pose(wp.transform)
                msg.poses.append(ps)
        self.waypoint_publisher.publish(msg)
        self.get_logger().info(f"Ruta publicada con {len(msg.poses)} waypoints.")
    def find_ego_vehicle_actor(self):
        """
        Busca entre los actores de CARLA aquel cuyo atributo 'role_name' coincida con el parámetro.
        Si se detecta un cambio (nuevo actor o reubicación del vehículo), se dispara el replanificado.
        """
        hero = None
        for actor in self.world.get_actors():
            if actor.attributes.get('role_name') == self.role_name:
                hero = actor
                break
        ego_vehicle_changed = False
        if hero is None and self.ego_vehicle is not None:
            ego_vehicle_changed = True
        if hero is not None and self.ego_vehicle is None:
            ego_vehicle_changed = True
        if hero is not None and self.ego_vehicle is not None and hero.id != self.ego_vehicle.id:
            ego_vehicle_changed = True
        if ego_vehicle_changed:
            self.get_logger().info("El ego vehicle ha cambiado.")
            self.ego_vehicle = hero
            self.reroute()
        elif self.ego_vehicle:
            current_location = self.ego_vehicle.get_location()
            if self.ego_vehicle_location is not None:
                dx = self.ego_vehicle_location.x - current_location.x
                dy = self.ego_vehicle_location.y - current_location.y
                distance = math.sqrt(dx * dx + dy * dy)
                if distance > self.WAYPOINT_DISTANCE:
                    self.get_logger().info("El ego vehicle se ha reubicado, recalculando ruta.")
                    self.reroute()
            self.ego_vehicle_location = current_location
def main(args=None):
    rclpy.init(args=args)
    node = CarlaToRosWaypointConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupción por teclado.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()
