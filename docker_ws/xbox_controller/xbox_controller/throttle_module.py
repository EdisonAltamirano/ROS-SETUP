#!/usr/bin/env python
import can
import time
import rclpy
from rclpy.node import Node
from sdv_msg.msg import ThrottleMsg
from std_msgs.msg import String
from numpy import interp

class ThrottleModule(Node):
    def __init__(self):
        super().__init__('throttle_module')
        self.safe_velocity=90 # % of safe_pot  
        self.safe_pot = 140
        self.throttle_module_id = 1030 #hex 406
        #Send WiperPot position
        self.pot_id = 0x5 #hex.5
        #Send Max velocity
        self.max_id = 0X8 #hex.8
        self.old_maxvel = self.safe_velocity
        self.temp_pot= 1
        self.new_maxvel = self.safe_velocity
        self.bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=125000)
        self.pub_throttle_status = self.create_publisher(ThrottleMsg, 'throttle/status', 10)
        timer_period = 0.1 #1 second
        self.timer=self.create_timer(timer_period,self.timer_callback)
        #Sub to change max velocity and position (Auto Mode)
        self.sub_throttle = self.create_subscription(
            ThrottleMsg,
            '/sdv/xbox_controller/xbox_throttle',
            self.throttle_callback,
            10
        )
        self.old_increase_maxvel = 0 
        self.increase_maxvel = 0 
        self.old_decrease_maxvel = 0 
        self.decrease_maxvel = 0
        self.new_pot = 0
        self.old_pot = 0 
        self.limit_pot = self.safe_velocity

    def throttle_callback(self,msg):
        #Detect down bottom to decrease velocity
        #Change max velocity
        if bool(msg.increase_maxvel.data):
            if self.new_maxvel<=self.safe_velocity-5:
                self.new_maxvel+=5
        if bool(msg.decrease_maxvel.data):
            if self.new_maxvel>=5:
                self.new_maxvel-=5
        if self.old_maxvel != self.new_maxvel:
            self.limit_pot = interp(self.new_maxvel, [0,100], [0,self.safe_pot])
            self.old_maxvel=self.new_maxvel
            self.get_logger().info('New max velocity: '+ str(self.new_maxvel)+" %")
            self.get_logger().info('Pot Position: '+ str(self.limit_pot))
            self.bus.send(can.Message(arbitration_id=self.throttle_module_id,is_extended_id=False, data=[self.max_id,int(self.limit_pot)]), timeout=1)
        #Change pot position
        self.new_pot = msg.pot.data
    def timer_callback(self):
        #Modo 2 (0-100%) en 100 segundos
        if int(self.new_pot)>0:
            self.temp_pot=100 if self.temp_pot>=100 else self.temp_pot+1
            temp_pos = interp(self.temp_pot, [0,100], [1,self.limit_pot]) 
            self.get_logger().info('Vel position: '+ str(self.temp_pot))
            self.get_logger().info('Pot position: '+ str(temp_pos))
            self.bus.send(can.Message(arbitration_id=self.throttle_module_id,is_extended_id=False,  data=[self.pot_id,int(temp_pos)]), timeout=1)
        else:
            self.temp_pot=0 if self.temp_pot<=0 else self.temp_pot-5
            temp_pos = interp(self.temp_pot, [0,100], [1,self.limit_pot]) 
            self.get_logger().info('Vel position: '+ str(self.temp_pot))
            self.get_logger().info('Pot position: '+ str(temp_pos))
            self.bus.send(can.Message(arbitration_id=self.throttle_module_id,is_extended_id=False,  data=[self.pot_id,int(temp_pos)]), timeout=1)

def main(args=None):
    rclpy.init(args=args)
    throttle = ThrottleModule()
    throttle.get_logger().info('Throttle node started')
    rclpy.spin(throttle)
    throttle.destroy_node()
    rclpy.shutdown()
            
if __name__ == "__main__":
    main()
