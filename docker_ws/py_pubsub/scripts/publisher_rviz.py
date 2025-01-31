#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16
from geometry_msgs.msg import Pose
from py_pubsub.msg import Obstacle, DistanceFruit
from visualization_msgs.msg import Marker
class CustomPublisher:

    def __init__(self):
        rospy.init_node('custom_publisher', anonymous=True)
        self.publisher_ = rospy.Publisher('obstacle', Obstacle, queue_size=10)
        self.marker_publisher_ = rospy.Publisher('marker_fruit', Marker, queue_size=10)
        self.subscription = rospy.Subscriber('distance_fruit', DistanceFruit, self.listener_callback)
        self.rate = rospy.Rate(2)  # 2 Hz
        self.i = 0
        self.test_obstacles =[{"type":"Obstacle", "pose":[1,2]},{"type":"Fruit", "pose":[4,6]},{"type":"Obstacle", "pose":[1,9]},{"type":"Fruit", "pose":[2,5]}]
        self.colorsFruits = {    
            "Apple": [1.0, 0.0, 0.0, 1.0],       # Red
            "Orange": [1.0, 0.65, 0.0, 1.0],     # Orange
            "Pineapple": [1.0, 1.0, 0.0, 1.0],   # Yellow
            "Blueberry": [0.0, 0.0, 1.0, 1.0]    # Blue
            }
        self.marker = Marker()
        self.marker.header.frame_id = "map" #base_link
        self.marker.header.stamp = rospy.Time.now()
        
        self.marker.ns = "fruit"
        self.marker.id = 0
        
        self.marker.type = Marker.SPHERE
        
        self.marker.action = Marker.ADD
        
        self.marker.pose.position.y = 0
        self.marker.pose.position.z = 0
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0
        
        self.marker.scale.x = 1.0
        self.marker.scale.y = 1.0
        self.marker.scale.z = 1.0
        

    def timer_callback(self):

        self.i += 1
        if self.i == len(self.test_obstacles):
            self.i = 0
        msg = Obstacle()
        msg.type = self.test_obstacles[self.i]["type"]
        msg.pose = Pose()
        msg.pose.position.x = self.test_obstacles[self.i]["pose"][0]
        msg.pose.position.y = self.test_obstacles[self.i]["pose"][1]
        self.publisher_.publish(msg)
        rospy.loginfo('Publishing: "%s"' % msg)

    def fruit_marker(self,msg):
        self.marker.pose.position.x = msg.depth.data
        self.marker.color.r = self.colorsFruits[msg.clase][0]
        self.marker.color.g = self.colorsFruits[msg.clase][1]
        self.marker.color.b = self.colorsFruits[msg.clase][2]
        self.marker.color.a = self.colorsFruits[msg.clase][3]
        self.marker_publisher_.publish(self.marker)

    def listener_callback(self, msg):
        if msg.depth.data <= 6.0 :
            rospy.logwarn('I saw a "%s" at : "%f" meters' ,msg.clase,msg.depth.data)
            self.fruit_marker(msg)
        
    def spin(self):
        while not rospy.is_shutdown():
            self.timer_callback()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        custom_publisher = CustomPublisher()
        custom_publisher.spin()
    except rospy.ROSInterruptException:
        pass