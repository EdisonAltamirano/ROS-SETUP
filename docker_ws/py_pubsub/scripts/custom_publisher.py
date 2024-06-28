#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16
from geometry_msgs.msg import Pose
from py_pubsub.msg import Obstacle, DistanceFruit
class CustomPublisher:

    def __init__(self):
        rospy.init_node('custom_publisher', anonymous=True)
        self.publisher_ = rospy.Publisher('obstacle', Obstacle, queue_size=10)
        self.subscription = rospy.Subscriber('distance_fruit', DistanceFruit, self.listener_callback)
        self.rate = rospy.Rate(2)  # 2 Hz
        self.i = 0
        self.test_obstacles =[{"type":"Obstacle", "pose":[1,2]},{"type":"Fruit", "pose":[4,6]},{"type":"Obstacle", "pose":[1,9]},{"type":"Fruit", "pose":[2,5]}]

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


    def listener_callback(self, msg):
        if msg.depth.data <= 6.0 :
            rospy.logwarn('I saw a "%s" at : "%f" meters' ,msg.clase,msg.depth.data)
        
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