#!/usr/bin/env python3

import rospy
from py_pubsub.msg import Obstacle, DistanceFruit
import random
class CustomSubscriber:

    def __init__(self):
        rospy.init_node('custom_subscriber', anonymous=True)
        self.subscription = rospy.Subscriber('obstacle', Obstacle, self.listener_callback)
        self.publisher_ = rospy.Publisher('distance_fruit', DistanceFruit, queue_size=10)
        self.rate = rospy.Rate(2)  # 2 Hz
        self.object = Obstacle()
        self.myposition = [0,0]
        self.listFruits = ["Apple", "Orange", "Pineapple", "Blueberry"]
    def listener_callback(self, msg):
        self.object = msg
    def euclidean_distance(self, point1, point2):
        """
        Calculates the Euclidean distance between two points.

        Args:
            point1: A list or tuple representing the first point's coordinates.
            point2: A list or tuple representing the second point's coordinates.

        Returns:
            The Euclidean distance between the two points.
        """

        if len(point1) != len(point2):
            raise ValueError("Points must have the same number of dimensions")

        # Calculate the squared difference for each dimension
        squared_differences = [(p1 - p2) ** 2 for p1, p2 in zip(point1, point2)]

        # Sum the squared differences
        sum_of_squares = sum(squared_differences)

        # Return the square root of the sum (Euclidean distance)
        return sum_of_squares ** 0.5
    def timer_callback(self):
        if self.object.pose.position.x is not None  and self.object.type == 'Fruit':
            random_fruit = random.choice(self.listFruits)
            msg = DistanceFruit()
            msg.clase = random_fruit
            fruit_obstacle = [self.object.pose.position.x,self.object.pose.position.y]
            msg.depth.data = self.euclidean_distance(self.myposition,fruit_obstacle)
            self.publisher_.publish(msg)
            rospy.loginfo('Publishing: "%s"' % msg)

    def spin(self):
        while not rospy.is_shutdown():
            self.timer_callback()
            self.rate.sleep()
def main():
    try:
        custom_subscriber = CustomSubscriber()
        custom_subscriber.spin()    
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")

if __name__ == '__main__':
    main()