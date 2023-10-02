#!/usr/bin/python3
from hw3.msg import Nad

import rospy
from nav_msgs.msg import Odometry
import tf


class Obstacle_detector():
    def __init__(self):
        rospy.init_node("obstacle_detector", anonymous=False)
        rospy.on_shutdown(self.on_shutdown)

        self.closest_obstacle = rospy.Publisher("/closest_obstacle", Nad, queue_size=10)  ## NAD --> Name And Distace
        self.odom = rospy.Subscriber("/odom", Odometry, self.update_pose, queue_size=10)


        self.position_of_obstacles = {
            "bookshelf" : (2.64, -1.55),
            "dumpster" : (1.23, -4.57),
            "barrel" : (-2.51, -3.08),
            "postbox"	: (-4.47, -0.57),
            "brick_box" : (-3.44, 2.75),
            "cabinet"	: (-0.45, 4.05),
            "cafe_table" : (1.91, 3.37),
            "fountain" : (4.08, 1.14)
            }

        self.x = 0
        self.y = 0

        self.dt = 1
        self.rate = rospy.Rate(1/self.dt)

    def update_pose(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y


    def distance_from_obstacle(self, obstacle_x, obstacle_y):
        dif_x = (self.x - obstacle_x) ** 2
        dif_y = (self.y - obstacle_y) ** 2
        return (dif_x + dif_y) ** 0.5

    
    def detect_closest_obstacle(self):
        rospy.sleep(2)
        temp = {}
        while not rospy.is_shutdown():
            for key, value in self.position_of_obstacles.items():
                temp[key] = self.distance_from_obstacle(value[0], value[1])
                # print(key, " dist: ", self.distance_from_obstacle(value[0], value[1]))

            nad = Nad()
            nad.obstacle_name = min(temp, key=temp.get)
            nad.distance = min(temp.values())
            print(type(nad))
            print(nad)
            self.closest_obstacle.publish(nad)

            print(" ----------------------------- ")

            self.rate.sleep()

    def on_shutdown(self):
        rospy.sleep(1)


if __name__ == "__main__":
    obstacle_detector = Obstacle_detector()
    obstacle_detector.detect_closest_obstacle()
