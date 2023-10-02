#!/usr/bin/python3

from hw3.msg import Nad
import rospy


def display(data):
    rospy.loginfo('{} {}'.format(data.obstacle_name, data.distance))


def hardware_departement():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('closest_obstacle', Nad, callback=display)
    rospy.spin()

if __name__ == '__main__':
    hardware_departement()