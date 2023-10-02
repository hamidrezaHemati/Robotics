#!/usr/bin/python3

import rospy
from rand_Stu.msg import Student

def display(data):
    rospy.loginfo('{} {} age:{}'.format(data.name, data.last_name, data.age))


def software_departement():
    rospy.init_node('hardware', anonymous=True)
    rospy.Subscriber('hardware', Student, callback=display)
    rospy.spin()


if __name__ == '__main__':
    software_departement()