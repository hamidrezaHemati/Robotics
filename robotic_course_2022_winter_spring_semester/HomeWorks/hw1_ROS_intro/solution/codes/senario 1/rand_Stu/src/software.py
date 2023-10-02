#!/usr/bin/python3

from rand_Stu.msg import Student
import rospy


def display(data):
    rospy.loginfo('{} {} age:{}'.format(data.name, data.last_name, data.age))


def hardware_departement():
    rospy.init_node('software', anonymous=True)
    rospy.Subscriber('software', Student, callback=display)
    rospy.spin()

if __name__ == '__main__':
    hardware_departement()