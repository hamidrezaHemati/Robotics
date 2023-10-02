#!/usr/bin/python3

from rand_Stu.msg import Student
import rospy

def callback(data):
    # rospy.loginfo("%s %s is %d in %s" % (data.name, data.last_name, data.age
    #               , data.departement))
    if data.departement == 'Hardware':
        # rospy.loginfo("{}\t{} in Hardware".format(data.name, data.last_name))
        pub_hard.publish(data)
    else:
        # rospy.loginfo("{}\t{} in Software".format(data.name, data.last_name))
        pub_soft.publish(data)


if __name__ == '__main__':
    rospy.init_node('splitter', anonymous=True)
    pub_hard = rospy.Publisher('hardware', Student, queue_size=10)
    pub_soft = rospy.Publisher('software', Student, queue_size=10)
    rospy.Subscriber("std_request", Student, callback)
    rospy.spin()