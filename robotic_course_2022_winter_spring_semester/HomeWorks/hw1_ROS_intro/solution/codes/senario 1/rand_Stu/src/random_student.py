#!/usr/bin/python3

from copy import deepcopy
from rand_Stu.msg import Student
from random import randint, seed, choice
from time import time
import rospy

names = [
    'Ali',
    "Mohammad",
    "Fatemeh",
    "Amir",
    "Reza",
    "Sahar",
    "Aref",
    "Aria",
    "Ahmad",
    "Akbar",
    "Mohammad Reza",
    "Amir Hosein",
    "Saman",
    "Mohsen",
    "Radin",
    "Maryam",
    "Javad",
    "Ramin",
    "Soroush",
    "Farhad",
    "Siamak",
    "Mehran",
    "Karim",
]

lNames = [
    'Akbari',
    'Hashemi',
    "Ghasemi",
    "Hoseini",
    "Eslami",
    "Kazemi",
    "Kashfi",
    "Shahi",
    "Sheikhi",
    "Kabiri",
    "Majidi",
    "Karimi",
    "Ghafori",
    "Pormokhber",
    "Ansari",
    "Modiri",
    "Fallah",
    "Ansarifard",
]


departements= [
    "Software",
    "Hardware",
]


def randName():
    # global names
    return choice(names)

def randLName():
    # global lName
    return choice(lNames)


def randAge():
    return randint(18,30)


def randDepartement():
    # global departements
    return choice(departements)


def randStudent():
    seed(time())
    student = Student()
    student.name = randName()
    student.age = randAge()

    student.last_name = randLName()
    student.departement = randDepartement()

    return student


def std_request():
    student = Student()
    pub = rospy.Publisher('std_request', Student, queue_size=10)
    rospy.init_node('student_request', anonymous=True)
    rate = rospy.Rate(0.5)

    while not rospy.is_shutdown():
        student = randStudent()
        rospy.loginfo('{} {}\t\t age:{}\t{}'
        .format(student.name, student.last_name, student.age, student.departement))
        
        pub.publish(student)
        rate.sleep()

if __name__ == '__main__':
    try:
        std_request()
    except rospy.ROSInterruptException:
        pass



