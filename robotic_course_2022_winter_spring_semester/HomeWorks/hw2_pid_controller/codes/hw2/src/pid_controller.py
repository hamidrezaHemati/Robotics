#!/usr/bin/python3

from importlib.resources import path
from locale import normalize
from re import X
from turtle import distance
from unittest import TestSuite
import rospy
import tf
import numpy as np

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from itertools import count, cycle

from math import atan2, radians, degrees, pi, cos, sin, tan, exp


class Controller():
    def __init__(self):
        rospy.init_node("controller", anonymous=False)
        rospy.on_shutdown(self.on_shutdown)

        self.cmd_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.odom = rospy.Subscriber("/odom", Odometry, self.update_pose, queue_size=1)

        self.path_array = self.octagonal()
        self.path = cycle(self.path_array)
        self.goal = next(self.path)

        self.x = 0
        self.y = 0
        self.yaw = 0

        self.kp = 1.5
        self.ki = 0.02
        self.kd = 20
        self.distance_margin = 0.2

        self.kp_theta = 3

        self.dt = 0.05
        self.rate = rospy.Rate(1/self.dt)

        self.errors = []

    
    def rectangle(self):
        rectangle = []
        x1 = np.linspace(3, -3, 15)
        rectangle = [*rectangle, *[[i, 2.0] for i in x1]]

        x2 = np.linspace(2, -2, 10)
        rectangle = [*rectangle, *[[-3.0, i] for i in x2]]

        y1 = np.linspace(-3, 3, 15)
        rectangle = [*rectangle, *[[i, -2.0] for i in y1]]

        y2 = np.linspace(-2, 2, 10)
        rectangle = [*rectangle, *[[3.0, i] for i in y2]]
        
        return rectangle


    def circles(self):
        circles = []
        X1 = np.linspace(-6., -2, 10)
        circles = [*circles, *[[i, 0.0] for i in X1]]

        x_dim, y_dim = 2, 2
        t = np.linspace(np.pi, 0, 15)
        circles = [*circles, *[[x_dim * np.cos(i), y_dim * np.sin(i)] for i in t]]

        X3 = np.linspace(2, 6, 10)
        circles = [*circles, *[[i, 0.0] for i in X3]]

        x_dim, y_dim = 6, 6
        t = np.linspace(np.pi * 2, np.pi, 40)
        circles = [*circles, *[[x_dim * np.cos(i), y_dim * np.sin(i)] for i in t]]
        return circles

    def archimedean_spiral(self):
        archimedean_spiral = []
        growth_factor = 0.1
        for i in range(400):
            t = i / 20 * pi
            dx = (1 + growth_factor * t) * cos(t)
            dy = (1 + growth_factor * t) * sin(t)
            archimedean_spiral.append([dx, dy])
        return archimedean_spiral


    def logaritmic_spiral(self):
        logarithmic_spiral = []
        a = 0.17
        k = tan(a)

        for i in range(150):
            t = i / 20 * pi
            dx = a * exp(k * t) * cos(t)
            dy = a * exp(k * t) * sin(t)
            logarithmic_spiral.append([dx, dy])
        return logarithmic_spiral


    def update_pose(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = self.quaternion_to_euler(msg)


    def quaternion_to_euler(self, msg):
        orientation = msg.pose.pose.orientation
        # convert quaternion to odom
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((
            orientation.x ,orientation.y ,orientation.z ,orientation.w
        )) 
        return yaw

    def octagonal(self):
        octagonal = []
        X1 = np.linspace(-1, 1 , 10)
        octagonal = [*octagonal, *[[i, 3.0] for i in X1]]


        X2 = np.linspace(1, 1 + 2**(1/2) , 10)
        Y2 = - (2**(1/2)) * (X2 - 1) + 3
        octagonal = [*octagonal, *[(i, j) for (i, j) in zip(X2, Y2)]]

        Y3 = np.linspace(1, -1 , 10)
        X3 = np.array([1 + 2**(1/2)]*10)
        octagonal = [*octagonal, *[(i, j) for (i, j) in zip(X3, Y3)]]

        X4 = np.linspace(1 + 2**(1/2), 1, 10)
        Y4 = (2**(1/2)) * (X4 - 1 - 2**(1/2)) -1 
        octagonal = [*octagonal, *[(i, j) for (i, j) in zip(X4, Y4)]]

        X5 = np.linspace(1, -1 , 10)
        octagonal = [*octagonal, *[[i, -3.0] for i in X5]]

        X6 = np.linspace(-1, -1 - 2**(1/2) , 10)
        Y6 = - (2**(1/2)) * (X6 + 1) - 3 
        octagonal = [*octagonal, *[(i, j) for (i, j) in zip(X6, Y6)]]

        Y7 = np.linspace(-1, 1 , 10)
        X7 = np.array([- 1 - 2**(1/2)]*10)
        octagonal = [*octagonal, *[(i, j) for (i, j) in zip(X7, Y7)]]


        X8 = np.linspace(-1 - 2**(1/2), -1, 10)
        Y8 = (2**(1/2)) * (X8 + 1 + 2**(1/2)) + 1
        octagonal = [*octagonal, *[(i, j) for (i, j) in zip(X8, Y8)]]
        return octagonal


    def distance(self, goal,  _x, _y):
        dist = ((goal[0] - _x)**2 + (goal[1] - _y)**2)**0.5
        print("dist: ", dist)
        return dist
    

    def normalize_angle(self ,angle):
        res = angle
        while res > pi:
            res -= 2.0 * pi
        while res < -pi:
            res += 2.0 * pi
        return res
    

    def path_angle(self):
        inc_x = self.goal[0] - self.x
        inc_y = self.goal[1] - self.y
        path_angle = atan2(inc_y, inc_x)
        return path_angle


    def run(self):

        previous_err = self.distance(self.goal, self.x, self.y)
        error_diff = 0
        integral = 0
        path_angle = self.path_angle()
        current_angle = self.yaw
        previous_angle = self.yaw

        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        is_goal_changed = False

        while not rospy.is_shutdown():

            err = self.distance(self.goal, self.x, self.y)

            ## check if robot reach the margin point - if true change goal point to next point on path
            if err < self.distance_margin:
                self.goal = next(self.path)
                is_goal_changed = True
                integral = 0
                continue


            ## control equations
            current_angle = self.yaw
            path_angle = self.path_angle()
            
            error_diff = err - previous_err
            theta_err = self.normalize_angle(path_angle - current_angle)
            
            P = min(0.7, self.kp * err)
            I = self.ki * integral
            D = self.kd * error_diff
            T = self.kp_theta * theta_err

            ## control command
            twist.linear.x = min(P + I + D, 0.6)
            twist.angular.z = T
            self.cmd_publisher.publish(twist)

            ## update parameters
            previous_angle = current_angle
            previous_err = err
            integral += err * self.dt

            ## sleep
            self.rate.sleep()

    def rotate(self):
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0.2

        point1 = [1, 2]
        point2 = [2, 2]
        point3 = [3, 3]
        point4 = [0 ,1]
        point5 = [1, 0]

        points = []
        points.append(point2)
        points.append(point3)
        points.append(point1)
        points.append(point4)
        points.append(point5)

        self.cmd_publisher.publish(twist)

        while not rospy.is_shutdown():
            self.cmd_publisher.publish(twist)
            print("robot angle: ", degrees(self.yaw))

            for p in points:
                path_angle = atan2((p[1] - self.y) , (p[0] - self.x))
                print("angle to ", p, " is : ", degrees(path_angle))
            rospy.sleep(0.5)

    def move_to_point(self):
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        goal = [5, 1]

        while not rospy.is_shutdown():

            err = self.distance(goal, self.x, self.y)
            P = self.kp * err
            twist.linear.x = P
            self.cmd_publisher.publish(twist)
            self.rate.sleep()


            

    def on_shutdown(self):
        ## stop robot

        ## plot pid

        rospy.sleep(1)


if __name__ == "__main__":
    controller = Controller()
    controller.run()