#!usr/bin/python3

from locale import normalize
from turtle import distance
import rospy
import tf
import numpy as np

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from itertools import count, cycle

from math import atan2, radians, degrees, pi


class Controller():
    def __init__(self):
        rospy.init_node("pid_controller", anonymous=False)
        rospy.on_shutdown(self.on_shutdown)

        self.cmd_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.odom = rospy.Subscriber("/odom", Odometry, self.update_pose, queue_size=1)

        self.path_array = self.rectangle()
        self.path = cycle(self.path_array)
        self.goal = next(self.path)

        self.x = 0
        self.y = 0
        self.yaw = 0

        self.kp = 1
        self.ki = 0
        self.kd = 0
        self.distance_margin = 0.2

        self.kp_theta = 0.3

        self.dt = 0.005
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


    def distance(goal, x, y):
        dist = ((goal[0] - x)**2 + (goal[1])**2)**0.5
        print("dist: ", dist)
        return dist
    

    def normalize_angle(current_angle, previous_angle):
        if previous_angle > pi-0.1 and current_angle <= 0:
            current_angle = 2*pi + current_angle
        elif previous_angle < -pi+0.1 and current_angle > 0:
            current_angle = -2*pi + current_angle
        print("current angle: ", current_angle)
        return current_angle
    

    def path_angle(self):
        path_angle = atan2((self.goal[1] - self.y) , (self.goal[0] - self.x))
        if path_angle < -pi/4 or path_angle > pi/4:
            if self.y < 0 and self.yaw < self.y:
                path_angle = -2*pi + path_angle
            elif self.y >= 0 and self.yaw > self.y:
                path_angle = 2*pi + path_angle
        print("path angle: ", path_angle)
        return path_angle


    def run(self):

        previous_angle = self.yaw
        previous_err = self.distance(self.goal, self.x, self.y)
        error_diff = 0
        integral = 0

        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0

        while not rospy.is_shutdown():

            err = self.distance(self.goal, self.x, self.y)

            ## check if robot reach the margin point - if true change goal point to next point on path
            if err < self.distance_margin:
                self.goal = next(self.path)
                continue


            ## control equations
            current_angle = self.yaw
            current_angle = self.normalize_angle(current_angle, previous_angle)
            path_angle = self.path_angle()

            error_diff = err - previous_err
            
            P = self.kp * err
            I = self.ki * integral
            D = self.kd * error_diff

            theta_err = path_angle - current_angle
            T = self.kp_theta * theta_err

            print("P: ", P, " T: ", T)

            ## control command
            twist.linear.x = P + I + D
            twist.angular.z = T
            self.cmd_publisher.publish(twist)

            ## update parameters
            previous_angle = current_angle
            previous_err = err
            integral += err * self.dt

            ## sleep
            self.rate.sleep()

            

    def on_shutdown(self):
        ## stop robot

        ## plot pid

        rospy.sleep(1)


if __name__ == "__main__":
    controller = Controller()
    controller.run()