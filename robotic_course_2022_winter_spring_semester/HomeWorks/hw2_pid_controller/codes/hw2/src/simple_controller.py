#!/usr/bin/python3

from dis import dis
from time import sleep
import numpy as np
import rospy
import tf
import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from itertools import count, cycle
import matplotlib.pyplot as plt


X1 = np.linspace(-3, 3 , 100)
Y1 = np.array([2]*100)

Y2 = np.linspace(2, -2 , 100)
X2 = np.array([3]*100)

X3 = np.linspace(3, -3 , 100)
Y3 = np.array([-2]*100)

Y4 = np.linspace(-2, 2 , 100)
X4 = np.array([-3]*100)

class Simple_controller():

    def __init__(self) -> None:
        
        rospy.init_node("controller", anonymous=False)
        self.cmd_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.odom = rospy.Subscriber("/odom", Odometry, self.update_pose, queue_size=1)

        self.linear_speed = rospy.get_param("simple_controller/linear_speed")
        self.angular_speed = rospy.get_param("simple_controller/angular_speed")
        self.goal_angle = math.radians(rospy.get_param("simple_controller/linear_speed"))
        self.epsilon = rospy.get_param("simple_controller/epsilon")

        self.path_array = self.generate_path_array()
        self.path = cycle(self.path_array)
        self.goal = next(self.path)

        self.position_x = 0.0
        self.position_y = 0.0
        self.yaw = 0.0

        self.OFF_PATH, self.ON_PATH = 0, 1
        self.state = self.OFF_PATH

        self.errors = []
    

    def update_pose(self, msg):
        self.position_x = msg.pose.pose.position.x
        self.position_y = msg.pose.pose.position.y
        self.yaw = self.quaternion_to_euler(msg)
        

    def quaternion_to_euler(self, msg):
        orientation = msg.pose.pose.orientation
        # convert quaternion to odom
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((
            orientation.x ,orientation.y ,orientation.z ,orientation.w
        )) 
        return yaw
    

    def generate_path_array(x=1):
        return [
            [3, 2],
            [-3, 2],
            [-3, -2],
            [3, -2]
        ]


    def rotate_90degree(self, degree):
        remaining = math.radians(degree)
        prev_angle = abs(self.yaw)
        
        twist = Twist()
        twist.angular.z = self.angular_speed
        self.cmd_publisher.publish(twist)

        while remaining >= self.epsilon:
            current_angle = abs(self.yaw)
            delta = abs(prev_angle - current_angle)
            remaining -= delta
            print("yaw: ", math.degrees(current_angle), " remaning: ", math.degrees(remaining))
            prev_angle = current_angle
        
        twist.angular.z = 0
        self.cmd_publisher.publish(twist)
        self.cmd_publisher.publish(Twist())
        rospy.sleep(1)          


    def rotate(self, goal_x, goal_y):
        inc_x = goal_x - self.position_x
        inc_y = goal_y - self.position_y
        angle_to_goal = math.atan2(inc_y, inc_x)

        print(math.degrees(angle_to_goal), math.degrees(self.yaw))

        twist = Twist()
        twist.angular.z = self.angular_speed
        twist.linear.x = 0
        self.cmd_publisher.publish(twist)
        
        while abs(angle_to_goal - self.yaw) > math.radians(5):
            print(math.degrees(angle_to_goal), math.degrees(self.yaw))
            
        self.cmd_publisher.publish(Twist())
        rospy.sleep(1)        

    
    def move(self, distance):
        travelled_dist = 0
        start_x = self.position_x
        start_y = self.position_y

        twist = Twist()

        while distance > travelled_dist:
            twist.linear.x = self.linear_speed  
            self.cmd_publisher.publish(twist)
            travelled_dist = abs(self.position_y - start_y) + abs(self.position_x - start_x) 

            self.errors.append(min(min(abs(X1 - self.position_x) + abs(Y1 - self.position_y)), min(abs(X2 - self.position_x) + abs(Y2 - self.position_y)), 
                               min(abs(X3 - self.position_x) + abs(Y3 - self.position_y)), min(abs(X4 - self.position_x) + abs(Y4 - self.position_y))))
            
            print("distance: ", distance, " travelled: ", travelled_dist)
        
        self.cmd_publisher.publish(Twist())
        rospy.sleep(1)


    def calculate_distance(self):
        if self.state == self.OFF_PATH:
            dist = math.sqrt(math.pow( (3 - self.position_x) , 2) + math.pow( (0 - self.position_y ) , 2))
        else:
            # dist = math.sqrt(math.pow( (self.goal[0] - self.position_x) , 2) + math.pow( (self.goal[1] - self.position_y ) , 2))
            dist = abs(self.goal[0]  - self.position_x) + abs(self.goal[1] - self.position_y) 
        print("distance: ", dist, " goal: ", self.goal[0], self.goal[1], " start point: ", self.position_x, self.position_y)
        return dist


    def controller(self):
        counter = 0
        rospy.sleep(1)
        while not rospy.is_shutdown():
            if self.state == self.OFF_PATH:
                ## beginning of the code
                dist = self.calculate_distance()
                self.move(dist)
                # self.rotate(self.goal[0], self.goal[1])
                self.rotate_90degree(90)
                self.state = self.ON_PATH
            print("goal: ", self.goal[0], self.goal[1])
            dist = self.calculate_distance()
            self.move(dist)
            self.goal = next(self.path)
            # self.rotate(self.goal[0], self.goal[1])
            self.rotate_90degree(90)
            
            if counter == 5:
                break
            counter += 1

        plt.title("AVG error: {:.2f}".format(sum(self.errors)/len(self.errors)))
        plt.plot(list(range(1, len(self.errors) + 1)), self.errors)
        plt.show()
            

if __name__ == "__main__":
    simple_controller = Simple_controller()
    simple_controller.controller()