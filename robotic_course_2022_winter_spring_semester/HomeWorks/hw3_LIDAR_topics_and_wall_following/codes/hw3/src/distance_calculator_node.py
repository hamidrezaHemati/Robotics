#!/usr/bin/python3
from dis import dis
import rospy
from sensor_msgs.msg import LaserScan
from hw3.srv import GetDistance, GetDistanceResponse
from nav_msgs.msg import Odometry


class Distance_calculator():
    def __init__(self) -> None:
        
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

        self.distances = {
        "bookshelf" : -1.0,
        "dumpster" : -1.0,
        "barrel" : -1.0,
        "postbox"	: -1.0,
        "brick_box" : -1.0,
        "cabinet"	: -1.0,
        "cafe_table" : -1.0,
        "fountain" : -1.0
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


    def update_distances(self):
        self.update_pose()
        for key, value in self.position_of_obstacles.items():
            self.distances[key] = self.distance_from_obstacle(value[0], value[1])
        

    def get_distance(self, req):
        distance = -1
        obst_name = req.obst_name
        print("obstacle name: ", obst_name)
        for key, value in self.distances.items():
            if key == obst_name:
                distance = value
                res = GetDistanceResponse()
                res.distance = distance
                print("distanc: ", distance)
                return res 
        rospy.logerr(f'direction_name is not valid: {obst_name}')
        return None



def listener():
    rospy.init_node('distance_calculator_node', anonymous=True)
    dc = Distance_calculator()
    rospy.Subscriber("/odom", Odometry, dc.update_distances)
    s = rospy.Service('/get_distance', GetDistance, dc.get_distance)
    rospy.spin()


if __name__ == '__main__':
    listener()