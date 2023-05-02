#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from math import degrees
import math
import numpy as np


class Tb3Move(object):
    def __init__(self):
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.publisher_rate = rospy.Rate(10) # Hz
        self.vel_cmd = Twist()

    def set_move_cmd(self, linear = 0.0, angular = 0.0):
        self.vel_cmd.linear.x = linear
        self.vel_cmd.angular.z = angular
    
    def publish(self):
        self.publisher.publish(self.vel_cmd)
    
    def stop(self):
        self.set_move_cmd()
        self.publish()

class Tb3Odometry(object):
    def odom_cb(self, odom_data):
        orientation = odom_data.pose.pose.orientation
        position = odom_data.pose.pose.position
        (_, _, yaw) = euler_from_quaternion([orientation.x,
            orientation.y, orientation.z, orientation.w],'sxyz')
        
        self.yaw = self.round(degrees(yaw), 4)
        self.posx = self.round(position.x, 4)
        self.posy = self.round(position.y, 4)
    
    def __init__(self):
        self.posx = 0.0
        self.posy = 0.0
        self.yaw = 0.0
        self.subscriber = rospy.Subscriber('/odom', Odometry, self.odom_cb)
    
    def round(self, value, precision):
        value = int(value * (10**precision))
        return float(value) / (10**precision)

class Tb3LaserScan(object):
    def laserscan_cb(self, scan_data):
        left_arc = scan_data.ranges[0:20]
        right_arc = scan_data.ranges[-20:]
        front_arc = np.array(left_arc[::-1] + right_arc[::-1])
        
        self.min_distance = front_arc.min()
        arc_angles = np.arange(-20, 20)
        self.closest_object_position = arc_angles[np.argmin(front_arc)]

    def __init__(self):
        self.min_distance = 0.0
        self.closest_object_position = 0.0 # degrees
        self.subscriber = rospy.Subscriber('/scan', LaserScan, self.laserscan_cb) 

class Tb3LaserScan_left(object):
    def laserscan_cb_left(self, scan_data):
        left_upper_arc = scan_data.ranges[91:101]
        left_lower_arc = scan_data.ranges[70:90]
        left_front_arc = np.array(left_upper_arc[::-1] + left_lower_arc[::-1])
        
        self.min_distance_left = left_front_arc.min()
        # arc_angles_left = np.arange(70, 111)
        # self.closest_object_position = arc_angles_left[np.argmin(left_front_arc)]

    def __init__(self):
        self.min_distance_left = 0.0
        self.closest_object_position = 0.0 # degrees
        self.subscriber = rospy.Subscriber('/scan', LaserScan, self.laserscan_cb_left) 

class Tb3LaserScan_right(object):
    def laserscan_cb_right(self, scan_data):
        right_upper_arc = scan_data.ranges[-111:-91]
        right_lower_arc = scan_data.ranges[-90:-70]
        right_front_arc = np.array(right_upper_arc[::-1] + right_lower_arc[::-1])
        
        self.min_distance_right = right_front_arc.min()
        # arc_angles_right = np.arange(-90, -111)
        # self.closest_object_position = arc_angles_right[np.argmin(right_front_arc)]

    def __init__(self):
        self.min_distance_right = 0.0
        self.closest_object_position = 0.0 # degrees
        self.subscriber = rospy.Subscriber('/scan', LaserScan, self.laserscan_cb_right) 
class AStar:
    def __init__(self):
        # initialize laser scan and odometry subscribers
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        # initialize obstacle list
        self.obstacles = []

    def laser_callback(self, msg):
        # clear obstacle list
        self.obstacles = []
        # get minimum and maximum angles for scan range
        min_angle = msg.angle_min
        max_angle = msg.angle_max
        # iterate over scan range and add obstacles to list
        for i in range(len(msg.ranges)):
            # get angle of current scan reading
            angle = min_angle + i * msg.angle_increment
            # check if scan reading is within 45 degree range in front of robot
            if abs(angle) < math.pi/4:
                # check if scan reading is within minimum range (0.5 meters)
                if msg.ranges[i] < 0.5:
                    # calculate x and y coordinates of obstacle in robot frame
                    x = msg.ranges[i] * math.cos(angle)
                    y = msg.ranges[i] * math.sin(angle)
                    # transform obstacle to global frame using current odometry data
                    obstacle_x = self.current_x + x*math.cos(self.current_theta) - y*math.sin(self.current_theta)
                    obstacle_y = self.current_y + x*math.sin(self.current_theta) + y*math.cos(self.current_theta)
                    print(self.current_theta)
                    # add obstacle to list
                    self.obstacles.append((obstacle_x, obstacle_y))

    def odom_callback(self, msg):
        # get current x, y, and theta from odometry message
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        _, _, self.current_theta = euler_from_quaternion(quaternion)

    def collision_check(self, x, y):
        # check if given coordinates are inside any obstacle
        for obstacle in self.obstacles:
            obstacle_x = obstacle[0]
            obstacle_y = obstacle[1]
            distance = math.sqrt((x-obstacle_x)**2 + (y-obstacle_y)**2)
            if distance < 0.2:
                return True
        return False

    def a_star_search(self, start_x, start_y, goal_x, goal_y):
        # define heuristic function (Euclidean distance)
        def heuristic(a, b):
            return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

        # define cost function (distance between adjacent nodes)
        def cost(a, b):
            return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

        # define function to get neighboring nodes
        def neighbors(point):
            x, y = point
            return [(x+0.2, y), (x-0.2, y), (x, y+0.2), (x, y-0.2)]

        # initialize start and goal nodes
        start = (start_x, start_y)
        goal = (goal_x, goal_y)
        # initialize frontier and explored sets
        frontier = [(0, start)]
        explored = set()

        # initialize dictionaries to keep track of parent nodes and g values
        parents = {}
        g_values = {start: 0}
        # iterate until goal is found or frontier is empty
        while frontier:
            # pop node with lowest f value from frontier
            current_f, current_node = min(frontier)
            frontier.remove((current_f, current_node))
            # print(current_f)
            # print(current_node)
            # check if current node is goal node
            if current_node == goal:
                path = [current_node]
                while path[-1] != start:
                    path.append(parents[path[-1]])
                path.reverse()
                return path

            # add current node to explored set
            explored.add(current_node)

            # expand current node
            for neighbor in neighbors(current_node):
                # check if neighbor is already explored
                if neighbor in explored:
                    continue
                if self.obstacles:
                    print(self.obstacles)
                if self.obstacles and self.collision_check(neighbor[0], neighbor[1], self.obstacles):
                    continue
                # calculate tentative g value and add neighbor to frontier
                tentative_g = g_values[current_node] + cost(current_node, neighbor)
                if (tentative_g, neighbor) not in frontier:
                    parents[neighbor] = current_node
                    g_values[neighbor] = tentative_g
                    frontier.append((tentative_g + heuristic(neighbor, goal), neighbor))

        # return empty path if no path was found
        return []