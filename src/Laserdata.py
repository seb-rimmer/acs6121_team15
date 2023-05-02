#!/usr/bin/env python3
from statistics import mean
import rospy
from sensor_msgs.msg import LaserScan
# from laser_filters.srv import LaserScanMLSFilter

class LaserData(object):
    def laserscan_cb(self, msg):
        self.front_dis = round(mean(min(msg.ranges[0:15]), min(msg.ranges[345:359])), 2)
        self.left_dis = round(mean(msg.ranges[75:105]), 2)
        self.right_dis = round(mean(msg.ranges[255:285]), 2)
        self.back_dis = round(mean(msg.ranges[145:215]), 2)
        self.right_lower = round(mean(msg.ranges[255:270]), 2) # PID second stage switch
        self.right_upper = round(mean(msg.ranges[282:285]), 2) # PID second stage switch
        self.front_obs = round(mean(min(msg.ranges[0:30]), min(msg.ranges[330:359])), 2)
    def __init__(self):
        self.front_dis = 0
        self.left_dis = 0
        self.right_dis = 0
        self.back_dis = 0
        self.right_lower = 0
        self.right_upper = 0
        self.front_obs= 0
        self.subscriber = rospy.Subscriber('/scan', LaserScan, self.laserscan_cb)
