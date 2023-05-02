#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
# from laser_filters.srv import LaserScanMLSFilter

class LaserData(object):
    def laserscan_cb(self, msg):
        self.front_dis = round(min(min(msg.ranges[0:15]), min(msg.ranges[345:359])), 2)
        self.left_dis = round(min(msg.ranges[75:105]), 2)
        self.right_dis = round(min(msg.ranges[255:285]), 2)
        self.back_dis = round(min(msg.ranges[145:215]), 2)
        self.right_lower = round(max(msg.ranges[255:270]), 2) # PID second stage switch
        self.right_upper = round(min(msg.ranges[282:285]), 2) # PID second stage switch
        self.front_obs = round(min(min(msg.ranges[0:30]), min(msg.ranges[330:359])), 2)
    def __init__(self):
        self.front_dis = 0
        self.left_dis = 0
        self.right_dis = 0
        self.back_dis = 0
        self.right_lower = 0
        self.right_upper = 0
        self.front_obs= 0
        self.subscriber = rospy.Subscriber('/scan', LaserScan, self.laserscan_cb)
