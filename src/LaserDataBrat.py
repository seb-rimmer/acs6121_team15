#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from statistics import mean
# from laser_filters.srv import LaserScanMLSFilter

class LaserDataBrat(object):
    
    def laserscan_cb(self, msg):

        self.l2 = round(mean(msg.ranges[280:315]), 2)
        self.l1 = round(mean(msg.ranges[315:359]), 2)
        self.r1 = round(mean(msg.ranges[0:45]), 2)
        self.r2 = round(mean(msg.ranges[45:80]), 2)

        # self.front_dis = round(min(mean(msg.ranges[0:15]), mean(msg.ranges[345:359])), 2)
        # self.left_dis = round(mean(msg.ranges[75:105]), 2)
        # self.right_dis = round(mean(msg.ranges[255:285]), 2)
        # self.back_dis = round(mean(msg.ranges[145:215]), 2)
        # self.right_lower = round(mean(msg.ranges[255:270]), 2) # PID second stage switch
        # self.right_upper = round(mean(msg.ranges[282:285]), 2) # PID second stage switch
        # self.front_obs = round(min(mean(msg.ranges[0:30]), mean(msg.ranges[330:359])), 2)
    
    def __init__(self):
        
        self.l2 = 0
        self.l1 = 0
        self.r1 = 0
        self.r2 = 0 

        # self.front_dis = 0
        # self.left_dis = 0
        # self.right_dis = 0
        # self.back_dis = 0
        # self.right_lower = 0
        # self.right_upper = 0
        # self.front_obs= 0
        self.subscriber = rospy.Subscriber('/scan', LaserScan, self.laserscan_cb)