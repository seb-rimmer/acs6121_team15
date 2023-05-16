#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from scipy.ndimage import gaussian_filter1d, median_filter


class Sensor(object):
    def laserscan_cb(self, msg):
        Angular = np.array(msg.ranges)
        Angular = [r if r > 0.2 else float("inf") for r in msg.ranges]
        Angular = [min(max(r, 0.2), 3) for r in Angular]
        Angular = self.median_filter(Angular)
        self.F_dist = round(np.min(Angular[0:25].tolist() + Angular[345:359].tolist()), 2)
        self.L_dist = round(np.min(Angular[75:105]), 2)
        self.R_dist = round(np.min(Angular[255:285]), 2)
        self.B_dist = round(np.min(Angular[145:215]), 2)
        self.right_upper = round(np.mean(Angular[282:285]), 2) 
        self.front_obs = round(np.min(Angular[0:35].tolist() + Angular[325:359].tolist()), 2)#180 turning
        #initial
        self.F_dist_inl=round(np.min(Angular[0:15].tolist() + Angular[345:359].tolist()), 2)
        self.L_dist_inl = round(np.min(Angular[75:105]), 2)
        self.R_dist_inl = round(np.min(Angular[255:285]), 2)

    def median_filter(self, data, size=3):
        return median_filter(data, size)

    def __init__(self):
        self.F_dist_inl=0
        self.L_dist_inl=0
        self.R_dist_inl=0
        self.F_dist = 0
        self.L_dist = 0
        self.R_dist = 0
        self.B_dist = 0
        self.right_lower = 0
        self.right_upper = 0
        self.front_obs = 0

        self.subscriber = rospy.Subscriber('/scan', LaserScan, self.laserscan_cb)

if __name__ == '__main__':
    rospy.init_node('laser_data_node', anonymous=True)
    laser_data = Sensor()
    rospy.spin()
