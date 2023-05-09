#!/usr/bin/env python3

from cmath import cos
from turtle import distance
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math, numpy as np
from Laserdata import LaserData

# Import the tb3 modules from tb3.py
from tb3 import Tb3Move, Tb3Odometry, Tb3LaserScan

class Explorer:

    def scan_callback(self, scan_data: LaserScan):

        # when new message from topic received, data pushed into LaserScan message
        left_arc = scan_data.ranges[0:21]       # left bits of arc (first 20)
        right_arc = scan_data.ranges[-20:]      # right bits of arc, last 20 of 360
        front_arc = np.array(left_arc[::-1] + right_arc[::-1])      # merge into 1 array
        
        self.min_distance = front_arc.min()     # thing thats closest to the robot!

        # Optional Extra; where exactly is the nearest object?
        arc_angles = np.arange(-20, 21)
        self.object_angle = arc_angles[np.argmin(front_arc)]
        print(f"in laser scan, min dis is {self.min_distance}")

    def __init__(self) -> None:

        rospy.init_node('explorer')

        # set up subscriber to get laser scan data
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        # set up publisher to send movement commands
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # initialize the movement command
        self.move_cmd       = Twist()
        self.tb3_odom       = Tb3Odometry()
        self.tb3_lidar      = Tb3LaserScan()

        self.explore()



    def move_fwd(self, speed):

        # set movement command for moving forward
        self.move_cmd.linear.x = speed
        self.move_cmd.angular.z = 0
        self.cmd_vel_pub.publish(self.move_cmd)

    def stop_moving(self):

        # set movement command for moving forward
        self.move_cmd.linear.x = 0
        self.move_cmd.angular.z = 0
        self.cmd_vel_pub.publish(self.move_cmd)

    def explore(self):
        # continue exploring until the node is stopped
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            
            while  self.tb3_lidar.min_distance > 0.5:
                
                print(f"Going forward : min dis: {self.tb3_lidar.min_distance}")
                # go forwards until hit an object and stop
                self.move_fwd(0.15)
            
            print("Found wall, stopping!")

            self.stop_moving()

            rate.sleep()

if __name__ == '__main__':
    Explorer()
