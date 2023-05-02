#!/usr/bin/env python3

from cmath import cos
from turtle import distance
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
from LaserDataBrat import LaserDataBrat

class BrattenExplorer:

    def __init__(self):
        print("EXPLORING 1")
        rospy.init_node('bratten_explorer')
        # set up subscriber to get laser scan data
        # self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.process_scan)

        # set up publisher to send movement commands
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # set the linear and angular speeds for movement
        self.linear_speed = 0.18    # maximum is 0.26
        self.angular_speed = 0.8    # max is 1.82

        # TUNING PARAMETERS
        self.weight_r1 = 2
        self.weight_r2 = 1
        self.weight_l1 = 2
        self.weight_l2 = 1

        # initialize the movement command
        self.move_cmd = Twist()
        self.LaserDataBrat = LaserDataBrat()

        # set up state machine
        # self.states = {"stop_moving":self.stop_moving,"Heading_Wall":self.Heading_Wall,"obstacle_avoidance":self.obstacle_avoidance}
        # self.current_state = 'stop_moving'

        # start exploring
        self.explore()

    def weighted_mean(self):
        
        view_data = [self.LaserDataBrat.l2, self.LaserDataBrat.l1, self.LaserDataBrat.r1, self.LaserDataBrat.r2]

        return -view_data[0]*self.weight_l2 + -view_data[1]*self.weight_l1 +  view_data[2]*self.weight_r1 +  view_data[3]*self.weight_r2

    def move(self, fwd_vel, ang_vel):

        # set movement command for moving forward
        self.move_cmd.linear.x = fwd_vel
        self.move_cmd.angular.z = ang_vel
        self.cmd_vel_pub.publish(self.move_cmd)

    def stop(self):

        # set movement command for moving forward
        self.move_cmd.linear.x = 0
        self.move_cmd.angular.z = 0
        self.cmd_vel_pub.publish(self.move_cmd)

    def explore(self):
        print("EXPLORING 2")
        # continue exploring until the node is stopped
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            
            # if self.flag==0:
            #     # self.Heading_Wall()
            #     self.current_state = 'Heading_Wall'
            # elif self.flag==1:
            #     # print(self.flag)
            #     # self.obstacle_avoidance()
            #     # print()
            #     self.current_state = 'obstacle_avoidance'

            # get new weighted mean for distances
            weight = self.weighted_mean()
            ang_v = weight * 1.82

            print(f"Weighted mean: {weight}, angular velocity: {ang_v}")
            self.move(0.1, ang_v)

            rate.sleep()

        return 0

if __name__ == '__main__':
    BrattenExplorer()
