#!/usr/bin/env python3

from cmath import cos
from turtle import distance
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
from Laserdata import LaserData

class Explorer:
    def __init__(self):
        rospy.init_node('explorer')
        # set up subscriber to get laser scan data
        # self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.process_scan)

        # set up publisher to send movement commands
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # set the linear and angular speeds for movement
        self.linear_speed = 0.2
        self.angular_speed = 0.4
        self.threshold= 1
        self.thres_obstacle = 0.4
        self.idx=[]
        self.flag=0
        self.flag_FW=0
        self.turn_finish=0
        self.distance=[]
        #1st PID
        self.SideWallDistance = 0.3
        self.Kp_SideWall=2
        self.Ki_SideWall=0
        self.Kd_SideWall=100
        self.error_SideWall=0
        self.errorSum_SideWall=0
        #2nd PID
        self.SideWallDistance_Sec=0.15
        self.secPID_tres_open=self.SideWallDistance+0.35
        self.secPID_tres_close=self.SideWallDistance/math.cos(20)
        self.Kp_SideWall_Sec=2
        self.Ki_SideWall_Sec=0
        self.Kd_SideWall_Sec=50
        self.error_SideWall_Sec=0
        self.errorSum_SideWall_Sec=0
        


        # initialize the movement command
        self.move_cmd = Twist()
        self.LaserData = LaserData()
        # set up state machine
        self.states = {'moving': self.move, 'turning_left': self.turnleft,'turning_right': self.turnright,'turning_180': self.turn180,"stop":self.stop_moving,"obstacle_avoidance":self.obstacle_avoidance}
        self.current_state = 'moving'

        # start exploring
        self.explore()


        

    def PID_WallFollower(self,SideDist):
        #   PID loop
        errorOld_SideWall = self.error_SideWall;       # Save the old error for differential component
        self.error_SideWall = self.SideWallDistance - SideDist  # Calculate the error in position
        self.errorSum_SideWall = self.errorSum_SideWall+self.error_SideWall
        proportional = self.error_SideWall * self.Kp_SideWall  # Calculates Proportional Error

        integral = self.errorSum_SideWall * self.Ki_SideWall # Calculates Steady State Error

        differential = (self.error_SideWall - errorOld_SideWall) * self.Kd_SideWall # Calculates Rate of Error Change

        output_SideWall = proportional + self.constrain(integral,-50,0) + differential  # Calculate the result
        return output_SideWall
    def PID_Sec(self,SideDist):
        #   PID loop
        errorOld_SideWall = self.error_SideWall_Sec;       # Save the old error for differential component
        self.error_SideWall_Sec = self.SideWallDistance_Sec - SideDist  # Calculate the error in position
        self.errorSum_SideWall_Sec = self.errorSum_SideWall_Sec+self.error_SideWall_Sec
        proportional = self.error_SideWall_Sec * self.Kp_SideWall_Sec  # Calculates Proportional Error

        integral = self.errorSum_SideWall_Sec * self.Ki_SideWall_Sec # Calculates Steady State Error

        differential = (self.error_SideWall_Sec - errorOld_SideWall) * self.Kd_SideWall_Sec # Calculates Rate of Error Change

        output_SideWall = proportional + self.constrain(integral,-50,0) + differential  # Calculate the result
        return output_SideWall
        
    def constrain(self,val, min_val, max_val):
        return min(max_val, max(min_val, val))

    def arena_dis(self, distance):
        indices = []
        for i, x in enumerate(distance):
            if x > self.threshold:
                indices.append(i+1)
        return indices
    def obtain_dis(self):
        front_dis= self.LaserData.front_dis
        left_dis = self.LaserData.left_dis
        right_dis= self.LaserData.right_dis
        back_dis = self.LaserData.back_dis


        distance=[front_dis,left_dis,right_dis,back_dis]
        idx=self.arena_dis(distance)
        return idx,distance

    def start_direction(self,idx):
        print("sd")
        print("idx[0]",idx[0])
        if idx[0]==2:
            self.current_state = "turning_left"

        elif idx[0]==3:
            self.current_state = "turning_right"

        elif idx[0]==4:
            self.current_state = "turning_180"

    def move(self):

        # set movement command for moving forward
        self.move_cmd.linear.x = self.linear_speed
        self.move_cmd.angular.z = 0
        self.cmd_vel_pub.publish(self.move_cmd)

    def stop_moving(self):

        # set movement command for moving forward
        self.move_cmd.linear.x = 0
        self.move_cmd.angular.z = 0
        self.cmd_vel_pub.publish(self.move_cmd)

    def turnleft(self):
        rate = rospy.Rate(10) # 10 Hz
        # set movement command for turning away from obstacle
        self.move_cmd.linear.x = 0
        self.move_cmd.angular.z = self.angular_speed
        turn_duration = math.pi / 2 / self.angular_speed # 90 degrees in radians
        t0 = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - t0 < turn_duration:
            self.cmd_vel_pub.publish(self.move_cmd)
            rate.sleep()
            print("uuuuu")
        self.move_cmd.angular.z = 0
        self.cmd_vel_pub.publish(self.move_cmd)
        # rate.sleep()
        
    def turnright(self):
        rate = rospy.Rate(10) # 10 Hz
        # set movement command for turning away from obstacle
        self.move_cmd.linear.x = 0.01
        self.move_cmd.angular.z = -self.angular_speed
        turn_duration = math.pi  /2/ self.angular_speed# 90 degrees in radians
        t0 = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - t0 < turn_duration:
            self.cmd_vel_pub.publish(self.move_cmd)
            rate.sleep()
        print("finish")
        self.move_cmd.angular.z = 0
        self.cmd_vel_pub.publish(self.move_cmd)

    def turn180(self):
        rate = rospy.Rate(10) # 10 Hz
        # set movement command for turning away from obstacle
        self.move_cmd.linear.x = 0
        self.move_cmd.angular.z = self.angular_speed
        turn_duration = math.pi/ self.angular_speed # 90 degrees in radians
        t0 = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - t0 < turn_duration:
            self.cmd_vel_pub.publish(self.move_cmd)
            rate.sleep()
            print("180")
        self.move_cmd.angular.z = 0
        self.cmd_vel_pub.publish(self.move_cmd)
    def wall_follower(self,distance):

        turn_output=round(self.PID_WallFollower(distance),3)
        # print("turn_output",turn_output)
        # set movement command for wall following
        self.move_cmd.linear.x = self.linear_speed
        self.move_cmd.angular.z = self.constrain(turn_output,-1.82,1.82)
        print("z=",self.move_cmd.angular.z)
        self.cmd_vel_pub.publish(self.move_cmd)
    def wall_turning(self,distance):

        turn_output=round(self.PID_Sec(distance),3)
        # print("SEC_output",turn_output)
        
        # set movement command for wall following
        self.move_cmd.linear.x = self.linear_speed
        self.move_cmd.angular.z = self.constrain(turn_output,-1.82,1.82)
        self.cmd_vel_pub.publish(self.move_cmd)
        # print("sec_z=",self.move_cmd.angular.z)

    def obstacle_avoidance(self):
        self.idx,self.distance=self.obtain_dis()
        right_upper=self.LaserData.right_upper
        right_lower=self.LaserData.right_lower
        print("secPID_tres_close",self.secPID_tres_close+0.05)
        if right_upper>self.secPID_tres_open:
            if self.distance[0]>self.thres_obstacle:
                # print("self.LaserData.right_upper",self.LaserData.right_upper)
                print("sec_2dis",round(self.LaserData.right_upper*math.cos(40),2))
                self.wall_turning(round(self.LaserData.right_upper*math.cos(40),2))
            else:
                self.turnleft()
            # self.wall_turning(self.distance[2]
        elif right_upper<self.secPID_tres_close+0.05:
            if self.distance[0]>self.thres_obstacle:
                # self.wall_follower(round(self.LaserData.right_upper*math.cos(40),2))
                self.wall_follower(self.SideWallDistance)
            else:
                self.turnleft()

           
            
    def explore(self):
        # continue exploring until the node is stopped
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            if self.flag==0:
                self.idx,self.distance=self.obtain_dis() 
                print(self.idx)   
                print(self.distance)     
                while len(self.idx)==0:
                    rate1=rospy.Rate(1)
                    self.current_state = 'moving' 
                    # rospy.sleep(1)
                    rate1.sleep()
                    self.stop_moving()
                    self.idx,self.distance=self.obtain_dis()
                    # print(self.distance)
                    # print(self.idx)
                self.start_direction(self.idx)
                # self.states[self.current_state]()``
                self.idx,self.distance=self.obtain_dis()
                if self.distance[0]>self.thres_obstacle:
                    self.current_state = 'moving'
                else:
                    self.current_state = 'turning_left'   
                    # self.current_state = 'stop'                    
                    self.flag=1
            elif self.flag==1:
                
                # print(self.flag)
                # self.obstacle_avoidance()
                self.current_state = 'obstacle_avoidance'

                

            self.states[self.current_state]()

            rate.sleep()

if __name__ == '__main__':
    Explorer()

