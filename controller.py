#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math,os,rospkg,time
from Sensordata import Sensor
import numpy as np

class MainTask:
    def __init__(self):
        rospy.init_node('explorer')

        # set up publisher to send movement commands
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        #1st PID
        self.RightDist = 0.35
        self.Kp=1
        self.Ki=0
        self.Kd=50
        self.Ero=0
        self.EroSum=0
        # set the linear and angular speeds for movement
        self.slam_switch=0
        self.TurnRate=1.8
        self.clock=10
        self.X_speed = 0.26
        self.Z_speed = 1.2
        self.Z_speed_TurnCircle=0.7
        self.threshold_firsttoWall= 1
        self.thres_startTuring = 0.45
        self.thres_stopTurning = 0.5

        self.ind=[]
        self.stage=0
        self.InlDirec_ind=0
        self.stagebreak=0
        self.turningDown=0
        self.Any_R_upper=0
        self.distV=[]
        self.distV_inl=[]


        # initialize the movement command
        self.speed = Twist()
        self.Sensor = Sensor()
        # set up state machine
        self.states = {"Stop":self.Stop,"Go_To_Wall":self.Go_To_Wall,"Follow_Wall":self.Follow_Wall}
        self.Now_state = 'Stop'

        # start exploring
        self.explore()
    def PID(self,SideDist):
        #   PID loop
        EroOld = self.Ero;       # Save the old error for differential component
        self.Ero = self.RightDist - SideDist  # Calculate the error in position
        self.EroSum = self.EroSum+self.Ero
        proportional = self.Ero * self.Kp  # Calculates Proportional Error

        integral = self.EroSum * self.Ki # Calculates Steady State Error

        differential = (self.Ero - EroOld) * self.Kd # Calculates Rate of Error Change

        output = proportional + self.constrain(integral,-0.3,0.3) + differential  # Calculate the result
        return output
        
    def constrain(self,val, min_val, max_val):
        return min(max_val, max(min_val, val))

    def firstStage_dist(self, Dist):
        indices = []
        for i, x in enumerate(Dist):
            if x > self.threshold_firsttoWall:
                indices.append(i+1)
        return indices
    def SecStage_dist(self):
        F_dist= self.Sensor.F_dist
        L_dist = self.Sensor.L_dist
        R_dist= self.Sensor.R_dist
        B_dist = self.Sensor.B_dist
        Dist=[F_dist,L_dist,R_dist,B_dist]
        ind=self.firstStage_dist(Dist)
        return ind,Dist
    def FirstStage_dist_initial(self):
        F_dist_inl= self.Sensor.F_dist_inl
        L_dist_inl = self.Sensor.L_dist_inl
        R_dist_inl= self.Sensor.R_dist_inl
        B_dist_inl = self.Sensor.B_dist
        distance_initial=[F_dist_inl,L_dist_inl,R_dist_inl,B_dist_inl]
        idx_initial=self.firstStage_dist(distance_initial)
        return idx_initial,distance_initial

    def start_direction(self,ind):

        if ind[0]==2:
            self.turnleft()

        elif ind[0]==3:
            self.turnright()

        elif ind[0]==4:
            self.turn180()

    def move(self):

        # set movement command for moving forward
        self.speed.linear.x = self.X_speed
        self.speed.angular.z = 0
        self.cmd_vel_pub.publish(self.speed)

    def Stop(self):

        # set movement command for moving forward
        self.speed.linear.x = 0
        self.speed.angular.z = 0
        self.cmd_vel_pub.publish(self.speed)

    def turnleft(self):
        rate = rospy.Rate(self.clock) # 10 Hz
        # set movement command for turning away from obstacle
        self.speed.linear.x = 0
        self.speed.angular.z = self.Z_speed
        turn_duration = math.pi / 2 / self.Z_speed # 90 degrees in radians
        t0 = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - t0 < turn_duration:
            if (self.Sensor.front_obs<self.thres_stopTurning):

                self.cmd_vel_pub.publish(self.speed)
                rate.sleep()
            else:
                print("break")
                self.stagebreak=1
                break 
        if self.stagebreak==1:
            
            self.speed.angular.z = 0
            self.speed.linear.x=0
            self.cmd_vel_pub.publish(self.speed)
            self.stagebreak=0


        self.speed.angular.z = 0
        self.cmd_vel_pub.publish(self.speed)
        
    def turnright(self):
        rate = rospy.Rate(self.clock) # 10 Hz
        # set movement command for turning away from obstacle
        self.speed.linear.x = 0
        self.speed.angular.z = -self.Z_speed
        turn_duration = math.pi  /2/ self.Z_speed# 90 degrees in radians
        t0 = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - t0 < turn_duration:
            self.cmd_vel_pub.publish(self.speed)
            rate.sleep()
        self.speed.angular.z = 0
        self.cmd_vel_pub.publish(self.speed)

    def turn180(self):
        rate = rospy.Rate(self.clock) # 10 Hz
        # set movement command for turning away from obstacle
        self.speed.linear.x = 0
        self.speed.angular.z = self.Z_speed
        turn_duration = math.pi/ self.Z_speed # 90 degrees in radians
        t0 = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - t0 < turn_duration:
            self.cmd_vel_pub.publish(self.speed)
            rate.sleep()
        self.speed.angular.z = 0
        self.cmd_vel_pub.publish(self.speed)
    def With_Wall(self,Dist):

        turn_output=round(self.PID(Dist),3)
        # set movement command for wall following
        self.speed.linear.x = self.X_speed
        self.speed.angular.z = self.constrain(turn_output,-1.82,1.82)
        self.cmd_vel_pub.publish(self.speed)

    def Inl_direc(self):
        self.ind,self.distV_inl=self.FirstStage_dist_initial() 
        while len(self.ind)==0:
            rate1=rospy.Rate(1)
            index_of_max=np.argmax(self.distV_inl)+1
            self.start_direction([index_of_max])
            self.move()
            rate1.sleep()
            self.Stop()
            self.ind,self.distV_inl=self.FirstStage_dist_initial()
        self.start_direction(self.ind)
        self.InlDirec_ind=1
    def Go_To_Wall(self):
        if self.InlDirec_ind==0:
           self.Inl_direc() 
        self.ind,self.distV=self.SecStage_dist()
        if self.distV[0]>self.thres_startTuring:
            self.move()
        else:
            self.turnleft()              
            self.stage=1
    def Follow_Wall(self):
        self.ind,self.distV=self.SecStage_dist()
        right_upper=self.Sensor.right_upper
        degrees = 12
        radians = degrees * math.pi / 180
        cos_degrees = math.cos(radians)

        self.Any_R_upper=bool(right_upper<(self.RightDist/cos_degrees+1.4))
        if self.distV[0]>self.thres_startTuring:

            if self.Any_R_upper==1:
                self.With_Wall(self.distV[2])
            else:
                self.Point_Turning()
        else: 

             self.turnleft()      
                
    def Point_Turning(self):

        rate = rospy.Rate(self.clock) # 10 Hz

        self.speed.angular.z = -self.Z_speed_TurnCircle
        turn_duration = math.pi*1/ self.Z_speed_TurnCircle # 90 degrees in radians
        self.speed.linear.x = round(self.Z_speed_TurnCircle*(0.3+2*self.RightDist)/math.pi,2)######ralated with real robot size and sidewall dis need adjust
        t0 = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - t0 < turn_duration*self.TurnRate:######duration rate need adjust
            self.ind,self.distV=self.SecStage_dist()

            if (self.Sensor.front_obs>self.thres_stopTurning):

                self.cmd_vel_pub.publish(self.speed)
                rate.sleep()

            else:

                self.stagebreak=1
                break 

        if self.stagebreak==0:

            self.speed.angular.z = 0
            self.speed.linear.x=self.X_speed
            self.cmd_vel_pub.publish(self.speed) 
            rate1=rospy.Rate(1)
            rate1.sleep()
        else:
            self.speed.angular.z = 0
            self.speed.linear.x=0
            self.cmd_vel_pub.publish(self.speed)
            self.turnleft()
            self.stagebreak=0

              
    def explore(self):
        # continue exploring until the node is stopped
        rate = rospy.Rate(self.clock)  # 10 Hz
        # check if it's time to save the map
        map_save_interval = 10
        # get the current time
        last_map_save_time = time.time()
        while not rospy.is_shutdown():
            if self.stage==0:

                self.Now_state = 'Go_To_Wall'
            elif self.stage==1:
                self.Now_state = 'Follow_Wall'

            self.states[self.Now_state]()
            
            # check if it's time to save the map
            current_time = time.time()
            if self.slam_switch==1:
                if current_time - last_map_save_time >= map_save_interval:
                    # Save the map
                    rospack = rospkg.RosPack()
                    maps_dir = rospack.get_path('acs6121_team15') + '/maps'
                    file_path = os.path.join(maps_dir, "explore_map")
                    os.system("rosrun map_server map_saver -f {}".format(file_path))
                    # update the last map save time
                    last_map_save_time = current_time

            rate.sleep()



if __name__ == '__main__':
    MainTask()

