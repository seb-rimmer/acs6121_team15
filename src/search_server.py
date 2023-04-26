#! /usr/bin/env python3
#
# The action server should make the robot move forwards until it detects an obstacle up ahead.
#
# Similarly to the Service Server that you created last week, your Action Server here should be
# configured to accept two goal parameters:
#
#      1.  The speed (in m/s) at which the robot should move forwards when the action server is called. 
#          Consider doing some error checking on this to make sure a velocity request is less than the 
#          maximum speed that the robot can actually achieve (0.26 m/s)!
#
#      2.  The distance (in meters) at which the robot should stop ahead of any objects or boundary walls 
#          that are in front of it. To do this you'll need to subscribe to the /scan topic. Be aware that 
#          an object won't necessarily be directly in front of the robot, so you may need to monitor a 
#          range of LaserScan data points (within the ranges array) to make the collision avoidance effective 
#          (recall the LaserScan callback example and also have a look at the Tb3LaserScan class within the 
#          tuos_ros_examples/tb3.py module that might help you with this).
#          
#      Whilst your server performs its task it should provide the following feedback to the Action Caller:
#
#      1. The distance travelled (in meters) since the current action was initiated.
#
#         To do this you'll need to subscribe to the /odom topic. Remember that there's a Tb3Odometry class
#         within the tuos_ros_examples/tb3.py module that might help you with obtaining this data.
# 
#         Remember also that your robot's orientation shouldn't change over the course of a single action call,
#         only its linear.x and linear.y positions should vary. Bear in mind however that the robot won't 
#         necessarily be moving along the X or Y axis, so you will need to consider the total distance travelled 
#         in the X-Y plane. You should have done this in the Week 3 move_square exercise, so refer to this if 
#         you need a reminder.
# 
#      Finally, on completion of the action, your server should provide the following three result parameters:
# 
#      1. The total distance travelled (in meters) over the course of the action.
#
#      2. The distance to the obstacle that made the robot stop (this should match, or very close to, the 
#         distance that was provided by the Action Client in the goal).
#
#      3. The angle (in degrees) at which this obstacle is located in front of the robot (Tb3LaserScan class 
#         within the tuos_ros_examples/tb3.py module, which may already provide this).
# 
# 
#       search_server.py

# Import the core Python modules for ROS and to implement ROS Actions:
from traceback import format_list
import rospy
import actionlib

# Import all the necessary ROS message types:
from tuos_ros_msgs.msg import SearchAction, SearchFeedback, SearchResult, SearchGoal

# Import the tb3 modules from tb3.py
from tb3 import Tb3Move, Tb3Odometry, Tb3LaserScan

# Import some other useful Python Modules
from math import sqrt, pow

class SearchActionServer():
    feedback = SearchFeedback() 
    result = SearchResult()

    def __init__(self):

        self.actionserver = actionlib.SimpleActionServer(
                            "/search_action_server",     # what we name our action
                            SearchAction,                # using search action  
                            self.action_server_launcher, # callback function
                            auto_start=False)

        # pull in some useful publisher/subscriber functions from the tb3.py module:
        self.vel_controller = Tb3Move()
        self.tb3_odom       = Tb3Odometry()
        self.tb3_lidar      = Tb3LaserScan()

        self.actionserver.start()
        rospy.loginfo("The 'Search Action Server' is active...")

    # The action's "callback function":
    def action_server_launcher(self, goal: SearchGoal):
        rate = rospy.Rate(10)

        vel = goal.fwd_velocity
        dist = goal.approach_distance
        success = True

        if vel > 0.26 or vel < 0:
            # print("invalid velocity, only 0 < v < 0.26 m/s")
            success = False
        
        if dist < 0.2:
            success = False     # to avoid getting too close to something

        if not success:
            # define results abort
            self.result.closest_object_angle = -1
            self.result.closest_object_distance = -1
            self.result.total_distance_travelled = -1
            
            self.actionserver.set_aborted(self.result)
            return

        print(f"\n#####\n"
            f"The 'search_action_server' has been called.\n"
            f"Goal: move forward at {vel:.3f} m/s and stop at {dist:.3f}m from an object if detected ahead...\n\n"
            f"Commencing the action...\n"
            f"#####\n")

        # Get the robot's current odometry from the Tb3Odometry() class:
        self.posx0 = self.tb3_odom.
        self.posy0 = self.tb3_odom.posy
        # Get information about objects up ahead from the Tb3LaserScan() class:
        self.closest_object = self.tb3_lidar.min_distance
        self.closest_object_location = self.tb3_lidar.closest_object_position

        self.vel_controller.set_move_cmd(vel, 0)

        while  self.tb3_lidar.min_distance > dist:
            
            # update LaserScan data:
            self.closest_object = self.tb3_lidar.min_distance
            self.closest_object_location = self.tb3_lidar.closest_object_position

            ## TODO: publish a velocity command to make the robot start moving 
            self.vel_controller.publish()

            # determine how far the robot has travelled so far:
            self.distance = sqrt(pow(self.posx0 - self.tb3_odom.posx, 2) + pow(self.posy0 - self.tb3_odom.posy, 2))

            self.result.closest_object_distance = self.closest_object
            self.result.closest_object_angle = self.closest_object_location
            self.result.total_distance_travelled = self.distance

            # check if there has been a request to cancel the action mid-way through:
            if self.actionserver.is_preempt_requested():
                ## TODO: take appropriate action if the action is cancelled (peempted)...
                
                rospy.loginfo("Pre-emtp requested, cancelling the moving action.")

                self.actionserver.set_preempted(self.result)
                
                # stop the robot:
                self.vel_controller.stop()

                success = False
                # exit the loop:
                break

            self.feedback.current_distance_travelled = self.distance
            self.actionserver.publish_feedback(self.feedback)

            rospy.loginfo(f"Have travelled {self.distance}m from start.\n")
            rospy.loginfo(f"Maintaining fwd course at {self.vel_controller.vel_cmd.linear.x}m/s with closest object {self.closest_object}m away ...\n")

            rate.sleep()

        if success:
            rospy.loginfo("approach completed successfully.")
            ## TODO: Set the action server to "succeeded" and stop the robot...
            rospy.loginfo("Search action completed successfully.")
            self.actionserver.set_succeeded(self.result)
            self.vel_controller.stop()

if __name__ == '__main__':
    rospy.init_node("search_action_server")
    SearchActionServer()
    rospy.spin()
