#!/usr/bin/env python
import roslib
roslib.load_manifest('project1')
 
import sys
#import copy
import rospy
import numpy as np
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import shape_msgs.msg as shape_msgs
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import String
import tf_conversions
import math

class position_finder(object):

	def __init__(self):
		self. cube_index = []
		self.bucket_index = None
		self.number_of_cubes = 0
		self.cube_pos = geometry_msgs.msg.PoseStamped()
		self.bucket_pos = geometry_msgs.msg.PoseStamped()

		# topic msgs: [ground_plane, jaco_on_table, cube0, cube1, ... , bucket] [String, Pose, Twist] <= ModelStates
		self.pose_subscriber = rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback) # initialize subscriver
		self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory) # initialize publisher

		self.initialize_stuff(self.display_trajectory_publisher)

	def initialize_stuff(self, display_trajectory_publisher): # method to do the environment setup
	    ## First initialize moveit_commander and rospy.
	    print "============ Starting tutorial setup"
	    moveit_commander.roscpp_initialize(sys.argv)

	    self.robot = moveit_commander.RobotCommander()
	    self.scene = moveit_commander.PlanningSceneInterface()
	    self.group = moveit_commander.MoveGroupCommander("Arm")

	    print "============ Waiting for RVIZ..."
	    rospy.sleep(2)

	    ## We can get the name of the reference frame for this robot
	    print "============ Reference frame: %s" % self.group.get_planning_frame()
	    ## We can also print the name of the end-effector link for this group
	    print "============ Reference frame: %s" % self.group.get_end_effector_link()
	    ## We can get a list of all the groups in the robot
	    print "============ Robot Groups:"
	    print self.robot.get_group_names()

	    ## Sometimes for debugging it is useful to print the entire state of the robot.
	    print "============ Printing robot state"
	    print self.robot.get_current_state()
	    print "============"

	    self.scene = moveit_commander.PlanningSceneInterface()
	    self.robot = moveit_commander.RobotCommander()

	    rospy.sleep(2)

	    self.find_stuff(self.group, self.robot, display_trajectory_publisher)

	def find_stuff(self, group, robot, display_trajectory_publisher): # method to loop over every cube

	    for i in range(0,number_of_cubes):

	        ## Planning to a Pose goal - Go grab a cube
	        print "============ Generating plan 1"

	        ## Let's setup the planner
	        group.set_planning_time(1.0)
	        group.set_goal_orientation_tolerance(1)
	        group.set_goal_tolerance(1)
	        group.set_goal_joint_tolerance(0.1)
	        group.set_num_planning_attempts(10)

	        self.pose_goal_1 = group.get_current_pose().pose
	        self.pose_goal_1.orientation = self.cube_pos.pose.orientation
	        self.pose_goal_1.position = self.cube_pos.pose.position
	        print '#', i, 'cube -> pose goal:', self.pose_goal_1
	        group.set_pose_target(self.pose_goal_1)

	        ## Now, we call the planner to compute the plan and visualize it if successful Note that we are just planning, not asking move_group to actually move the robot
	        #group.set_goal_position_tolerance(1.5)
	        self.plan_1 = group.plan()
	        rospy.sleep(0.5)
	        ## You can ask RVIZ to visualize a plan (aka trajectory) for you.  But the group.plan() method does this automatically so this is not that useful here (it just displays the same trajectory again).
	        self.display_trajectory_1 = moveit_msgs.msg.DisplayTrajectory()
	        self.display_trajectory_1.trajectory_start = robot.get_current_state()
	        self.display_trajectory_1.trajectory.append(self.plan_1)
	        display_trajectory_publisher.publish(self.display_trajectory_1);

	        rospy.sleep(0.5)
	        group.go(wait=True)

	        ## second movement - Go to the bucket and leave the item
	        ## ^^^^^^^^^^^^^^^^^^^^^
	        print "============ Generating plan 2"
	        self.pose_goal_2 = group.get_current_pose().pose
	        self.pose_goal_2.orientation = self.bucket_pos.pose.orientation
	        self.pose_goal_2.position = self.bucket_pos.pose.position
	        print 'Bucket -> pose goal:', self.pose_goal_2
	        group.set_pose_target(self.pose_goal_2)

	        self.plan_2 = group.plan()
	        rospy.sleep(0.5)
	        self.display_trajectory_2 = moveit_msgs.msg.DisplayTrajectory()
	        self.display_trajectory_2.trajectory_start = robot.get_current_state()
	        self.display_trajectory_2.trajectory.append(self.plan_2)
	        display_trajectory_publisher.publish(self.display_trajectory_2);

	        rospy.sleep(0.5)
	        group.go(wait=True)

	    ## When finished shut down moveit_commander.
	    moveit_commander.roscpp_shutdown()

	    R = rospy.Rate(10)
	    while not rospy.is_shutdown():
	        R.sleep()

	def callback(self, ms): # callback function for 'pose' messages
	    #print ms.name
	    #print "Message: " , ms.pose <- 9 elements[pos, orientation0, ... , orientation8]
	    index = 0
	    for str in ms.name:
	        if(str.find('cube')>=0):
	            self.cube_index.append(index)
	            #print 'Cube', index-2, 'pose:\n', ms.pose[index]
	            self.cube_pos.pose = ms.pose[index]
	            self.number_of_cubes+=1 # increase cube counter
	        if(str.find('bucket')>=0):
	            self.bucket_index = index
	            #print 'Bucket pose:\n', ms.pose[index]
	            self.bucket_pos.pose = ms.pose[index]
	        index+=1
	    #print cube_index
	    #print bucket_index

def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('find_stuff', anonymous=True)
    c = position_finder()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Position Finder module"
    cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
        pass