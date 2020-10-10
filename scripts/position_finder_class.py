#!/usr/bin/env python
import roslib
roslib.load_manifest('project1')
 
import sys
import copy
import rospy
import math
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf_conversions
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import JointState

class position_finder():

	def __init__(self):
		# "global" parameters
		self.cube_index = []
		self.bucket_index = None
		self.number_of_cubes = 0
		self.number_of_elements = 0
		self.cube_pos = geometry_msgs.msg.PoseStamped()
		self.bucket_pos = geometry_msgs.msg.PoseStamped()

		# initialize robot, scene, group, Subscriber, Publishers
		self.initialize_stuff()

	def initialize_stuff(self): # method to do the environment setup
		## First initialize moveit_commander and rospy.
		print "============ Starting tutorial setup"
		moveit_commander.roscpp_initialize(sys.argv)

		self.robot = moveit_commander.RobotCommander()
		self.scene = moveit_commander.PlanningSceneInterface()
		self.group = moveit_commander.MoveGroupCommander("Arm")

		print "============ Waiting for RVIZ..."

		print "============ Reference frame: %s" % self.group.get_planning_frame()
		print "============ Reference frame: %s" % self.group.get_end_effector_link()
		print "============ Robot Groups:"
		print self.robot.get_group_names()

		print "============ Printing robot state"
		print self.robot.get_current_state()
		print "============"

		# topic msgs: [ground_plane, jaco_on_table, cube0, cube1, ... , bucket] [String, Pose, Twist] <= ModelStates
		self.pose_subscriber = rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback) # initialize pose subscriber
		self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory) # initialize trajectory publisher
		self.joint_publisher = rospy.Publisher("/jaco/joint_control", JointState, queue_size=1) # initialize joint state publisher for gripper

	def findStuff(self): # method to loop over every cube
		## Let's setup the planner
		#group.set_planning_time(1.0)
		self.group.set_goal_orientation_tolerance(0.01)
		self.group.set_goal_tolerance(0.001)
		self.group.set_goal_joint_tolerance(0.01)
		self.group.set_num_planning_attempts(100)

		# reset the arm now
		self.resetArm() # first of all, reset the arm
		if self.number_of_cubes==0: # the first time we need to open the gripper from the start
			self.openGripper()

		print "============ Generating plan 1, aka: \nGrab it by the cuby."
		pose_goal_1 = self.group.get_current_pose().pose
		
		waypoints_1 = []
		waypoints_1.append(pose_goal_1) # current pose
		pose_goal_1.position = self.cube_pos.pose.position
		print 'Cube -> pose goal:\n', pose_goal_1.position
		waypoints_1.append(pose_goal_1) # goal pose

		# create cartesian plan based on current and goal pose
		(plan_1, fraction_1) = self.group.compute_cartesian_path(waypoints_1, 0.01, 0.0)
		## Now, we call the planner to compute the plan and visualize it if successful Note that we are just planning, not asking move_group to actually move the robot
		rospy.sleep(2)

		## You can ask RVIZ to visualize a plan (aka trajectory) for you.  But the group.plan() method does this automatically so this is not that useful here (it just displays the same trajectory again).
		display_trajectory_1 = moveit_msgs.msg.DisplayTrajectory()
		display_trajectory_1.trajectory_start = self.robot.get_current_state()
		display_trajectory_1.trajectory.append(plan_1)
		self.display_trajectory_publisher.publish(display_trajectory_1)
		rospy.sleep(0.5)

		print "==== Executing plan 1."
		self.group.execute(plan_1,wait=True)
		rospy.sleep(5)

		# reset the arm again
		self.closeGripper()
		self.resetArm()

		print "============ Generating plan 2: \nReturn to the bucket" # go to the bucket
		pose_goal_2 = self.group.get_current_pose().pose

		waypoints_2 = []
		waypoints_2.append(pose_goal_2) # current pose
		pose_goal_2.position.x = self.bucket_pos.pose.position.x
		pose_goal_2.position.y = self.bucket_pos.pose.position.y
		pose_goal_2.position.z = 1.35
		print 'Bucket -> pose goal:\n', pose_goal_2.position
		waypoints_2.append(copy.deepcopy(pose_goal_2)) # goal pose

		# create cartesian plan based on current and goal pose
		(plan_2, fraction_2) = self.group.compute_cartesian_path(waypoints_2, 0.01, 0.0)
		rospy.sleep(2)

		display_trajectory_2 = moveit_msgs.msg.DisplayTrajectory()
		display_trajectory_2.trajectory_start = self.robot.get_current_state()
		display_trajectory_2.trajectory.append(plan_2)
		self.display_trajectory_publisher.publish(display_trajectory_2)
		rospy.sleep(0.5)

		print "==== Executing plan 2."
		self.group.execute(plan_2,wait=True)
		rospy.sleep(5)

		# open the gripper to drop the cube
		self.openGripper()

	def resetArm(self):
		print "============ Planning Reset" # go to the original position
		pose_goal = self.group.get_current_pose().pose

		pose_goal.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0., -math.pi/2, 0.))
		pose_goal.position.x =0.40
		pose_goal.position.y =-0.10
		pose_goal.position.z =1.35
		self.group.set_pose_target(pose_goal)

		plan = self.group.plan()
		rospy.sleep(2)

		display_trajectory = moveit_msgs.msg.DisplayTrajectory()
		display_trajectory.trajectory_start = self.robot.get_current_state()
		display_trajectory.trajectory.append(plan)
		self.display_trajectory_publisher.publish(display_trajectory);
		rospy.sleep(0.5)

		self.group.go(wait=True)
		rospy.sleep(5)

	def openGripper(self):
		print "============ Opening Grip"
		currentJointState = JointState()

		currentJointState = rospy.wait_for_message("/joint_states",JointState)
		currentJointState.header.stamp = rospy.get_rostime()
		tmp = 0.005

		currentJointState.position = tuple(list(currentJointState.position[:6]) + [tmp] + [tmp]+ [tmp])
		rate = rospy.Rate(10) # 10hz
		for i in range(3):
			self.joint_publisher.publish(currentJointState)
			rate.sleep()

	def closeGripper(self):
		print "============ Closing Grip"
		currentJointState = JointState()

		currentJointState = rospy.wait_for_message("/joint_states",JointState)
		currentJointState.header.stamp = rospy.get_rostime()
		tmp = 0.7

		currentJointState.position = tuple(list(currentJointState.position[:6]) + [tmp] + [tmp]+ [tmp])
		rate = rospy.Rate(10) # 10hz
		for i in range(3):
			self.joint_publisher.publish(currentJointState)
			rate.sleep()

	def callback(self, ms): # callback function for 'pose' messages
		#print "Message: " , ms.pose <- 9 elements[pos, orientation0, ... , orientation8]
		if len(ms.name) > self.number_of_elements: # check if new elements have been added and if so, find their position again
			index = 0
			for str in ms.name: # First get the position of the bucket
				if(str.find('bucket')>=0):
					self.bucket_index = index
					self.bucket_pos.pose = ms.pose[index]
				index+=1
			index = 0
			for str in ms.name: # Now drop all the cubes to the bucket
				self.number_of_elements+=1 # counter for all the elements in the virtual environment
				if(str.find('cube')>=0):
					self.cube_index.append(index) # get the index of every cube
					self.cube_pos.pose = ms.pose[index] # get the position of every cube in a row
					# now call the function to actually grab the specific cube
					print "============ CUBE <- #", self.number_of_cubes
					self.findStuff()
					self.number_of_cubes+=1 # increase cube counter
				index+=1
			print "============", self.number_of_cubes, "cubes so far...", "============" 

def main(args):
	'''Initializes and cleanup ros node'''
	rospy.init_node('stuff_finder', anonymous=True)
	try:
		c = position_finder()
		## When finished shut down moveit_commander.
		moveit_commander.roscpp_shutdown()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass

if __name__ == '__main__':
	main(sys.argv)
	pass