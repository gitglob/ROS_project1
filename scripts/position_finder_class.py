#!/usr/bin/env python
import roslib
roslib.load_manifest('project1')
 
import sys
import copy
import rospy
import math
import moveit_commander
import geometry_msgs.msg
import tf_conversions
from moveit_msgs.msg import DisplayTrajectory
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import JointState

class position_finder():

	def __init__(self):
		# "global" parameters
		self.cube_index = []
		self.bucket_index = None
		self.number_of_elements = 0
		self.cube_pos = []
		self.bucket_pos = []

		# initialize robot, scene, group, Subscriber, Publishers
		self.initialize_stuff()

	def initialize_stuff(self): # method to do the environment setup
		## First initialize moveit_commander and rospy.
		print "============ Starting setup"
		self.robot = moveit_commander.RobotCommander()
		self.scene = moveit_commander.PlanningSceneInterface()
		self.group = moveit_commander.MoveGroupCommander("Arm")

		## Let's setup the planner
		self.group.set_planning_time(2)
		self.group.set_goal_orientation_tolerance(0.01)
		self.group.set_goal_joint_tolerance(0.01)
		self.group.set_num_planning_attempts(100)

		# ModelStates: [ground_plane, jaco_on_table, cube0, cube1, ... , bucket] [String, Pose, Twist]
		# DisplayTrajecory: too complex...
		# JointState: [header, name, position, velocity, effort]
		self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', DisplayTrajectory, queue_size=1) # initialize trajectory publisher
		self.joint_publisher = rospy.Publisher("/jaco/joint_control", JointState, queue_size=1) # initialize joint state publisher for gripper
		self.pose_subscriber = rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback) # initialize pose subscriber
		rospy.sleep(1.)


		print "====== Number of cubes: ", self.number_of_cubes
		for i in range(self.number_of_cubes):
			self.findStuff(i)

	def findStuff(self, i): # method to loop over every cube
		print "============ CUBE <- #", i

		# start at a neutral configuration
		print "### Neutral ###"
		start_config = self.bucket_pos[0]
		start_config.x = 0.4
		start_config.y = -0.1
		start_config.z = 1.35
		self.SlowlyReach(start_config)

		# open the gripper the first time
		if i==0: 
			self.openGripper()

		print "============ Generating plan 1, aka: \nGrab it by the cuby."
		print "Next cube position:\n", self.cube_pos[i]

		# go above the cube
		print "### Above Cube ###"
		above_cube_config = self.cube_pos[i]
		temp = above_cube_config.z
		above_cube_config.z = 1.35
		self.SlowlyReach(above_cube_config)

		# go down and reach the cube
		print "### Reach Cube ###"
		reach_cube_config = above_cube_config
		reach_cube_config.z = temp+0.15
		self.SlowlyReach(reach_cube_config)

		# close grip to grab cube
		self.closeGripper()

		# go above the cube
		print "### Pick Up Cube ###"
		self.SlowlyReach(above_cube_config)

		print "============ Generating plan 2: \nReturn to the bucket"
		print "Bucket position:\n", self.bucket_pos[0]

		# go above the bucket
		print "### Above bucket ###"
		above_bucket_config = self.bucket_pos[0]
		above_bucket_config.z = 1.35
		self.SlowlyReach(above_bucket_config)

		# open the gripper to drop the cube
		self.openGripper()

	def SlowlyReach(self, config):
		tol_list = [0.1, 0.05, 0.02, 0.01, 0.005]
		for tolerance in tol_list:
			print 'Tolerance = ', tolerance
			self.group.set_goal_tolerance(tolerance)
			self.moveArm(config)

	def moveArm(self, config):
		print "- Planning Move..." # go to the original position
		pose_goal = self.group.get_current_pose().pose

		pose_goal.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0., -math.pi/2, 0.))
		pose_goal.position.x = config.x
		pose_goal.position.y = config.y
		pose_goal.position.z = config.z
		self.group.set_pose_target(pose_goal)

		plan = self.group.plan()
		rospy.sleep(0.5)

		display_trajectory = DisplayTrajectory()
		display_trajectory.trajectory_start = self.robot.get_current_state()
		display_trajectory.trajectory.append(plan)
		self.display_trajectory_publisher.publish(display_trajectory);
		rospy.sleep(1)

		print "- Executing..."
		self.group.go(wait=True)
		rospy.sleep(2)

	def moveArmCartesian(self,config):
		waypoints = []
		pose_goal = self.group.get_current_pose().pose
		waypoints.append(pose_goal) # current pose
		pose_goal.position.x = config[0]
		pose_goal.position.y = config[1]
		pose_goal.position.z = config[2]
		waypoints.append(copy.deepcopy(pose_goal)) # goal pose
		print 'Pose goal:\n', pose_goal.position

		# create cartesian plan based on current and goal pose
		# compute_cartesian_path: Compute a sequence of waypoints that make the end-effector move in straight line segments that follow the poses specified as waypoints. 
		# Configurations are computed for every eef_step meters; 
		# The jump_threshold specifies the maximum distance in configuration space between consecutive points in the resulting path. 
		# The return value is a tuple: a fraction of how much of the path was followed, the actual RobotTrajectory. 
		(plan, fraction) = self.group.compute_cartesian_path(waypoints, 0.01, 0.0)
		rospy.sleep(0.5)

		display_trajectory = DisplayTrajectory()
		display_trajectory.trajectory_start = self.robot.get_current_state()
		display_trajectory.trajectory.append(plan)
		self.display_trajectory_publisher.publish(display_trajectory)
		rospy.sleep(2.)

		print "==== Executing plan"
		self.group.execute(plan,wait=True)
		rospy.sleep(4.)

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
		self.number_of_cubes = sum("cube" in s for s in ms.name)
		if len(ms.name) > self.number_of_elements: # check if new elements have been added and if so, find their position again
			index = 0
			for str in ms.name: # Now drop all the cubes to the bucket
				self.number_of_elements+=1 # counter for all the elements in the virtual environment
				if(str.find("cube")>=0):
					self.cube_pos.append(ms.pose[index].position)
				if(str.find("bucket")>=0):
					self.bucket_pos.append(ms.pose[index].position)
				index+=1

def main(args):
	'''Initializes and cleanup ros node'''
	rospy.init_node("stuff_finder", anonymous=True)
	try:
		moveit_commander.roscpp_initialize(sys.argv)
		c = position_finder()
		moveit_commander.roscpp_shutdown()

		R = rospy.Rate(10)
		while not rospy.is_shutdown():
			R.sleep()
	except rospy.ROSInterruptException:
		pass

if __name__ == '__main__':
	main(sys.argv)
	pass