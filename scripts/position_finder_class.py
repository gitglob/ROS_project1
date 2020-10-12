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
		self.bucket_index = None
		self.cube_pos = [None]*6
		self.bucket_pos = None
		self.number_of_cubes = None
		self.i = None # cube counter
		self.reachable = None # variable to check if each cube is reachable

	def initializeStuff(self): # method to do the environment setup
		## First initialize moveit_commander and rospy.
		print "============ Starting setup"
		self.robot = moveit_commander.RobotCommander()
		self.scene = moveit_commander.PlanningSceneInterface()
		self.group = moveit_commander.MoveGroupCommander("Arm")

		## Let's setup the planner
		#self.group.set_planning_time(1)
		self.group.set_num_planning_attempts(100)

		# ModelStates: [ground_plane, jaco_on_table, cube0, cube1, ... , bucket] [String, Pose, Twist]
		# DisplayTrajecory: too complex...
		# JointState: [header, name, position, velocity, effort]
		self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', DisplayTrajectory, queue_size=1) # initialize trajectory publisher
		self.joint_publisher = rospy.Publisher("/jaco/joint_control", JointState, queue_size=1) # initialize joint state publisher for gripper
		self.pose_subscriber = rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback) # initialize pose subscriber
		rospy.sleep(1.)

		self.i = 0
		while self.i < self.number_of_cubes:
			self.findStuff()
			self.i+=1
			if ((self.i) == self.number_of_cubes):
				self.i = 0

	def findStuff(self): # method to loop over every cube
		print "====== Number of cubes: ", self.number_of_cubes
		print "============ CUBE <- #", self.i

		# check if cube is already inside bucket
		if self.cube_pos[self.i].z >= 0.75 and self.cube_pos[self.i].z <= 0.77:
			# start at a neutral configuration
			print "### Neutral ###"
			start_config = copy.deepcopy(self.bucket_pos) # for some weird reason, bucket_config doesn't take the value, but BECOMES self.bucket_pos
			start_config.x = 0.21 # 0.578
			start_config.y = 0.23 # -0.015
			start_config.z = 1.26 # 1.295
			min_precision = 1 # precision ranges from 0~4 : 4 is the highest
			max_precision = 1
			self.SlowlyReach(start_config, min_precision, max_precision)

			print "============ Generating plan 1, aka: \nGrab it by the cuby"
			print "Next cube position:\n", self.cube_pos[self.i]
			cube_config = copy.deepcopy(self.cube_pos[self.i])

			# go above the cube
			print "### Above Cube ###"
			cube_z = copy.deepcopy(cube_config.z)
			cube_config.z = cube_z + 0.5
			min_precision = 2
			max_precision = 2
			self.SlowlyReach(cube_config, min_precision, max_precision)

			# open the gripper
			self.openGripper()

			# go down and up
			critical = 0.165
			index = 1
			for dz in [0.4, 0.2, 0.19, 0.18, critical, 0.181, 0.21, 0.41, 0.51]:
				print "$$ Move: ", index, "/9", "(4 + 1 + 4)"
				min_precision = 4
				max_precision = 4
				cube_config.z = cube_z + dz
				self.SlowlyReach(cube_config, min_precision, max_precision)
				index +=1
				if (dz == 0.18) and (not self.reachable):
					print "** Unreachable Cube **"
					break
				if (dz == critical):
					self.closeGripper()

			# if the cube was reachable, return it to the bucket and do i middle stop to the neutral configuration
			if self.reachable:
				print "============ Generating plan 2: \nReturn to the bucket"
				print "Bucket position:\n", self.bucket_pos

				# go above the bucket
				print "### Above bucket ###"
				bucket_config = copy.deepcopy(self.bucket_pos)
				bucket_config.z = bucket_config.z + 0.5
				min_precision = 4
				max_precision = 4
				self.SlowlyReach(bucket_config, min_precision, max_precision)

				# open the gripper to drop the cube
				self.openGripper()
		
		#print "$$$ Cube", self.i, "is unreachable $$$"
		elif self.cube_pos[self.i].z >= 0.81:
			print "$$$ Cube", self.i, "is already inside the Bucket $$$"
		elif self.cube_pos[self.i].z <= 0.75:
			print "$$$ Cube", self.i, "has fallen $$$"

	def SlowlyReach(self, config, min_precision, max_precision):
		tol_list = [0.1, 0.07, 0.04, 0.02, 0.01]
		for precision in range (min_precision, max_precision+1):
			tolerance = tol_list[precision]
			print 'Tolerance = ', tolerance
			self.group.set_goal_tolerance(tolerance)
			self.moveArmCartesian(config)

	def moveArmCartesian(self,config):
		step = 0.01
		attempts = 1

		waypoints = []
		pose_goal = self.group.get_current_pose().pose
		waypoints.append(pose_goal) # current pose
		pose_goal.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0., -math.pi/2, 0.))
		pose_goal.position.x = config.x
		pose_goal.position.y = config.y
		pose_goal.position.z = config.z
		waypoints.append(copy.deepcopy(pose_goal)) # goal pose

		print "- Planning Move..."
		# create cartesian plan based on current and goal pose
		# compute_cartesian_path: Compute a sequence of waypoints that make the end-effector move in straight line segments that follow the poses specified as waypoints. 
		# Configurations are computed for every eef_step meters; 
		# The jump_threshold specifies the maximum distance in configuration space between consecutive points in the resulting path. 
		# The return value is a tuple: a fraction of how much of the path was followed, the actual RobotTrajectory. 
		# avoid_collision = True
		(plan, fraction) = self.group.compute_cartesian_path(waypoints, step, 0.0)
		rospy.sleep(0.5)
		print "%% success:", fraction
		while (fraction<0.8) and (attempts<=4): # force a somewhat successful plan by adding more intermediary points
			attempts+=1
			step = step/2
			print "%% success:", fraction
			(plan, fraction) = self.group.compute_cartesian_path(waypoints, step, 0.0)
			rospy.sleep(0.5)

		# check if the move was successful
		if fraction > 0.8:
			print "Success.-"
			display_trajectory = DisplayTrajectory()
			display_trajectory.trajectory_start = self.robot.get_current_state()
			display_trajectory.trajectory.append(plan)
			self.display_trajectory_publisher.publish(display_trajectory)
			rospy.sleep(1)

			print "- Executing..."
			self.group.execute(plan,wait=True)
			rospy.sleep(1)
			self.reachable = True
		else:
			print "Fail.-"
			self.reachable = False

	def moveArm(self, config):
		print "- Planning Move..."
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
		rospy.sleep(1)

	def openGripper(self):
		print "~~~~~ Opening Grip"
		currentJointState = JointState()

		currentJointState = rospy.wait_for_message("/joint_states",JointState)
		currentJointState.header.stamp = rospy.get_rostime()
		tmp = 0.005

		currentJointState.position = tuple(list(currentJointState.position[:6]) + [tmp] + [tmp]+ [tmp]) # change the last 3 joints to 0.005
		rate = rospy.Rate(10) # 10hz
		for i in range(3):
			self.joint_publisher.publish(currentJointState)
			rate.sleep()

	def closeGripper(self):
		print "~~~~~ Closing Grip"
		currentJointState = JointState()

		currentJointState = rospy.wait_for_message("/joint_states",JointState)
		currentJointState.header.stamp = rospy.get_rostime()
		tmp = 0.85

		currentJointState.position = tuple(list(currentJointState.position[:6]) + [tmp] + [tmp]+ [tmp]) # change the last 3 joints to 0.7
		rate = rospy.Rate(10) # 10hz
		for i in range(3):
			self.joint_publisher.publish(currentJointState)
			rate.sleep()

	def callback(self, ms): # callback function for 'pose' messages
		self.number_of_cubes = sum("cube" in s for s in ms.name)
		index = 0
		cube_index = 0
		for str in ms.name: # Now drop all the cubes to the bucket
			if(str.find("cube")>=0):
				self.cube_pos[cube_index] = ms.pose[index].position
				cube_index+=1
			if(str.find("bucket")>=0):
				self.bucket_pos = ms.pose[index].position
			index+=1

def main(args):
	'''Initializes and cleanup ros node'''
	rospy.init_node("stuff_finder", anonymous=True)
	try:
		moveit_commander.roscpp_initialize(sys.argv)
		c = position_finder()
		c.initializeStuff()
		moveit_commander.roscpp_shutdown()

		R = rospy.Rate(10)
		while not rospy.is_shutdown():
			R.sleep()
	except rospy.ROSInterruptException:
		pass

if __name__ == '__main__':
	main(sys.argv)
	pass