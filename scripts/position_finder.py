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

# global variables
cube_index = []
#bucket_index = None
#number_of_cubes = 0
cube_pos = geometry_msgs.msg.PoseStamped()
bucket_pos = geometry_msgs.msg.PoseStamped()

def get_positions(ms):
    #print ms.name
    #print "Message: " , ms.pose <- 9 elements[pos, orientation0, ... , orientation8]
    index = 0
    for str in ms.name:
        if(str.find('cube')>=0):
            cube_index.append(index)
            #print 'Cube', index-2, 'pose:\n', ms.pose[index]
            cube_pos.pose = ms.pose[index]
            #number_of_cubes+=1 # increase cube counter
        if(str.find('bucket')>=0):
            bucket_index = index
            #print 'Bucket pose:\n', ms.pose[index]
            bucket_pos.pose = ms.pose[index]
        index+=1

    #print cube_index
    #print bucket_index

def find_stuff():
    # Always the first thing: Initialize node
    rospy.init_node('position_finder_node', anonymous=True)

    ## First initialize moveit_commander and rospy.
    print "============ Starting tutorial setup"
    moveit_commander.roscpp_initialize(sys.argv)
    ## Instantiate a RobotCommander object.  This object is an interface to the robot as a whole.
    robot = moveit_commander.RobotCommander()
    ## Instantiate a PlanningSceneInterface object.  This object is an interface to the world surrounding the robot.
    scene = moveit_commander.PlanningSceneInterface()
    ## Instantiate a MoveGroupCommander object.  This object is an interface to one group of joints.  In this case the group is the joints in the left
    ## arm.  This interface can be used to plan and execute motions on the left arm.
    group = moveit_commander.MoveGroupCommander("Arm")
    ## We create this DisplayTrajectory publisher which is used below to publish trajectories for RVIZ to visualize.
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)

    ## Wait for RVIZ to initialize. This sleep is ONLY to allow Rviz to come up.
    print "============ Waiting for RVIZ..."
    rospy.sleep(2)

    ## Getting Basic Information
    ## We can get the name of the reference frame for this robot
    print "============ Reference frame: %s" % group.get_planning_frame()
    ## We can also print the name of the end-effector link for this group
    print "============ Reference frame: %s" % group.get_end_effector_link()
    ## We can get a list of all the groups in the robot
    print "============ Robot Groups:"
    print robot.get_group_names()

    ## Sometimes for debugging it is useful to print the entire state of the robot.
    print "============ Printing robot state"
    print robot.get_current_state()
    print "============"

    scene = moveit_commander.PlanningSceneInterface()
    robot = moveit_commander.RobotCommander()

    rospy.sleep(2)

    # topic msgs: [ground_plane, jaco_on_table, cube0, cube1, ... , bucket] [String, Pose, Twist] <= ModelStates
    rospy.Subscriber("/gazebo/model_states", ModelStates, get_positions)

    for i in range(0,2):

        ## Planning to a Pose goal - Go grab a cube
        ## We can plan a motion for this group to a desired pose for the end-effector
        print "============ Generating plan 1"

        #group.set_joint_value_target([1.57,0.,0.,0.])

        ## Let's setup the planner
        group.set_planning_time(1.0)
        group.set_goal_orientation_tolerance(1)
        group.set_goal_tolerance(1)
        group.set_goal_joint_tolerance(0.1)
        group.set_num_planning_attempts(10)

        pose_goal_1 = group.get_current_pose().pose
        print '#', i, 'cube pose:', cube_pos.pose
        #pose_goal_1.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0., -math.pi/2, 0.)) # cube orientation
        pose_goal_1.orientation = cube_pos.pose.orientation
        pose_goal_1.position = cube_pos.pose.position
        #pose_goal_1.position.x = cube_pos.pose.position.x # cube global x position
        #pose_goal_1.position.y = cube_pos.pose.position.y # cube global y position
        #pose_goal_1.position.z = cube_pos.pose.position.z # cube global z position
        print 'Cube pose goal:', pose_goal_1
        group.set_pose_target(pose_goal_1)

        ## Now, we call the planner to compute the plan and visualize it if successful Note that we are just planning, not asking move_group to actually move the robot
        #group.set_goal_position_tolerance(1.5)
        plan_1 = group.plan()

        print "============ Waiting while RVIZ displays plan 1..."
        rospy.sleep(0.5)
        ## You can ask RVIZ to visualize a plan (aka trajectory) for you.  But the group.plan() method does this automatically so this is not that useful here (it just displays the same trajectory again).
        print "============ Visualizing plan 1"
        display_trajectory_1 = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory_1.trajectory_start = robot.get_current_state()
        display_trajectory_1.trajectory.append(plan_1)
        display_trajectory_publisher.publish(display_trajectory_1);

        print "============ Waiting while plan 1 is visualized (again)..."
        rospy.sleep(0.5)
        group.go(wait=True)

        ## second movement - Go to the bucket and leave the item
        ## ^^^^^^^^^^^^^^^^^^^^^
        print "============ Generating plan 2"
        pose_goal_2 = group.get_current_pose().pose
        #pose_goal_2.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0., -math.pi/2, 0.)) # cube orientation
        pose_goal_2.orientation = bucket_pos.pose.orientation
        pose_goal_2.position.x = bucket_pos.pose.position.x # cube global x position
        pose_goal_2.position.y = bucket_pos.pose.position.y # cube global y position
        pose_goal_2.position.z = bucket_pos.pose.position.z # cube global z position
        #print 'Bucket pose goal:', pose_goal_2
        group.set_pose_target(pose_goal_2)

        plan_2 = group.plan()
        rospy.sleep(0.5)
        print "============ Visualizing plan 2"
        display_trajectory_2 = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory_2.trajectory_start = robot.get_current_state()
        display_trajectory_2.trajectory.append(plan_2)
        display_trajectory_publisher.publish(display_trajectory_2);
        print "============ Waiting while plan 2 is visualized (again)..."
        rospy.sleep(0.5)
        group.go(wait=True)

    ## When finished shut down moveit_commander.
    moveit_commander.roscpp_shutdown()

    R = rospy.Rate(10)
    while not rospy.is_shutdown():
        R.sleep()
 

if __name__=='__main__':
    try:
        print "Main."
        find_stuff()
    except rospy.ROSInterruptException:
        pass