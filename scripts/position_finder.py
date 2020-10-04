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
bucket_index = -1
cube_pos = geometry_msgs.msg.PoseStamped()

def callback(ms):
    #print ms.name
    #print ms.pose

    index = 0
    for str in ms.name:
        if(str.find('cube')>=0):
            cube_index.append(index)
            cube_pos.pose = ms.pose[index]
        if(str.find('bucket')>=0):
            bucket_index = index
        index+=1

    #print cube_index
    #print bucket_index



def find_stuff():
    rospy.init_node('position_finder_node')

    ## BEGIN_TUTORIAL
    ## First initialize moveit_commander and rospy.
    print "============ Starting tutorial setup"
    moveit_commander.roscpp_initialize(sys.argv)

    ## Instantiate a RobotCommander object.  This object is an interface to
    ## the robot as a whole.
    robot = moveit_commander.RobotCommander()

    ## Instantiate a PlanningSceneInterface object.  This object is an interface
    ## to the world surrounding the robot.
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a MoveGroupCommander object.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the left
    ## arm.  This interface can be used to plan and execute motions on the left
    ## arm.
    group = moveit_commander.MoveGroupCommander("Arm")


    ## We create this DisplayTrajectory publisher which is used below to publish
    ## trajectories for RVIZ to visualize.
    display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory)

    ## Wait for RVIZ to initialize. This sleep is ONLY to allow Rviz to come up.
    print "============ Waiting for RVIZ..."
    rospy.sleep(2)
    print "============ Starting tutorial "

    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    ##
    ## We can get the name of the reference frame for this robot
    print "============ Reference frame: %s" % group.get_planning_frame()

    ## We can also print the name of the end-effector link for this group
    print "============ Reference frame: %s" % group.get_end_effector_link()

    ## We can get a list of all the groups in the robot
    print "============ Robot Groups:"
    print robot.get_group_names()

    ## Sometimes for debugging it is useful to print the entire state of the
    ## robot.
    print "============ Printing robot state"
    print robot.get_current_state()
    print "============"

    scene = moveit_commander.PlanningSceneInterface()
    robot = moveit_commander.RobotCommander()

    rospy.sleep(2)
    '''
    scene.remove_world_object('groundplane')
    scene.remove_world_object('table')
    '''
    # topic msgs: [ground_plane, hello_ros_roboxt, fire_hydrant_0, fire_hydrant] [String, Pose, Twist] <= ModelStates
    rospy.Subscriber("/gazebo/model_states", ModelStates, callback)

    ## Planning to a Pose goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector
    print "============ Generating plan 1"

    #group.set_joint_value_target([1.57,0.,0.,0.])


    ## Let's setup the planner
    group.set_planning_time(2.0)
    group.set_goal_orientation_tolerance(1)
    group.set_goal_tolerance(1)
    group.set_goal_joint_tolerance(0.1)
    group.set_num_planning_attempts(100)


    pose_goal = group.get_current_pose().pose
    pose_goal.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0., -math.pi/2, 0.))
    pose_goal.position.x = cube_pos.pose.position.x
    pose_goal.position.y = cube_pos.pose.position.y
    pose_goal.position.z = cube_pos.pose.position.z
    print pose_goal
    group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan
    ## and visualize it if successful
    ## Note that we are just planning, not asking move_group
    ## to actually move the robot
    #group.set_goal_position_tolerance(1.5)
    plan1 = group.plan()

    print "============ Waiting while RVIZ displays plan1..."
    rospy.sleep(0.5)


    ## You can ask RVIZ to visualize a plan (aka trajectory) for you.  But the
    ## group.plan() method does this automatically so this is not that useful
    ## here (it just displays the same trajectory again).
    print "============ Visualizing plan1"
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()

    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan1)
    display_trajectory_publisher.publish(display_trajectory);

    print "============ Waiting while plan1 is visualized (again)..."
    rospy.sleep(0.5)

    group.go(wait=True)

    ## When finished shut down moveit_commander.
    moveit_commander.roscpp_shutdown()

    ## END_TUTORIAL

    R = rospy.Rate(10)
    while not rospy.is_shutdown():
        R.sleep()
 

if __name__=='__main__':
  try:
    print "Main."
    find_stuff()
  except rospy.ROSInterruptException:
    pass