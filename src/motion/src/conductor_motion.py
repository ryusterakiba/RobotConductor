#!/usr/bin/env python
"""
Execute complex conducting path sequence
Ryu Akiba, Cooper Collier
"""

from baxter_interface import Limb

import rospy
import numpy as np
import sys
import copy
import traceback

from moveit_msgs.msg import OrientationConstraint, RobotTrajectory
from geometry_msgs.msg import PoseStamped, Pose
from moveit_commander import roscpp_initialize
import moveit_commander

from conductor_planner import PathPlanner

# ============================================================================ #

file_location = '/home/cc/ee106a/fl21/class/ee106a-abw/ros_workspaces/robot_conductor_ryu/src/motion/positions/'

# ============================================================================ #
'''
Execution

A way without using waypoints.
Use joint positions
'''
def timed_joint_path(right_arm_motion,left_arm_motion,speed,tempo):
    while not rospy.is_shutdown():
        planner = PathPlanner("both_arms")
        time = 60/tempo
        for i in range(len(right_arm_motion)):
            joint_goal_right = np.loadtxt(file_location + right_arm_motion[i]+ ".txt" ).tolist()
            joint_goal_left  = np.loadtxt(file_location + left_arm_motion[i]+ ".txt").tolist()
            joint_goal = joint_goal_left+joint_goal_right
            try:
                #Plan
                plan = planner.plan_to_joint_goal(joint_goal, speed)

                #Execute
                #raw_input("Press Enter to go")
                planner.execute_plan(plan,timed = True)
                rospy.sleep(time)
                planner._group.stop()
                
            except Exception as e:
                print e
                traceback.print_exc()

def go_to_joint_position(position):
    while not rospy.is_shutdown():
        try:
            planner = PathPlanner("both_arms")
            joint_goal = np.loadtxt(file_location + position + ".txt").tolist()
            planner._group.go(joint_goal,wait=True)

        except Exception as e:
            print e
            traceback.print_exc()
        else:
            break

# ============================================================================ #

def main():
    """
    Main Script
    """
    right_arm_motion = ["right_beat_1","right_beat_2","right_beat_3","right_beat_4","right_beat_1"]
    left_arm_motion  = ["left_neutral","left_neutral","left_crescendo_bot","left_crescendo_top","left_neutral"]

    #Initial Position
    go_to_joint_position("neutral_both")

    #Start
    raw_input("Press Enter to go")
    timed_joint_path(right_arm_motion,left_arm_motion,1,60)


if __name__ == '__main__':
    joint_state_topic = ['joint_states:=/robot/joint_states']
    moveit_commander.roscpp_initialize(joint_state_topic)
    rospy.init_node('moveit_node',anonymous=False)
    roscpp_initialize(sys.argv)
    main()
