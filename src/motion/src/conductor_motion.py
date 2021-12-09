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
from motion.msg import music_commands

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
    planner = PathPlanner("both_arms")
    
    for i in range(len(right_arm_motion)):
        time = rospy.Duration.from_sec(tempo[i])

        joint_goal_right = np.loadtxt(file_location + right_arm_motion[i]+ ".txt" ).tolist()
        joint_goal_left  = np.loadtxt(file_location + left_arm_motion[i]+ ".txt").tolist()
        joint_goal = joint_goal_left+joint_goal_right
        try:
            #Plan
            plan = planner.plan_to_joint_goal(joint_goal, speed)

            #Execute
            if i == 0:
                raw_input("Press Enter to go! Will begin with a pre-beat motion...")
            planner.execute_plan(plan,timed = True)
            rospy.sleep(time)
            planner._group.stop()
            
        except Exception as e:
            print e
            traceback.print_exc()
    

def go_to_joint_position(arm,position):
    while not rospy.is_shutdown():
        try:
            planner = PathPlanner(arm)
            joint_goal = np.loadtxt(file_location + position + ".txt").tolist()
            planner._group.go(joint_goal,wait=True)

        except Exception as e:
            print e
            traceback.print_exc()
        else:
            break

# ============================================================================ #

def main(message):
    """
    Main Script
    """
    # right_arm_motion = ["right_beat_1","right_beat_2","right_beat_3","right_beat_4","right_beat_1",
    #                         "right_beat_2","right_beat_3","right_beat_4","right_beat_1",
    #                         "right_last_hold","right_last_swing","right_last_end"]
    # left_arm_motion  = ["left_neutral","left_neutral","left_neutral","left_neutral","left_beat_1",
    #                         "left_beat_2","left_beat_3","left_beat_4","left_beat_1",
    #                         "left_last_hold","left_last_swing","left_last_end"]
    # left_arm_motion  = ["left_crescendo_start","left_crescendo_1_4","left_crescendo_2_4",
    #                         "left_crescendo_3_4","left_crescendo_end",
    #                         "left_neutral","left_neutral","left_beat_4","left_beat_1",
    #                         "left_last_hold","left_last_swing","left_last_end"]


    # tempo = np.ones(12)
    # tempo[9:] = np.ones(3)*1.5


    #Initial Position
    go_to_joint_position("both_arms","neutral_both")
    #go_to_joint_position("left_arm","left_crescendo_start")


    #Read command from conductor_commands topic
    tempo = message.tempo
    right_arm_motions = message.right_arm_motions
    left_arm_motions = message.left_arm_motions

    #Execute commands
    # TODO: ideally, scale arm velocity based on tempo with a linear function
    
    timed_joint_path(right_arm_motions,left_arm_motions,1.8,tempo)

def listener():
    rospy.Subscriber("conductor_commands",music_commands,main)
    rospy.spin()

if __name__ == '__main__':
    joint_state_topic = ['joint_states:=/robot/joint_states']
    moveit_commander.roscpp_initialize(joint_state_topic)
    rospy.init_node('moveit_node',anonymous=False)
    roscpp_initialize(sys.argv)
    listener()
