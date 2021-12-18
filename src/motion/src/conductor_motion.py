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

file_location = "src/motion/positions/"

# ============================================================================ #

# Execute a timed joint path, consisting of a list of positions for the 
# two arms to hit at each beat.
# Tempo is a list of time intervals, in seconds. Each element of tempo is a number
# of seconds that says how many seconds are between this beat and the next beat.
def timed_joint_path(right_arm_motion, left_arm_motion, tempo):

    # Instantiate the path planner
    planner = PathPlanner("both_arms")

    # Instantiate two Limb objects
    right_arm_object = Limb("right")
    left_arm_object = Limb("left")

    # Iterate through each of the positions. Move both arms to the given
    # position, then repeat (go to the next position...)
    for i in range(len(right_arm_motion)):

        # Record the length of time between this position and the next
        time_to_move = rospy.Duration.from_sec(tempo[i])

        # Record the current positions of the joints.
        current_joint_pos_right = right_arm_object.joint_angles().values()
        current_joint_pos_left = left_arm_object.joint_angles().values()

        # Create the joint goals by specifying where
        # the left and right arms should go now
        next_joint_pos_right = np.loadtxt(file_location + right_arm_motion[i] + ".txt" ).tolist()
        next_joint_pos_left  = np.loadtxt(file_location + left_arm_motion[i] + ".txt").tolist()
        next_joint_pos = next_joint_pos_left + next_joint_pos_right

        angle_to_move_right = max([next_joint_pos_right[j] - current_joint_pos_right[j] for j in range(7)])
        angle_to_move_left = max([next_joint_pos_left[j] - current_joint_pos_left[j] for j in range(7)])
        angle_to_move = max(angle_to_move_right, angle_to_move_left)

        # The speed we want to move, in radians per second
        speed_angular = angle_to_move / tempo[i] # radians per second

        # I found experimentally that calling planner.plan_to_joint_goal() with speed = 1
        # causes the robot's joints to move at about 0.6 radians per second
        speed = (speed_angular / 1.0)

        # Start planning and executing the movement
        try:

            speed = min(speed, 3.0)

            # Use the path planner to plan a trajectory to this joint position
            plan = planner.plan_to_joint_goal(next_joint_pos, speed)

            # Use the path planner to execute the planned trajectory
            planner.execute_plan(plan, timed = True)

            # Wait until it is time for the next movement to begin
            rospy.sleep(time_to_move)
            planner._group.stop()
            
        # Catch an error and halt the program if needed
        except Exception as e:
            print e
            traceback.print_exc()

# ============================================================================ #
    
def go_to_joint_position(arm, position):
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

def debug():

    right_arm_motion = ["right_beat_1","right_beat_2","right_beat_3","right_beat_4","right_beat_1",
                            "right_beat_2","right_beat_3","right_beat_4","right_beat_1",
                            "right_last_hold","right_last_swing","right_last_end"]
    left_arm_motion  = ["left_neutral","left_neutral","left_neutral","left_neutral","left_beat_1",
                            "left_beat_2","left_beat_3","left_beat_4","left_beat_1",
                            "left_last_hold","left_last_swing","left_last_end"]

    tempo = np.ones(12)
    tempo[9:] = np.ones(3) * 1.5

    go_to_joint_position("right_arm","right_beat_1")
    go_to_joint_position("left_arm","left_beat_1")
    raw_input("Press Enter")

    go_to_joint_position("right_arm","right_test")
    go_to_joint_position("left_arm","left_test")
    raw_input("Press Enter")

    go_to_joint_position("right_arm","right_beat_2")
    go_to_joint_position("left_arm","left_beat_2")
    raw_input("Press Enter")

    go_to_joint_position("right_arm","right_beat_3")
    go_to_joint_position("left_arm","left_beat_3")
    raw_input("Press Enter")

    go_to_joint_position("right_arm","right_beat_4")
    go_to_joint_position("left_arm","left_beat_4")
    raw_input("Press Enter")

    go_to_joint_position("right_arm","right_last_hold")
    go_to_joint_position("left_arm","left_last_hold")
    raw_input("Press Enter")

    go_to_joint_position("right_arm","right_last_swing")
    go_to_joint_position("left_arm","left_last_swing")
    raw_input("Press Enter")

    go_to_joint_position("right_arm","right_last_end")
    go_to_joint_position("left_arm","left_last_end")
    raw_input("Press Enter")

# ============================================================================ #

def main(message):

    tempo = message.tempo
    right_arm_motions = message.right_arm_motions
    left_arm_motions = message.left_arm_motions

    raw_input("Press Enter to go!")

    go_to_joint_position("both_arms", "neutral_both")
    
    debug()

    # timed_joint_path(right_arm_motions,left_arm_motions, tempo)

def listener():
    rospy.Subscriber("conductor_commands", music_commands,main)
    rospy.spin()

if __name__ == '__main__':
    joint_state_topic = ['joint_states:=/robot/joint_states']
    moveit_commander.roscpp_initialize(joint_state_topic)
    rospy.init_node('moveit_node',anonymous=False)
    roscpp_initialize(sys.argv)
    listener()

# ============================================================================ #
