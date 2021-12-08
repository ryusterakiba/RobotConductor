#!/usr/bin/env python
"""
Execute complex conducting path sequence
Dean Zhang
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

'''
ASSUMPTIONS FOR NOW: 
- EVERYTHING IS IN 4/4 TIME
- THE LAST NOTE WILL BE A HALF NOTE
'''


# ============================================================================ #

# Feel free to change name of file based on what song you're conducting!!!
file_location = '/home/cc/ee106a/fl21/class/ee106a-abw/ros_workspaces/robot_conductor_ryu/src/sensing/src/output/'
music = 'hcb_theoretical_wo_time.txt'
# ============================================================================ #

def calc_num_measures(line):
	total_beats_in_line = 0
	split_by_space = line.split(" ")
	for note in split_by_space:
		split_by_backslash = note.split("/")
		denominator = float(split_by_backslash[1].split("]")[0])
		total_beats_in_line += 1/denominator
	return total_beats_in_line

def duration_of_last_note(line):
	split_by_space = line.split(" ")
	last_note = 0
	for note in split_by_space:
		split_by_backslash = note.split("/")
		last_note = float(split_by_backslash[1].split("]")[0])
	return last_note

def talker():
	pub = rospy.Publisher('conductor_commands', music_commands, queue_size=10) #only 1 message is gonna be realistically sent
	r = rospy.Rate(10)

	while not rospy.is_shutdown():
		message = raw_input("Enter song file: ")
		music_file = file_location + music
		
		## Variable bookkeeping
		total_num_measures = 0
		## 

		with open(music_file) as f:
			for line in f:
				total_num_measures += calc_num_measures(line)
				last_note_duration = duration_of_last_note(line)

	    right_arm_motion = ["right_beat_1","right_beat_2","right_beat_3","right_beat_4"]
	    left_arm_motion  = ["left_beat_1","left_beat_2","left_beat_3","left_beat_4"]

	    # SPECIFIC TO LAST NOTE = HALF NOTE
	    right_arm_last_note_half = ["right_beat_1","right_beat_2","right_last_hold","right_last_swing","right_last_end"]
	    left_arm_last_note_half = ["left_beat_1","left_beat_2","left_last_hold","left_last_swing","left_last_end"]


		robot_commands = music_commands()
		# Need to separate functionality later on (if more time). For now, both arms mirror each other up until the last measure
		robot_commands.right_arm_motions = right_arm_motions * (total_num_measures - 1)
		robot_commands.left_arm_motions = left_arm_motions * (total_num_measures - 1)

		# Last measure
		if (last_note_duration == 2):
			robot_commands.right_arm_motions.extend(right_arm_last_note_half)
			robot_commands.left_arm_motions.extend(left_arm_last_note_half)

		# Ending sequence
		tempo = np.ones(len(robot_commands.right_arm_motions))

		if (last_note_duration == 2):
			#Cutoff actions, slower speed
			tempo[-3:] = np.ones(3)*1.5
		robot_commands.tempo = tempo

		pub.publish(robot_commands)
		print("Ready to run conductor_motion.py!")
		r.sleep()

if __name__ == '__main__':
	#command = music_commands()
	rospy.init_node('talker', anonymous=True)
	try:
		talker()
	except rospy.ROSInterruptException: pass