#!/usr/bin/env python
"""
Execute complex conducting path sequence
Dean Zhang, Ryu Akiba
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
right_arm_motion = ["right_beat_1","right_beat_2","right_beat_3","right_beat_4"]
left_arm_motion  = ["left_beat_1","left_beat_2","left_beat_3","left_beat_4"]

# SPECIFIC TO LAST NOTE = HALF NOTE
right_arm_last_note_half = ["right_beat_1","right_beat_2","right_last_hold","right_last_swing","right_last_end"]
left_arm_last_note_half = ["left_beat_1","left_beat_2","left_last_hold","left_last_swing","left_last_end"]

# SPECIFIC TO LAST NOTE = whole NOTE
right_arm_last_note_whole = ["right_beat_1","right_last_hold","right_last_swing","right_last_end"]
left_arm_last_note_whole = ["left_beat_1","left_last_hold","left_last_swing","left_last_end"]

#Crescendo over 4 beats
left_arm_crescendo_rest = ["left_neutral"]*4
left_arm_crescendo = ["left_crescendo_start","left_crescendo_1_4",
					"left_crescendo_2_4","left_crescendo_3_4"]

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
	return 4/last_note

def talker():
	pub = rospy.Publisher('conductor_commands', music_commands, queue_size=10) #only 1 message is gonna be realistically sent
	r = rospy.Rate(10)

	while not rospy.is_shutdown():
		message = raw_input("Enter song file (either hot_cross_buns or canon_in_d : ")
		

		if message == "hot_cross_buns":
			music = 'hcb_theoretical_wo_time.txt'
		elif message == "canon_in_d":
			music = "canon_in_d_theoretical_wo_time.txt"
		else:
			print("Invalid song file!")
			break

		music_file = file_location + music

		## Variable bookkeeping
		total_num_measures = 0
		last_note_duration = 0
		## 

		with open(music_file) as f:
			for line in f:
				total_num_measures += calc_num_measures(line)
				last_note_duration = duration_of_last_note(line)

		robot_commands = music_commands()
		# Need to separate functionality later on (if more time). For now, both arms mirror each other up until the last measure
		robot_commands.right_arm_motions = right_arm_motion * int(total_num_measures - 1)
		
		print(total_num_measures)
		print(last_note_duration)

		if (last_note_duration == 2):
			robot_commands.left_arm_motions = left_arm_motion * int(total_num_measures - 1)
		elif (last_note_duration == 4):
			robot_commands.left_arm_motions = left_arm_motion * int(total_num_measures - 4)
			robot_commands.left_arm_motions.extend(left_arm_crescendo_rest)
			robot_commands.left_arm_motions.extend(left_arm_crescendo)
			robot_commands.left_arm_motions.extend(left_arm_motion)
		

		# Last measure
		if (last_note_duration == 2):
			robot_commands.right_arm_motions.extend(right_arm_last_note_half)
			robot_commands.left_arm_motions.extend(left_arm_last_note_half)
		elif (last_note_duration == 4):
			robot_commands.right_arm_motions.extend(right_arm_last_note_whole)
			robot_commands.left_arm_motions.extend(left_arm_last_note_whole)


		# Ending sequence
		tempo = np.ones(len(robot_commands.right_arm_motions))

		if (last_note_duration == 2):
			#Cutoff actions, slower speed
			tempo[-3:] = np.ones(3)*1.3
		elif (last_note_duration == 4):
			tempo[-3:] = np.ones(3)*1.5
		robot_commands.tempo = tempo

		print(len(robot_commands.tempo))
		print(len(robot_commands.right_arm_motions))
		print(len(robot_commands.left_arm_motions))


		pub.publish(robot_commands)
		print("Ready to run conductor_motion.py!")
		r.sleep()

if __name__ == '__main__':
	#command = music_commands()
	rospy.init_node('talker', anonymous=True)
	try:
		talker()
	except rospy.ROSInterruptException: pass