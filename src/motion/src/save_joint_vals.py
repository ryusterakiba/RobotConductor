#!/usr/bin/env python
"""
robot conductor project
Ryu Akiba, Cooper Collier
"""

from baxter_interface import Limb

import sys
import rospy
import numpy as np
import traceback

from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import PoseStamped
from moveit_commander import roscpp_initialize
import moveit_commander

from conductor_planner import PathPlanner
'''
Save joint positions as text files
'''

def main():
    """
    Bring Baxter to neutral position
    """
    while not rospy.is_shutdown():

        try:

            #Both arms to initial position
            planner1        = PathPlanner("left_arm")
            planner2        = PathPlanner("right_arm")
            current_joint1  = planner1._group.get_current_joint_values()
            current_joint2  = planner2._group.get_current_joint_values()

            file_location = '/home/cc/ee106a/fl21/class/ee106a-abw/ros_workspaces/robot_conductor_ryu/src/motion/positions/'
            pose_name1 = raw_input("Enter name for left pose: ")
            np.savetxt(file_location + pose_name1 +".txt",np.array(current_joint1))
            pose_name2 = raw_input("Enter name for right pose: ")
            np.savetxt(file_location + pose_name2 +".txt",np.array(current_joint2))
            
        except Exception as e:
            print e
            traceback.print_exc()
        else:
            break

if __name__ == '__main__':
    joint_state_topic = ['joint_states:=/robot/joint_states']
    moveit_commander.roscpp_initialize(joint_state_topic)
    rospy.init_node('joint_val_node',anonymous=False)
    roscpp_initialize(sys.argv)
    main()
