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


def main():
    """
    Bring Baxter to neutral position
    """
    while not rospy.is_shutdown():

        try:
            #Both arms to initial position
            planner        = PathPlanner("both_arms",1)
            current_joint  = planner._group.get_current_joint_values()

            pose_name = input("Enter name for pose: ")
            file_location = ''
            np.savetxt(pose_name+".txt",np.array(current_joint))
            
        except Exception as e:
            print e
            traceback.print_exc()
        else:
            break

if __name__ == '__main__':
    joint_state_topic = ['joint_states:=/robot/joint_states']
    moveit_commander.roscpp_initialize(joint_state_topic)
    rospy.init_node('moveit_node',anonymous=False)
    roscpp_initialize(sys.argv)
    main()
