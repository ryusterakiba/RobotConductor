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
            planner = PathPlanner("both_arms",1)
            joint_goal  = planner._group.get_current_joint_values()
            print(joint_goal)

            #Left arm
            joint_goal[0]  = -0.156
            joint_goal[1]  = -0.314
            joint_goal[2]  = -0.373
            joint_goal[3]  = 0.937
            joint_goal[4]  = 1.124
            joint_goal[5]  = -0.747
            joint_goal[6]  =  -0.659

            #Right arm
            joint_goal[7]  = 0.150
            joint_goal[8]  = -0.431
            joint_goal[9]  = -0.106
            joint_goal[10] = 0.935
            joint_goal[11] = -0.877
            joint_goal[12] = -0.737
            joint_goal[13] =  0.711

            raw_input("Press <Enter> to move both arms to neutral: ")
            planner._group.go(joint_goal,wait=True)
            planner._group.stop()

       
            
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
