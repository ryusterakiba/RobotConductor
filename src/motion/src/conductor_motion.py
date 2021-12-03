#!/usr/bin/env python
"""
Path Planning Script for Lab 7
Author: Valmik Prabhu

Modified by Ryu Akiba for robot conductor project
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
    Main Script
    """
    
    # #Create a path constraint for the arm
    # orien_const = OrientationConstraint()
    # orien_const.link_name = "right_gripper";
    # orien_const.header.frame_id = "base";
    # orien_const.orientation.y = 1.0;
    # orien_const.absolute_x_axis_tolerance = 0.1;
    # orien_const.absolute_y_axis_tolerance = 0.1;
    # orien_const.absolute_z_axis_tolerance = 0.1;
    # orien_const.weight = 1.0;

    while not rospy.is_shutdown():
        # Neutral position
        while not rospy.is_shutdown():
            try:
                neutral = PoseStamped()
                neutral.header.frame_id = "base"

                #x, y, and z position 
                translation = [0.759, -0.453, 0.258]
                neutral.pose.position.x = translation[0]
                neutral.pose.position.y = translation[1]
                neutral.pose.position.z = translation[2]

                #Orientation as a quaternion
                rotation = [0.0,1.0,0.0,0.0]

                neutral.pose.orientation.x = rotation[0]
                neutral.pose.orientation.y = rotation[1]
                neutral.pose.orientation.z = rotation[2]
                neutral.pose.orientation.w = rotation[3]

                planner = PathPlanner("right_arm",.4)     
                plan = planner.plan_to_pose(neutral, [])
                print(planner._group.get_current_pose().pose)
                raw_input("Press <Enter> to move the right arm to neutral: ")

                start_time = rospy.get_time()
                if not planner.execute_plan(plan):
                    raise Exception("Execution failed")
                print(rospy.get_time()-start_time)

            except Exception as e:
                print e
                traceback.print_exc()
            else:
                break
        # Beat 4
        while not rospy.is_shutdown():
            try:
                beat_4 = PoseStamped()
                beat_4.header.frame_id = "base"

                #x, y, and z position
                beat_4.pose.position.x = translation[0]
                beat_4.pose.position.y = translation[1]
                beat_4.pose.position.z = translation[2]-0.2

                #Orientation as a quaternion
                rotation = [0.0,1.0,0.0,0.0]
                beat_4.pose.orientation.x = rotation[0]
                beat_4.pose.orientation.y = rotation[1]
                beat_4.pose.orientation.z = rotation[2]
                beat_4.pose.orientation.w = rotation[3]
                
                planner = PathPlanner("right_arm",.8)
                plan = planner.plan_to_pose(beat_4, [])

                raw_input("Press <Enter> to move the right arm to Beat 4: ")
                start_time = rospy.get_time()
                if not planner.execute_plan(plan):
                    raise Exception("Execution failed")
                print(rospy.get_time()-start_time)
            except Exception as e:
                print e
            else:
                break

        # # Beat 2
        # while not rospy.is_shutdown():
        #     try:
        #         beat_1 = PoseStamped()
        #         beat_1.header.frame_id = "base"

        #         #x, y, and z position
        #         beat_1.pose.position.x = 0.6
        #         beat_1.pose.position.y = -0.1
        #         beat_1.pose.position.z = 0.1

        #         #Orientation as a quaternion
        #         beat_1.pose.orientation.x = 0.0
        #         beat_1.pose.orientation.y = -1.0
        #         beat_1.pose.orientation.z = 0.0
        #         beat_1.pose.orientation.w = 0.0

        #         velocity = input("Input velocity of arm (0,1]: ")
        #         planner = PathPlanner("right_arm",float(velocity))
        #         plan = planner.plan_to_pose(beat_1, [orien_const])
                

        #         raw_input("Press <Enter> to move the right arm to Beat 1: ")
        #         start_time = rospy.get_time()
        #         if not planner.execute_plan(plan):
        #             raise Exception("Execution failed")
        #         print(rospy.get_time()-start_time)
        #     except Exception as e:
        #         print e
        #     else:
        #         break

if __name__ == '__main__':
    joint_state_topic = ['joint_states:=/robot/joint_states']
    moveit_commander.roscpp_initialize(joint_state_topic)
    rospy.init_node('moveit_node',anonymous=False)
    roscpp_initialize(sys.argv)
    main()
