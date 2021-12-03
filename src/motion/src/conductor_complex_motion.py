#!/usr/bin/env python
"""
Execute complex conducting path sequence
Ryu Akiba
"""

from baxter_interface import Limb

import rospy
import numpy as np
import sys
import copy
import traceback

from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import PoseStamped, Pose
from moveit_commander import roscpp_initialize
import moveit_commander

from conductor_planner import PathPlanner

def main():
    """
    Main Script
    """

    while not rospy.is_shutdown():
        # From top, down and bounce
        while not rospy.is_shutdown():
            try:
                planner = PathPlanner("right_arm",.4)
                
                waypoints = []
                waypoints.append(planner._group.get_current_pose().pose)

                #Down
                wpose = Pose()
                wpose.orientation.y = -1.0;
                wpose.position.x = waypoints[0].position.x
                wpose.position.y = waypoints[0].position.y
                wpose.position.z = waypoints[0].position.z-.4
                waypoints.append(copy.deepcopy(wpose))

                #bounce
                wpose.position.z = waypoints[1].position.z +.1
                waypoints.append(copy.deepcopy(wpose))
                
                plan = planner.plan_path_with_waypoints(waypoints)

                raw_input("Press <Enter> to move beat 1: ")

                start_time = rospy.get_time()
                if not planner.execute_plan(plan):
                    raise Exception("Execution failed")
                print("Time to complete: " + str(rospy.get_time()-start_time))

            except Exception as e:
                print e
                traceback.print_exc()
            else:
                break
            
        # to the left, loop
        while not rospy.is_shutdown():
            try:
                planner = PathPlanner("right_arm",.4)
                
                waypoints2 = []
                waypoints2.append(planner._group.get_current_pose().pose)

                #left/down
                wpose = Pose()
                wpose.orientation.y = -1.0;
                wpose.position.x = waypoints2[0].position.x
                wpose.position.y = waypoints2[0].position.y-.3
                wpose.position.z = waypoints2[0].position.z-.1
                waypoints2.append(copy.deepcopy(wpose))

                # loop 1
                wpose.position.y = waypoints2[1].position.y-.2
                wpose.position.z = waypoints2[1].position.z+.1
                waypoints2.append(copy.deepcopy(wpose))
                
                #loop 2
                wpose.position.y = waypoints2[2].position.y+.2
                wpose.position.z = waypoints2[2].position.z+.1
                waypoints2.append(copy.deepcopy(wpose))

                plan = planner.plan_path_with_waypoints(waypoints2)

                raw_input("Press <Enter> to move beat 2: ")

                start_time = rospy.get_time()
                if not planner.execute_plan(plan):
                    raise Exception("Execution failed")
                print("Time to complete: " + str(rospy.get_time()-start_time))

            except Exception as e:
                print e
                traceback.print_exc()
            else:
                break

        # # to the right, flick up
        # while not rospy.is_shutdown():
        #     try:
        #         planner = PathPlanner("right_arm",.5)
                
        #         waypoints = []
        #         waypoints.append(planner._group.get_current_pose().pose)

        #         #right/up to center
        #         wpose = Pose()
        #         wpose.orientation.y = -1.0;
        #         wpose.position.x = 
        #         wpose.position.y = waypoints[0].position.y
        #         wpose.position.z = 
        #         waypoints.append(copy.deepcopy(wpose))

        #         #right/down 
        #         wpose.position.x = 
        #         wpose.position.z = 
        #         waypoints.append(copy.deepcopy(wpose))
                
        #         #flick right/up
        #         wpose.position.x = 
        #         wpose.position.z = 
        #         waypoints.append(copy.deepcopy(wpose))
                
        #         plan = planner.plan_path_with_waypoints(waypoints)

        #         raw_input("Press <Enter> to move beat: ")

        #         start_time = rospy.get_time()
        #         if not planner.execute_plan(plan):
        #             raise Exception("Execution failed")
        #         print("Time to complete: " + str(rospy.get_time()-start_time))

        #     except Exception as e:
        #         print e
        #         traceback.print_exc()
        #     else:
        #         break

        # # curve to top
        # while not rospy.is_shutdown():
        #     try:
        #         planner = PathPlanner("right_arm",.5)
                
        #         waypoints = []
        #         waypoints.append(planner._group.get_current_pose().pose)

        #         #left down
        #         wpose = Pose()
        #         wpose.orientation.y = -1.0;
        #         wpose.position.x = 
        #         wpose.position.y = waypoints[0].position.y
        #         wpose.position.z = 
        #         waypoints.append(copy.deepcopy(wpose))

        #         #left/up 
        #         wpose.position.x = 
        #         wpose.position.z = 
        #         waypoints.append(copy.deepcopy(wpose))
                
        #         plan = planner.plan_path_with_waypoints(waypoints)

        #         raw_input("Press <Enter> to move beat: ")

        #         start_time = rospy.get_time()
        #         if not planner.execute_plan(plan):
        #             raise Exception("Execution failed")
        #         print("Time to complete: " + str(rospy.get_time()-start_time))

        #     except Exception as e:
        #         print e
        #         traceback.print_exc()
        #     else:
        #         break


if __name__ == '__main__':
    joint_state_topic = ['joint_states:=/robot/joint_states']
    moveit_commander.roscpp_initialize(joint_state_topic)
    rospy.init_node('moveit_node',anonymous=False)
    roscpp_initialize(sys.argv)
    main()
