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

MOTION_SPEED_1 = 1
MOTION_SPEED_2 = .2
MOTION_SPEED_3 = .8
MOTION_SPEED_4 = 1

# ============================================================================ #

def beat_4_path(current_pose):
    waypoints = [current_pose]

    #Waypoint
    wpose = Pose()
    wpose.orientation.x = waypoints[0].orientation.x;
    wpose.orientation.y = waypoints[0].orientation.y;
    wpose.orientation.z = waypoints[0].orientation.z;
    wpose.orientation.w = waypoints[0].orientation.w;
    
    wpose.position.x    = waypoints[0].position.x
    wpose.position.y    = waypoints[0].position.y+.3
    wpose.position.z    = waypoints[0].position.z+.1
    waypoints.append(copy.deepcopy(wpose))

    #position beat 4 at top
    wpose = Pose()
    wpose.orientation.x = waypoints[0].orientation.x;
    wpose.orientation.y = waypoints[0].orientation.y;
    wpose.orientation.z = waypoints[0].orientation.z;
    wpose.orientation.w = waypoints[0].orientation.w;
    
    wpose.position.x    = 1.097
    wpose.position.y    = -0.452
    wpose.position.z    = 0.593
    waypoints.append(copy.deepcopy(wpose))

    return waypoints

def beat_1_path(current_pose):

    return waypoints

def beat_2_path(current_pose):

    return waypoints

def execute_beat(beat,speed):
    # if beat == 1:
    #     path = beat_1_path
    # elif beat == 2:
    #     path = beat_2_path
    # elif beat == 3:
    #     path = beat_3_path
    # elif beat == 4:
    #     path = beat_4_path
    path = beat_4_path

    while not rospy.is_shutdown():
        try:
            planner = PathPlanner("right_arm",speed)
            
            waypoints = path(planner._group.get_current_pose().pose)

            plan = planner.plan_path_with_waypoints(waypoints)

            start_time = rospy.get_time()
            if not planner.execute_plan(plan):
                raise Exception("Execution failed")
            print("Time to complete: " + str(rospy.get_time()-start_time))

        except Exception as e:
            print e
            traceback.print_exc()
        else:
            break

def beat_4():
    # From right (beat 3 in 4/4)
    while not rospy.is_shutdown():
        try:
            planner = PathPlanner("right_arm",MOTION_SPEED_4)
            
            waypoints = beat_4_path(planner._group.get_current_pose().pose)

            plan = planner.plan_path_with_waypoints(waypoints)

            start_time = rospy.get_time()
            if not planner.execute_plan(plan):
                raise Exception("Execution failed")
            print("Time to complete: " + str(rospy.get_time()-start_time))

        except Exception as e:
            print e
            traceback.print_exc()
        else:
            break

def beat_1():
    # From top, down and bounce
    while not rospy.is_shutdown():
        try:
            planner = PathPlanner("right_arm",MOTION_SPEED_1)
            
            waypoints = []
            waypoints.append(planner._group.get_current_pose().pose)

            #Waypoint
            wpose = Pose()
            wpose.orientation.x = waypoints[0].orientation.x;
            wpose.orientation.y = waypoints[0].orientation.y;
            wpose.orientation.z = waypoints[0].orientation.z;
            wpose.orientation.w = waypoints[0].orientation.w;
            
            wpose.position.x    = waypoints[0].position.x
            wpose.position.y    = waypoints[0].position.y
            wpose.position.z    = waypoints[0].position.z-.5
            waypoints.append(copy.deepcopy(wpose))

            #position beat 4 at top
            wpose = Pose()
            wpose.orientation.x = waypoints[0].orientation.x;
            wpose.orientation.y = waypoints[0].orientation.y;
            wpose.orientation.z = waypoints[0].orientation.z;
            wpose.orientation.w = waypoints[0].orientation.w;
            
            wpose.position.x    = waypoints[0].position.x
            wpose.position.y    = waypoints[0].position.y
            wpose.position.z    = waypoints[1].position.z+.1
            waypoints.append(copy.deepcopy(wpose))
            
            plan = planner.plan_path_with_waypoints(waypoints)

            #raw_input("Press <Enter> to move beat 1: ")

            start_time = rospy.get_time()
            if not planner.execute_plan(plan):
                raise Exception("Execution failed")
            print("Time to complete: " + str(rospy.get_time()-start_time))

        except Exception as e:
            print e
            traceback.print_exc()
        else:
            break

def beat_2():
    # From bottom (beat 1 in 4/4)
    while not rospy.is_shutdown():
        try:
            planner = PathPlanner("right_arm",MOTION_SPEED_2)
            
            waypoints = []
            waypoints.append(planner._group.get_current_pose().pose)

            #Waypoints
            for i in range(4):
                wpose = Pose()
                wpose.orientation.x = waypoints[0].orientation.x;
                wpose.orientation.y = waypoints[0].orientation.y;
                wpose.orientation.z = waypoints[0].orientation.z;
                wpose.orientation.w = waypoints[0].orientation.w;
                
                wpose.position.x    = waypoints[0].position.x
                wpose.position.y    = waypoints[0].position.y+.08*i
                wpose.position.z    = waypoints[0].position.z+.08*(0.5*i)**2
                waypoints.append(copy.deepcopy(wpose))
            
            plan = planner.plan_path_with_waypoints(waypoints)

            #raw_input("Press <Enter> to move beat 2: ")

            start_time = rospy.get_time()
            if not planner.execute_plan(plan):
                raise Exception("Execution failed")
            print("Time to complete: " + str(rospy.get_time()-start_time))

        except Exception as e:
            print e
            traceback.print_exc()
        else:
            break

def beat_3():
    # From left (beat 2 in 4/4)
    while not rospy.is_shutdown():
        try:
            planner = PathPlanner("right_arm",MOTION_SPEED_3)
            
            waypoints = []
            waypoints.append(planner._group.get_current_pose().pose)

            #Waypoint
            wpose = Pose()
            wpose.orientation.x = waypoints[0].orientation.x;
            wpose.orientation.y = waypoints[0].orientation.y;
            wpose.orientation.z = waypoints[0].orientation.z;
            wpose.orientation.w = waypoints[0].orientation.w;
            
            wpose.position.x    = waypoints[0].position.x
            wpose.position.y    = waypoints[0].position.y-.4
            wpose.position.z    = waypoints[0].position.z-.2
            waypoints.append(copy.deepcopy(wpose))

            #position beat 3 at top
            wpose = Pose()
            wpose.orientation.x = waypoints[0].orientation.x;
            wpose.orientation.y = waypoints[0].orientation.y;
            wpose.orientation.z = waypoints[0].orientation.z;
            wpose.orientation.w = waypoints[0].orientation.w;
            
            wpose.position.x    = 1.028
            wpose.position.y    = -0.795
            wpose.position.z    = 0.288
            waypoints.append(copy.deepcopy(wpose))
            
            plan = planner.plan_path_with_waypoints(waypoints)

            #raw_input("Press <Enter> to move beat 3: ")

            start_time = rospy.get_time()
            if not planner.execute_plan(plan):
                raise Exception("Execution failed")
            print("Time to complete: " + str(rospy.get_time()-start_time))

        except Exception as e:
            print e
            traceback.print_exc()
        else:
            break

# ============================================================================ #

def main():
    """
    Main Script
    Assumes we started at initial config (position beat 3)
    """
    #4/4 beginning measure sequence
    execute_beat(4,MOTION_SPEED_4)
    beat_1()
    beat_2()
    beat_3()
    execute_beat(4,MOTION_SPEED_4)



if __name__ == '__main__':
    joint_state_topic = ['joint_states:=/robot/joint_states']
    moveit_commander.roscpp_initialize(joint_state_topic)
    rospy.init_node('moveit_node',anonymous=False)
    roscpp_initialize(sys.argv)
    main()
