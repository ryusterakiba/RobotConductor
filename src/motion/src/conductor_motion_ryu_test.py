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
'''
Execution
'''
def execute_beat(beat_path,speed = 1):
    while not rospy.is_shutdown():
        try:
            planner = PathPlanner("right_arm",speed)
            
            waypoints = beat_path(planner._group.get_current_pose().pose)

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

def execute_both_hands(right_beat,left_motion,right_speed,left_speed):
    while not rospy.is_shutdown():
        try:
            planner = PathPlanner("right_arm",speed)
            
            waypoints = beat_path(planner._group.get_current_pose().pose)

            plan = planner.plan_path_with_waypoints(waypoints)

            if not planner.execute_plan(plan):
                raise Exception("Execution failed")

        except Exception as e:
            print e
            traceback.print_exc()
        else:
            break

    
# ============================================================================ #
'''
Paths as list of waypoints
'''
def beat_1_path(current_pose):
    waypoints = []
    waypoints.append(current_pose)

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
        
    return waypoints

def beat_2_path(current_pose):
    waypoints = []
    waypoints.append(current_pose)

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
                
    return waypoints

def beat_3_path(current_pose):
    waypoints = []
    waypoints.append(current_pose)

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
            
    return waypoints

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

def combined_4_4_path(current_pose):
    #Full 4/4 pattern starting at 1
    waypoints_1 = beat_1_path(current_pose)
    waypoints_2 = beat_2_path(waypoints_1[-1])
    waypoints_3 = beat_3_path(waypoints_2[-1])
    waypoints_4 = beat_4_path(waypoints_3[-1])

    waypoints   = waypoints_1 + waypoints_2[1:] + waypoints_3[1:] + waypoints_4[1:]

    return waypoints

def crescendo(current_pose,duration):
    #From neutral, up for DURATION number of beats
    up_length = .5/duration
    goal_pose = copy.deepcopy(current_pose)
    goal_pose.position.z + up_length
    return goal_pose
    
# ============================================================================ #
'''
A way without using waypoints.
Use joint positions
'''
def timed_joint_path(right_arm_motion,left_arm_motion,speed,time):
    while not rospy.is_shutdown():
        for i in range(len(right_arm_motion)):
            try:
                if left_arm_motion[i] == "none":
                    planner = PathPlanner("right_arm",speed)
                    joint_goal = np.loadtxt(right_arm_motion[i] + ".txt").tolist()
                else:
                    planner = PathPlanner("both_arms",speed)
                    joint_goal_right = np.loadtxt(right_arm_motion[i] + ".txt").tolist()
                    joint_goal_left  = np.loadtxt(left_arm_motion[i]+".txt").tolist()
                    joint_goal = joint_goal_left.extend(joint_goal_right)
                
                planner._group.go(joint_goal,wait=False)
                rospy.sleep(time+.5)
                planner._group.stop()

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
    execute_beat(beat_4_path,speed = MOTION_SPEED_4)
    execute_beat(combined_4_4_path)



if __name__ == '__main__':
    joint_state_topic = ['joint_states:=/robot/joint_states']
    moveit_commander.roscpp_initialize(joint_state_topic)
    rospy.init_node('moveit_node',anonymous=False)
    roscpp_initialize(sys.argv)
    main()
