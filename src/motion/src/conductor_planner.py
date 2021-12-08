#!/usr/bin/env python
"""
Path Planner Class for Lab 7
Author: Valmik Prabhu

Modified by Ryu Akiba for robot conductor project
"""

import sys
import rospy
import copy

import moveit_commander
from moveit_msgs.msg import OrientationConstraint, Constraints, RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Pose

class PathPlanner(object):
    """
    Path Planning Functionality for Baxter/Sawyer

    Fields:
    _robot: moveit_commander.RobotCommander; for interfacing with the robot
    _scene: moveit_commander.PlanningSceneInterface; the planning scene stores a representation of the environment
    _group: moveit_commander.MoveGroupCommander; the move group is moveit's primary planning class
    _planning_scene_publisher: ros publisher; publishes to the planning scene
    """
    def __init__(self, group_name):
        """
        Constructor.

        Inputs:
        group_name: the name of the move_group.
            For Baxter, this would be 'left_arm' or 'right_arm'
        """
        
        # If the node is shutdown, call this function    
        rospy.on_shutdown(self.shutdown)

        # Initialize moveit_commander
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize the robot
        self._robot = moveit_commander.RobotCommander()

        # Initialize the planning scene
        self._scene = moveit_commander.PlanningSceneInterface()

        # Instantiate a move group
        self._group = moveit_commander.MoveGroupCommander(group_name)

        # Set the maximum time MoveIt will try to plan before giving up
        self._group.set_planning_time(2)

        # Set the bounds of the workspace
        self._group.set_workspace([-2, -2, -2, 2, 2, 2])

        # Set the velocity of the arm
        self._group.set_max_velocity_scaling_factor(1.0))
        self._group.set_max_acceleration_scaling_factor(1.0)
        
        # Sleep for a bit to ensure that all inititialization has finished
        rospy.sleep(0.5)

    def shutdown(self):
        """
        Code to run on shutdown. This is good practice for safety

        Currently deletes the object's MoveGroup, so that further commands will do nothing
        """
        self._group = None
        rospy.loginfo("Stopping Path Planner")

    def plan_to_pose(self, target):
        """
        Generates a plan given an end effector pose subject to orientation constraints

        Inputs:
        target: A geometry_msgs/PoseStamped message containing the end effector pose goal
        orientation_constraints: A list of moveit_msgs/OrientationConstraint messages

        Outputs:
        path: A moveit_msgs/RobotTrajectory path
        """

        self._group.set_pose_target(target)
        self._group.set_start_state_to_current_state()


        plan = self._group.plan()

        return plan

    def plan_to_joint_goal(self, target,speed):
        """
        Generates a plan given an joint configuration

        Inputs:
        target: A geometry_msgs/PoseStamped message containing the end effector pose goal

        Outputs:
        path: A moveit_msgs/RobotTrajectory path scaled to speed
        """
        try:
            
        self._group.set_joint_value_target(target)
        self._group.set_start_state_to_current_state()

        plan = self._group.plan()

        new_plan = copy.deepcopy(plan)
        n_points = len(plan.joint_trajectory.points)
        n_joints = len(plan.joint_trajectory.joint_names) #should be 14 for both arms
        
        #Iterate over JointTrajectoryPoint to scale velocity
        for i in range(n_points):            
            time_step = plan.joint_trajectory.points[i].time_from_start / speed
            new_plan.joint_trajectory.points[i].time_from_start = time_step
                
            for j in range(n_joints):
                scaled_speed = plan.joint_trajectory.points[i].velocities[j] * speed
                new_plan.joint_trajectory.points[i].velocities.append(plan.joint_trajectory.points[i].velocities[j])

        return new_plan

    def plan_path_with_waypoints(self, waypoints):
        """
        Generates a plan given a list of waypoints for the end effector to go through

        Inputs:
        waypoints: a list of geometry_msgs/Pose
        Outputs:
        path: A moveit_msgs/RobotTrajectory path
        """

        (plan,fraction) = self._group.compute_cartesian_path(waypoints,.02,0.0)

        return plan

    def execute_plan(self, plan,timed = False):
        """
        Uses the robot's built-in controllers to execute a plan

        Inputs:
        plan: a moveit_msgs/RobotTrajectory plan
        """

        return self._group.execute(plan, wait = not(timed))

        
