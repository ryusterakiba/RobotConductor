#!/usr/bin/env bash

# Before running this script, connect to the robot with:
# ./baxter.sh asimov.local

rosrun baxter_tools enable_robot.py -e

echo " ===================== Enabled robot"

rosrun baxter_tools tuck_arms.py -u

echo "===================== Robot is set to neutral position"

rosrun baxter_interface joint_trajectory_action_server.py &

echo "===================== Launched joint trajectory action server"

# Run command to take camera input and write to file: camera_input/SONG.png

# echo "===================== Recorded song input"

cd src/sensing/src
python3 main.py
sed '$ s/.$//' output/SONG.txt | sed '1d' > output/SONG_edited.txt
cd ../../..

echo "===================== Parsed song input"

roslaunch baxter_conductor_moveit_config demo_baxter.launch right_electric_gripper:=true left_electric_gripper:=true &

echo "===================== Launched motion planner"

echo $1 > bpm.txt

rosrun motion output_to_conductor_cmd_pub.py &

echo "===================== Sent song input to motion controller"

rosrun motion conductor_motion.py

echo "===================== Starting conductor routine"
