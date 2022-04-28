#!/bin/bash
source catkin_ws/devel/setup.bash
roslaunch ur_gazebo ur3_bringup.launch &
sleep 2
roslaunch ur3_moveit_config ur3_moveit_planning_execution.launch sim:=true &
sleep 5
roslaunch ur3_moveit_config moveit_rviz.launch config:=true