#!/bin/bash
source catkin_ws/devel/setup.bash
roslaunch ur_calibration calibration_correction.launch robot_ip:=192.168.12.200 target_filename:="${HOME}/ur3_calibration.yaml" &
sleep 5
roslaunch ur_robot_driver ur3_bringup.launch robot_ip:=192.168.12.200 kinematics_config:=${HOME}/ur3_calibration.yaml &
sleep 5
roslaunch ur3_moveit_config ur3_moveit_planning_execution.launch &
sleep 5
roslaunch ur3_moveit_config moveit_rviz.launch config:=true