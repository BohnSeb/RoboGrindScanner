#! /usr/bin/bash
#cd catkin_ws/Universal_Robots_ROS_Driver;
git submodule init & git submodule update;

#make catkin workspace
cd catkin_ws;
sudo apt update -qq;
rosdep update;
rosdep install --from-paths src --ignore-src -y;
catkin build;

#activate catkin_ws
source devel/setup.bash;