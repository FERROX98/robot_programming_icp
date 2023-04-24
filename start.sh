#!/usr/bin/bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
echo "START"

roscore &> /dev/null & 
sleep 1
roscd nicp_localization 
cd .. 
rviz -d test_data/rviz.rviz &> /dev/null & 
rosrun map_server map_server test_data/cappero_map.yaml  &> /dev/null & 
rosrun stage_ros stageros test_data/cappero.world &> /dev/null & 
echo "END"


