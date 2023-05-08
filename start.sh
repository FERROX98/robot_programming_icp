#!/usr/bin/bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
echo "----------------------------"

roscore &> /dev/null & 
sleep 1
roscd nicp_localization 
cd .. 

rviz -d test_data/rviz.rviz &> /dev/null & 
echo -e "rviz started\n"

rosrun map_server map_server test_data/cappero_map.yaml  &> /dev/null & 
echo -e "map_server started\n"

rosrun stage_ros stageros test_data/cappero.world &> /dev/null & 
echo -e "stage_ros started\n"

rosrun nicp_localization normal_viewer  &> /dev/null & 
echo -e "normal_viewer started\n"
echo -e "Complete\n"
echo -e "Starting localizer_node\n" 
echo "----------------------------"
sleep 5
rosrun nicp_localization localizer_node 




