#!/usr/bin/bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
echo "----"

roscore &> /dev/null & 
sleep 1
roscd nicp_localization 
cd .. 
rviz -d test_data/rviz.rviz &> /dev/null & 
echo "rviz started"
rosrun map_server map_server test_data/cappero_map.yaml  &> /dev/null & 
echo "map_server started"
rosrun stage_ros stageros test_data/cappero.world &> /dev/null & 
echo "stage_ros started"rosrun nicp_localization normal_viewer  &> /dev/null & 
rosrun nicp_localization normal_viewer  &> /dev/null & 
echo "normal_viewer started"rosrun nicp_localization normal_viewer  &> /dev/null & 
rosrun nicp_localization localizer_node 
echo "localizer_node started"rosrun nicp_localization normal_viewer  &> /dev/null & 
echo "Complete"

