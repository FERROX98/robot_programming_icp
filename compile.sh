#!/usr/bin/bash

cd ~/catkin_ws/
echo "Start compiling"
catkin_make &>/dev/null
echo "Complete" 
