#!/usr/bin/bash

cd ~/catkin_ws/
echo "Start compiling"
catkin_make &>catkin_make.log
if [ $(echo $?) = 0 ]; then
    echo "Complete" 
else 
    echo -e "ERROR\n"
    tail -n 50 catkin_make.log
fi 

rm catkin_make.log
