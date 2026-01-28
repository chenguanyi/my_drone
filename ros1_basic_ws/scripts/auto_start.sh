#!/bin/bash
echo "Start script at $(date)" >> /tmp/auto_start.log
source /opt/ros/noetic/setup.bash
source /home/orangepi/ros1_basic_ws/devel/setup.bash
sleep 5
echo "Launching ROS Noetic..." >> /tmp/auto_start.log
roslaunch top_launch_pkg .launch >> /tmp/auto_start.log 2>&1
echo "End script at $(date)" >> /tmp/auto_start.log