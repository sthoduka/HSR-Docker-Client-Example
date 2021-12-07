#!/bin/bash
echo "192.168.1.211 hsrc" >> /etc/hosts
export ROS_MASTER_URI=http://192.168.1.211:11311/
export ROS_IP=192.168.1.178
. /opt/ros/melodic/setup.sh && cd /home/root/catkin_ws && catkin_make
source /home/root/catkin_ws/devel/setup.bash
rosrun hsr_head_positioner hsr_head_positioner.py
