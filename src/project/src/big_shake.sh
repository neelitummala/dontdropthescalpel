#!/bin/bash
 
source /scratch/shared/baxter_ws/devel/setup.bash
export ROS_MASTER_URI="http://ayrton.local:11311/"
export ROS_HOSTNAME="192.168.1.8"

rosrun baxter_examples joint_trajectory_file_playback.py -f big_shake.rec

