#!/bin/bash

# Create a new tmux session
session_name="record_bag_$(date +%s)"
tmux new-session -d -s $session_name

# Change the directory to ../topomaps/bags and run the rosbag record command
tmux select-pane -t 0
tmux send-keys "cd ../bags/raw" Enter
tmux send-keys "rosbag record /camera/left/image_raw/compressed /camera/right/image_raw/compressed /scout_status /odom_chassis /robot_pose_ekf/odom_combined /gps/gps /imu -o $1" # change topic if necessary

# Attach to the tmux session
tmux -2 attach-session -t $session_name