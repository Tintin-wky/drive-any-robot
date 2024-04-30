#!/bin/bash

# Create a new tmux session
session_name="gnm_classbot_$(date +%s)"
tmux new-session -d -s $session_name

# Split the window into four panes
tmux selectp -t 0    # select the first (0) pane
tmux splitw -h -p 50 # split it into two halves

# Run the navigate.py script with command line args in the first pane
tmux select-pane -t 0
tmux send-keys "conda activate gnm_deployment" Enter
tmux send-keys "python create_topomap.py -s $@" Enter

# Change the directory to ../navigate/bags and run the rosbag record command in the second pane
tmux select-pane -t 1
tmux send-keys "cd ../bags/explore" Enter
tmux send-keys "rosbag record /rosout /camera/left/image_raw/compressed /camera/right/image_raw/compressed /scout_status /odom_chassis /robot_pose_ekf/odom_combined /gps/gps /imu /imu/data_raw -o gnm_classbot" Enter # change topic if necessary

# Attach to the tmux session
tmux -2 attach-session -t $session_name
