#!/bin/bash

# Create a new tmux session
session_name="record_bag_$(date +%s)"
tmux new-session -d -s $session_name
pass_word="classlab"

# Change the directory to ../topomaps/bags and run the rosbag record command
tmux select-pane -t 0
tmux send-keys "cd ../topomaps/bags/raw" Enter
tmux send-keys "rosbag record /camera/left/image_raw/compressed /camera/right/image_raw/compressed /scout_status /odom_chassis -o $1" # change topic if necessary

# Attach to the tmux session
tmux -2 attach-session -t $session_name