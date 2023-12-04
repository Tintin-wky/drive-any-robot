#!/bin/bash

# Create a new tmux session
session_name="record_bag_$(date +%s)"
tmux new-session -d -s $session_name
pass_word="classlab"

# Split the window into three panes
tmux selectp -t 0    # select the first (0) pane
tmux splitw -v -p 50 # split it into two halves
tmux selectp -t 0    # go back to the first pane
tmux splitw -h -p 50 # split it into two halves

# Run the can0 connect command in the first pane
tmux select-pane -t 0
tmux send-keys "echo ${pass_word} | sudo -S modprobe gs_usb" Enter
tmux send-keys "echo ${pass_word} | sudo -S ip link set can0 up type can bitrate 500000" Enter
tmux send-keys "candump can0" Enter

# Run the launch command in the second pane
tmux select-pane -t 1
tmux send-keys "roslaunch gnm_classbot.launch" Enter

# Change the directory to ../topomaps/bags and run the rosbag record command in the third pane
tmux select-pane -t 2
tmux send-keys "cd ../topomaps/bags" Enter
tmux send-keys "rosbag record /camera/left/image_raw/compressed /camera/right/image_raw/compressed /scout_status /cmd_vel /odom_chassis -o $1" # change topic if necessary

# Attach to the tmux session
tmux -2 attach-session -t $session_name