#!/bin/bash

# Create a new tmux session
session_name="gnm_classbot_$(date +%s)"
tmux new-session -d -s $session_name
log="gnm_classbot_$(date +%s)"
pass_word="classlab"

# Split the window into four panes
tmux selectp -t 0    # select the first (0) pane
tmux splitw -v -p 50 # split it into two halves
tmux selectp -t 1    # select the first (0) pane
tmux splitw -h -p 50 # split it into two halves
tmux selectp -t 0    # go back to the first pane

# Run the ros launch command in the first pane

# Run the navigate.py script with command line args in the first pane
tmux select-pane -t 0
tmux send-keys "conda activate gnm_deployment" Enter
tmux send-keys "python navigate.py $@" Enter

# Run the pd_controller.py script in the second pane
tmux select-pane -t 1
tmux send-keys "conda activate gnm_deployment" Enter
tmux send-keys "python pd_controller.py" Enter

# Change the directory to ../navigate/bags and run the rosbag record command in the third pane
tmux select-pane -t 2
tmux send-keys "cd ../bags/navigate" Enter
tmux send-keys "rosbag record /rosout /camera/left/image_raw/compressed /camera/right/image_raw/compressed /scout_status /cmd_vel /odom_chassis -o $log" Enter # change topic if necessary

# Attach to the tmux session
tmux -2 attach-session -t $session_name
