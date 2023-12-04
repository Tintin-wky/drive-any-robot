#!/bin/bash

# Create a new tmux session
session_name="create_topomap_$(date +%s)"
tmux new-session -d -s $session_name

# Split the window into three panes
tmux selectp -t 0    # select the first (0) pane
tmux splitw -v -p 50 # split it into two halves
tmux splitw -h -p 50 # split it into two halves
tmux selectp -t 0    # go back to the first pane
tmux splitw -h -p 50 # split it into two halves

# Run roscore in the first pane
tmux select-pane -t 0
tmux send-keys "roscore" Enter

# Run the create_topoplan.py script with command line args in the second pane
tmux select-pane -t 1
tmux send-keys "conda activate gnm_deployment" Enter
tmux send-keys "python create_topomap.py --dt 1 --name $1" Enter

# Image conversion
tmux select-pane -t 2
tmux send-keys "rosrun image_transport republish compressed in:=/camera/left/image_raw raw out:=/camera/left/image_raw" Enter

# Change the directory to ../topomaps/bags and run the rosbag play command in the third pane
tmux select-pane -t 3
# tmux send-keys "mkdir -p ../topomaps/bags" Enter
# tmux send-keys "cd ../topomaps/bags" Enter
tmux send-keys "rosbag play -s 1 -r 5 $2" # feel free to change the playback rate to change the edge length in the graph

# Attach to the tmux session
tmux -2 attach-session -t $session_name
