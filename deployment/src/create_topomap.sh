#!/bin/bash

# Create a new tmux session
session_name="create_topomap_$(date +%s)"
tmux new-session -d -s $session_name

# Split the window into three panes
tmux selectp -t 0    # select the first (0) pane
tmux splitw -v -p 50 # split it into two halves

# Run roscore in the first pane
tmux select-pane -t 0
tmux send-keys "roscore" Enter

# Run the create_topoplan.py script with command line args in the second pane
tmux select-pane -t 1
tmux send-keys "conda activate gnm_deployment" Enter
tmux send-keys "python create_topomap.py --name $1 --rosbag $2" Enter
tmux send-keys "python topomap_visualization.py --name $1" Enter

# Attach to the tmux session
tmux -2 attach-session -t $session_name
