#!/bin/bash

# Create a new tmux session
session_name="gnm_locobot_$(date +%s)"
tmux new-session -d -s $session_name

# Split the window into four panes
tmux selectp -t 0    # select the first (0) pane
tmux splitw -h -p 50 # split it into two halves
tmux selectp -t 0    # select the first (0) pane
tmux splitw -v -p 50 # split it into two halves

tmux selectp -t 2    # select the new, second (2) pane
tmux splitw -v -p 50 # split it into two halves
tmux selectp -t 0    # go back to the first pane

# Run the ros launch command in the first pane
tmux select-pane -t 1
tmux send-keys "roslaunch gnm_classbot.launch" Enter

# Run the navigate.py script with command line args in the second pane
tmux select-pane -t 1
tmux send-keys "conda activate gnm_deployment" Enter
tmux send-keys "python navigate.py $@" Enter

# Run the remote control interface in the third pane
tmux select-pane -t 2
tmux send-keys "roslaunch scout_bringup scout_teleop_keyboard.launch" Enter

# Run the pd_controller.py script in the fourth pane
tmux select-pane -t 3
tmux send-keys "conda activate gnm_deployment" Enter
tmux send-keys "python pd_controller.py" Enter

# Attach to the tmux session
tmux -2 attach-session -t $session_name
