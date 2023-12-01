#!/bin/bash

# Create a new tmux session
session_name="gnm_classnpt_$(date +%s)"
tmux new-session -d -s $session_name
pass_word="classlab"

# Split the window into two panes
tmux selectp -t 0    # select the first (0) pane
tmux splitw -v -p 50 # split it into two halves

# Run the can0 connect command in the first pane
tmux select-pane -t 0
tmux send-keys "echo ${pass_word} | modprobe gs_usb" Enter
tmux send-keys "ip link set can0 up type can bitrate 500000" Enter
tmux send-keys "candump can0" Enter

# Run the ros launch command in the second pane
tmux select-pane -t 1
tmux send-keys "roslaunch gnm_classbot.launch" Enter

# Attach to the tmux session
tmux -2 attach-session -t $session_name