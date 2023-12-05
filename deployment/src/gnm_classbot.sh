#!/bin/bash

# Create a new tmux session
session_name="gnm_classbot_$(date +%s)"
tmux new-session -d -s $session_name
pass_word="classlab"

# Run the launch command
tmux select-pane -t 0
tmux send-keys "echo ${pass_word} | sudo -S modprobe gs_usb" Enter
tmux send-keys "echo ${pass_word} | sudo -S ip link set can0 up type can bitrate 500000" Enter
tmux send-keys "roslaunch gnm_classbot.launch" Enter

# Attach to the tmux session
tmux -2 attach-session -t $session_name