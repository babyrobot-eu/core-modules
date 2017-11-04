#!/bin/bash

tmux new-session -d -s services
tmux new-window -t services:1 -n roscore 'bash -c "roscore"'
sleep 4
tmux new-window -t services:2 -n speech_features 'bash -c "roslaunch speech_features server.launch"'
sleep 4
tmux new-window -t services:3 -n emorec 'bash -c "roslaunch emorec server.launch"'
sleep 4
tmux new-window -t services:4 -n objectrec 'bash -c "roslaunch objectrec server.launch"'
