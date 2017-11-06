#!/bin/bash

/usr/bin/tmux new-session -d -s services
/usr/bin/tmux new-window -t services:1 -n roscore 'bash -c "roscore"'
sleep 4
/usr/bin/tmux new-window -t services:2 -n speech_features 'bash -c "roslaunch --wait speech_features server.launch"'
sleep 4
/usr/bin/tmux new-window -t services:3 -n emorec 'bash -c "roslaunch --wait emorec server.launch"'
sleep 4
/usr/bin/tmux new-window -t services:4 -n objectrec 'bash -c "roslaunch --wait objectrec server.launch"'