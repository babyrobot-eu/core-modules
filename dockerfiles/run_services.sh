#!/bin/bash

/usr/bin/tmux new-session -d -s services
/usr/bin/tmux new-window -t services:1 -n roscore 'bash -c "roscore"'
sleep 2
/usr/bin/tmux new-window -t services:2 -n speech_features 'bash -c "roslaunch --wait speech_features server.launch"'
sleep 2
/usr/bin/tmux new-window -t services:3 -n emorec 'bash -c "roslaunch --wait emorec server.launch"'
sleep 2
/usr/bin/tmux new-window -t services:4 -n objectrec 'bash -c "roslaunch --wait objectrec server.launch"'
sleep 2
/usr/bin/tmux new-window -t services:5 -n concept_space 'bash -c "roslaunch --wait concept_net concept_space_server.launch"'
sleep 2
/usr/bin/tmux new-window -t services:6 -n semantic_fusion 'bash -c "roslaunch --wait concept_net fusion_server.launch"'
sleep 2
/usr/bin/tmux new-window -t services:7 -n semantic_similarity 'bash -c "roslaunch --wait concept_net semantic_similarity_server.launch"'
