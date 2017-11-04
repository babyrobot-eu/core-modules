#!/bin/bash

tmux new-session -d -s roscore "roscore"
tmux new-session -d -s objectrec "roslaunch speech_features server.launch"
tmux new-session -d -s objectrec "roslaunch emorec server.launch"
tmux new-session -d -s objectrec "roslaunch objectrec server.launch"
