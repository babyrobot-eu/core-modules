#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import socket
import sys
from std_msgs.msg import String
import numpy as np
import time
import json
import roslaunch

##### customizable parameters #####
ticket_name = 'furhat'
iristk_name = 'emotion_launch_node'
broker_ip = "192.168.0.105"
port = 1932
socket_read_buffer_size = 1024
###################################

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
read_next_line = False
read_next_line_and_run = False
read_next_line_and_stop = False

def readlines(sock, recv_buffer=4096, delim='\n'):
    buffer = ''
    data = True
    while data:
        data = sock.recv(recv_buffer)
        buffer += data

        while buffer.find(delim) != -1:
            line, buffer = buffer.split('\n', 1)
            yield line
    return


def gesture_timing_handler():
    global read_next_line_and_run, read_next_line_and_stop
    # gestures by our system

    # launch_dict = {} 

    rospy.init_node('emotion_launch_launcher', anonymous=True)
    rospy.loginfo("Emotion launch launcher node is connected and ready")

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    launch = None

    while not rospy.is_shutdown():
        for line in readlines(sock):
            if read_next_line_and_run:
                #j = json.loads(line)
                #launch_file = j["text"]
                launch_file = "/home/kinect/indigo_ws/src/kinect_parser/launch/emotion.launch"
                if launch == None:         
                    rospy.loginfo("Launching %s"% launch_file)

                    launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_file])
                    launch.start()

                    rospy.loginfo("%s has been successfully launched" % launch_file)
                    read_next_line_and_run = False

            if line.startswith("EVENT athena.launch_emotion.launch"):
                read_next_line_and_run = True

            if line.startswith("EVENT athena.launch_emotion.stop"):
                try:
                    launch.shutdown()
                    launch = None
                except Exception as e:
                    print e


if __name__ == '__main__':
    sock.connect((broker_ip, port))

    msg = 'CONNECT %s %s\n' % (ticket_name, iristk_name)
    
    sock.send(msg)

    data = sock.recv(socket_read_buffer_size)

    sock.sendall('SUBSCRIBE athena.** \n')

    gesture_timing_handler()
