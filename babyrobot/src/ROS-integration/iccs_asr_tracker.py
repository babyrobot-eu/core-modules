#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import socket
import sys
from std_msgs.msg import String
import numpy as np
import time
import json
##### customizable parameters #####
ticket_name = 'furhat'
iristk_name = 'iccs_timings'

broker_ip = "192.168.0.105"
port = 1932
socket_read_buffer_size = 1024
###################################

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
read_next_line = False

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

def asr_timing_handler():
    global read_next_line
    # Define Publisher
    topic_name = 'iccs/states'
    pub = rospy.Publisher(topic_name, String, queue_size=10)
    
    node_name = 'iccs/guesstheobject/state'
    rospy.init_node('iccs_timings', anonymous=True)
    rospy.loginfo("Continuous Timings node is connected and ready")

    while not rospy.is_shutdown():
        for line in readlines(sock):
            if read_next_line:
                j = json.loads(line)
                event_name = j["event_name"]
                text = j["text"]
                read_next_line = False

            # Read event which contains text
            # if line.startswith("EVENT iccs.play") or line.startswith("EVENT iccs.gender"):
            #     read_next_line = True
            # Read event
            if line.startswith("EVENT iccs.game.start"):
                pub.publish("start iccs guesstheobject")
                rospy.loginfo("GuessTheObject GAME START")
            # if line.startswith("EVENT iccs.asr.listen"):
            #     pub.publish('asr.listen')
            #     rospy.loginfo("Sending LISTEN event to CONTINUOUS ASR module.")
            if line.startswith("EVENT iccs.system.state"):
                pub.publish(line)
                # rospy.loginfo("Sending LISTEN event to CONTINUOUS ASR module.")


if __name__ == '__main__':
    sock.connect((broker_ip, port))

    msg = 'CONNECT %s %s \n' % (ticket_name, iristk_name)
    
    sock.send(msg)

    # the number 10 is just a random thing

    data = sock.recv(10)

    sock.sendall('SUBSCRIBE iccs.** \n')


    asr_timing_handler()
