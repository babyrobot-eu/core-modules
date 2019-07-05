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
iristk_name = 'sensor_timings'
gesture_topic_name = 'gesture_timings'
kinect_gesture_topic_name = 'kinect_gesture_timings'
audio_topic_name = 'athena_asr'
sloc_topic_name = 'athena_sloc'
emotion_topic_name = 'athena_emotion'
broker_ip = "192.168.0.145"
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

def gesture_timing_handler():
    global read_next_line
    # gestures by our system
    pub = rospy.Publisher(gesture_topic_name, String, queue_size=10)
    # gestures by kinect
    pub_kin = rospy.Publisher(kinect_gesture_topic_name, String, queue_size=10)
    # gestures asr
    pub_asr = rospy.Publisher(audio_topic_name, String, queue_size=10)

    pub_sloc = rospy.Publisher(sloc_topic_name, String, queue_size=10)


    pub_emotion = rospy.Publisher(emotion_topic_name, String, queue_size=10)

    rospy.init_node('gesture_timings', anonymous=True)
    rospy.loginfo("Timings node is connected and ready")

    while not rospy.is_shutdown():
        for line in readlines(sock):
            if read_next_line:
                def_y = 0    
                j = json.loads(line)
                if "nai" in j["options"] and (not "game" in j): # pantomima nai
                    def_y = 1
                    listen_for = 5
                    timeout = 1
                elif "nai" in j["options"]: # farma nai
                    listen_for = 5
                    timeout = 1
                elif "siderwma" in j["options"]:
                    listen_for = 5
                    timeout = 2
                elif "papia" in j["options"] or "megalo" in j["options"]:
                    timeout = 1
                    listen_for = 6
                else:
                    listen_for = 5
                    timeout = 2

                pub_asr.publish("{options}-{time}-{timeout}-{def_y}".format(options=j["options"], time=listen_for, timeout=timeout, def_y=def_y))

                read_next_line = False

                rospy.loginfo("Sending LISTEN event to ASR module")
                # print j["options"]

            if line.startswith("EVENT athena.emotion.listen"):
                pub_emotion.publish("Start EMOTION")
                rospy.loginfo("Sending LISTEN event to EMOTION module.")

            if line.startswith("EVENT athena.asr.listen"):
                read_next_line = True

            if line.startswith("EVENT athena.gesture.listen"):
                pub.publish("Start GESTURE")
                # print("Sending GESTURE listen")
                rospy.loginfo("Sending LISTEN event to GESTURE module.")

            if line.startswith("EVENT athena.pantomime.listen"):
                pub.publish("Start PANTOMIME")
                rospy.loginfo("Sending LISTEN event to ACTION module.")
                # print "Sending PANTOMIME listen"

            if line.startswith("EVENT athena.sloc.listen"):
                pub_sloc.publish("Start")
                # print "Sending SLOC listen"
                rospy.loginfo("Sending LISTEN event to SLOC module.")

                # if len(data) > 2:
                #     if data[1] != None and (data[1] == "athena.gesture.listen"):
                #         pub.publish("Start")
                #     if data[1] != None and (data[1] == "athena.kinect_gesture.listen"):
                #         pub_kin.publish("Start")
                #     if data[1] != None and (data[1] == "athena.asr.listen"):
                #         print data
                #         pub_asr.publish("Start")

        # data = sock.recv(socket_read_buffer_size)
        # if data != None:
        #     data = data.split()
        #     if len(data) > 2:
        #         if data[1] != None and (data[1] == "athena.gesture.listen"):
        #             pub.publish("Start")
        #         if data[1] != None and (data[1] == "athena.kinect_gesture.listen"):
        #             pub_kin.publish("Start")
        #         if data[1] != None and (data[1] == "athena.asr.listen"):
        #             print data
        #             pub_asr.publish("Start")


if __name__ == '__main__':
    sock.connect((broker_ip, port))

    msg = 'CONNECT %s %s\n' % (ticket_name, iristk_name)
    
    sock.send(msg)

    data = sock.recv(socket_read_buffer_size)

    sock.sendall('SUBSCRIBE athena.** \n')

    # print "SUCCESSFULLY SUBSCRIBED TO BROKER TIMINGS"

    gesture_timing_handler()
