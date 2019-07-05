#!/usr/bin/env python

import rospy
import socket
import sys
from std_msgs.msg import String
import numpy as np
import ast
import time 


##### customizable parameters #####

ticket_name = 'furhat'
iristk_name = 'emotion_back_sender' ###### for iristk only 
server_ip = '192.168.0.105'
port = 1932

###################################

json_format = "{ \"class\": \"iristk.system.Event\", \"event_sender\": \"emotion_back_sender\" , \"event_name\": \"athena.emotion.recognized_english\", \"text\": \"%s\" }\n"
json_format_greek = "{ \"class\": \"iristk.system.Event\", \"event_sender\": \"emotion_back_sender\" , \"event_name\": \"athena.emotion.recognized_greek\", \"text\": \"%s\" }\n"
event_format = "EVENT athena.emotion.recognized_english %d\n"
event_format_greek = "EVENT athena.emotion.recognized_greek %d\n"
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

listening = False

buff = []

def callback(data):
    global listening
    if listening:
        emotion_english = data.split("-")[0]
        emotion_greek = data.split("-")[1]

        j =  json_format_greek % emotion_greek
        full_event = event_format_greek % len(j)
        sock.sendall(full_event)
        sock.sendall(j)


        j =  json_format % emotion_english
        full_event = event_format % len(j)
        sock.sendall(full_event)
        sock.sendall(j)
        listening = False

        rospy.loginfo("Emotion Module Ended Listening")

def callback_listen(data):
    global listening, check_buffer
    if listening:
        return

    # listen_for = 5

    time.sleep(0.5)
    listening = True
    rospy.loginfo("Emotion Module Started Listening" % listen_for)
    # time.sleep(listen_for)
    # listening = False
    # check_buffer = True
    # callback(None)



def gesture_sender():
    rospy.init_node('emotion_sender', anonymous=True)

    rospy.Subscriber("/emotion_topic", String, callback)

    rospy.Subscriber("/athena_emotion", String, callback_listen)

    rospy.loginfo("Emotion Sender Module is connected and ready")

    rospy.spin()


if __name__ == '__main__':
    sock.connect((server_ip, port))

    msg = 'CONNECT %s %s \n' % (ticket_name, iristk_name)
    
    sock.send(msg)

    # the number 10 is just a random thing

    data = sock.recv(10)

    sock.sendall('SUBSCRIBE athena.emotion.** \n')

    gesture_sender()
