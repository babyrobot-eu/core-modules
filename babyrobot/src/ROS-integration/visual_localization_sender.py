#!/usr/bin/env python

import rospy
import socket
import sys
from std_msgs.msg import String
import numpy as np
import ast
from kinect_parser.msg import JointOrientationAndType, JointPositionAndState, Lean, Body, BodyArray, Face
from geometry_msgs.msg import Point
import time

##### customizable parameters #####
ticket_name = 'furhat' # ticket name
iristk_name = 'sloc_system' # name of the system in iristk 
topic_name = '/head/kinect2/k2_bodies/bodies' # topic where bodies are published
server_ip = '192.168.0.105' # broker ip
port = 1932 # broker port
###################################

json_format = "{ \"class\": \"iristk.system.Event\", \"event_sender\": \"sloc_system\" , \"event_name\": \"athena.agent.attend\", \"x\": %f, \"y\": %f, \"z\": %f }\n"
event_format = "EVENT athena.agent.attend %d\n"
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

start_time = 0

def callback(data):
    global start_time
    if len(data.bodies) > 0:
        head = data.bodies[0].jointPositions[3]
        print head



def visual_loc_sender():
    rospy.init_node('visual_loc_sender', anonymous=True)

    rospy.Subscriber(topic_name, BodyArray, callback)

    rospy.spin()


if __name__ == '__main__':
#    sock.connect((server_ip, port))

 #   msg = 'CONNECT %s %s \n' % (ticket_name, iristk_name)
    
  #  sock.send(msg)

    # the number 10 is just a random thing

   # data = sock.recv(10)

    #sock.sendall('SUBSCRIBE athena.sloc.** \n')

    visual_loc_sender()
