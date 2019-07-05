#!/usr/bin/env python

import rospy
import socket
import sys
from std_msgs.msg import String
import numpy as np
import ast

##### customizable parameters #####

ticket_name = 'furhat'
iristk_name = 'sloc_system' ###### for iristk only 
topic_name = '/localization_topic'
server_ip = '192.168.0.105'
port = 1932

###################################

json_format = "{ \"class\": \"iristk.system.Event\", \"event_sender\": \"sloc_system\" , \"event_name\": \"athena.agent.attend\", \"x\": %f, \"y\": %f, \"z\": %f }\n"
event_format = "EVENT athena.agent.attend %d\n"
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

send_sloc = False
listening = False
buffer_x = []
buffer_y = []
buffer_z = []

def callback(data):
    global send_sloc,buffer_x,buffer_y,buffer_z, listening

    if listening:
        data = str(data).split()
        x = float(data[1])
        y = float(data[2])
        z = float(data[3])
        buffer_x.append(x)
        buffer_y.append(y)
        buffer_z.append(z)

    if send_sloc:
        j = json_format % (np.mean(np.asarray(buffer_x),np.asarray(buffer_y),np.asarray(buffer_z)))
        # len is to get bytes in python 2.X
        full_event = event_format % len(j)
        print j
        sock.sendall(full_event)
        sock.sendall(j)
        send_gesture = False


def callback_listen(data):
    global send_sloc, buffer_x, buffer_y, buffer_z
    if listening:
        return
    options = data.data.split(",")
    # time.sleep(3)
    listening = True
    print "START LISTENING"
    time.sleep(5)
    print "END LISTENING"
    listening = False
    send_sloc = True


def gesture_sender():
    rospy.init_node('slocs', anonymous=True)

    rospy.Subscriber(topic_name, String, callback)

    rospy.spin()


if __name__ == '__main__':
    sock.connect((server_ip, port))

    msg = 'CONNECT %s %s \n' % (ticket_name, iristk_name)
    
    sock.send(msg)

    # the number 10 is just a random thing

    data = sock.recv(10)

    sock.sendall('SUBSCRIBE athena.sloc.** \n')

    gesture_sender()
