#!/usr/bin/env python

import rospy
import socket
import sys
from std_msgs.msg import String
import numpy as np
import ast



gesture_dict = {
    "dancing": "xoros1",
    "painting_a_wall" : "vafo1",
    "digging_a_hole" : "trypa1",
    "cleaning_a_window": "tzamia1",
    "ironing_a_shirt": "siderono1",
    "hammering_a_nail": "karfono1",
    "wiping_the_floor": "skoupizeis1",
    "driving_a_bus": "leoforeio1",
    "reading": "vivlio1",
    "playing_the_guitar": "kithara1",
    "swimming": "kolympi1",
    "working_out": "gym1",
    "BM": "none",
    "greetings": "greetings",
    "come_closer": "come_closer",
    "point": "point",
    "sit_down": "sit_down",
    "stop": "stop",
    "circle": "circle",
    "none": "none",
    "agreement": "agreement"
}

_cl = {
    "xoros1": "Action",
    "vafo1": "Action",
    "trypa1": "Action",
    "tzamia1": "Action",
    "siderono1": "Action",
    "karfono1": "Action",
    "skoupizeis1": "Action",
    "leoforeio1": "Action",
    "vivlio1": "Action",
    "kithara1": "Action",
    "kolympi1": "Action",
    "gym1": "Action",
    "BM": "",
    "greetings": "Gesture",
    "come_closer": "Gesture",
    "point": "Gesture",
    "sit_down": "Gesture",
    "stop": "Gesture",
    "circle": "Gesture",
    "none": "",
    "agreement": "Gesture"    
}

translation_dic = {
    "xoros1": "Dancing",
    "vafo1": "Painting a wall",
    "trypa1": "Digging a hole",
    "tzamia1": "Cleaning a window",
    "siderono1": "Ironing a shirt",
    "karfono1": "Hammering a nail",
    "skoupizeis1": "Wiping the floor",
    "leoforeio1": "Driving a bus",
    "vivlio1": "Reading a book",
    "kithara1": "Playing the guitar",
    "kolympi1": "Swimming",
    "gym1": "Working out",
    "BM": "none",
    "greetings": "greetings",
    "come_closer": "come_closer",
    "point": "point",
    "sit_down": "sit_down",
    "stop": "stop",
    "circle": "circle",
    "none": "none",
    "agreement": "agreement"
}
        


##### customizable parameters #####

ticket_name = 'furhat'
iristk_name = 'gesture_system' ###### for iristk only 
topic_name = '/visualization_cmds'
server_ip = '192.168.0.105'
port = 1932

###################################

json_format = "{ \"class\": \"iristk.system.Event\", \"event_sender\": \"gesture_system\" , \"event_name\": \"athena.gesture.recognized\", \"text\": \"%s\" }\n"
event_format = "EVENT athena.gesture.recognized %d\n"
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

send_gesture = False

import time

# previous = time.time()

def callback(data):
    #print data
    global send_gesture, previous
    # current = time.time()

    # if current - previous < 3:
    #     return
    # previous = current

    gesture = get_gesture_from_visualization(data)
    if send_gesture:
        j =  json_format % gesture
        # len is to get bytes in python 2.X
        full_event = event_format % len(j)
        sock.sendall(full_event)
        sock.sendall(j)
        send_gesture = False


# def get_gesture_from_topic_msg(msg):
#     msg_array = str(msg).split()
#     print msg

#     # the 0th place is a placeholder containing the string 'data:'
#     nr = msg_array[1]
#     timestamp_start = msg_array[2]
#     timestamp_end = msg_array[3]

#     probabilities = []
#     gesture_names = []

#     for i in range(4, len(msg_array), 2):
#         gesture_names.append(msg_array[i])
#         probabilities.append(float(msg_array[i+1]))

#     np_gest = np.asarray(gesture_names)
#     np_prob = np.asarray(probabilities)

#     m = np.argmax(np_prob)
#     print np_gest[m]
#     return np_gest[m]


def get_gesture_from_visualization(msg):
    global send_gesture
    if send_gesture:
        return
    msg_array = ast.literal_eval(msg.data)
    #print type(msg_array)
    #print msg_array['modalityID']
    if msg_array['modalityID']==4:
        send_gesture = True
        gesture_names = msg_array['hypotheses']
        probabilities = msg_array['scores']
        np_gest = np.asarray(gesture_names)
        np_prob = np.asarray(probabilities)
        m = np.argmax(np_prob)
        # print "*****************RECOGNIZED: ", gesture_dict[np_gest[m]]
        try:
            rospy.loginfo("Recognized {}: **{}**".format(_cl[gesture_dict[np_gest[m]]], translation_dic[gesture_dict[np_gest[m]]]))
        except Exception as e:
            pass
        return gesture_dict[np_gest[m]]

def gesture_sender():
    rospy.init_node('gestures', anonymous=True)

    rospy.Subscriber(topic_name, String, callback)

    rospy.loginfo("Gesture and Action Perception Module is connected and ready")

    rospy.spin()


if __name__ == '__main__':
    sock.connect((server_ip, port))

    msg = 'CONNECT %s %s \n' % (ticket_name, iristk_name)
    
    sock.send(msg)

    # the number 10 is just a random thing

    data = sock.recv(10)

    sock.sendall('SUBSCRIBE athena.gesture.** \n')

    # print "SUCCESSFULLY SUBSCRIBED TO BROKER GESTURE SENDER"

    gesture_sender()
