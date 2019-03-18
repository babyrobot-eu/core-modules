#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import socket
import sys
import numpy as np
from kinect_parser.msg import JointOrientationAndType, JointPositionAndState, Lean, Body, BodyArray, Face
from std_msgs.msg import String
from geometry_msgs.msg import Point

##### customizable parameters #####

ticket_name = 'furhat'
iristk_name = 'rpc_game' ###### for iristk only 
topic_name = '/head/kinect2/k2_bodies/bodies'
head_topic_name = '/head/kinect2/k2_faces/faces'
timings_topic_name = '/kinect_gesture_timings'
server_ip = '192.168.0.120'
port = 1932

###################################

json_format = """{ \"class\": \"iristk.system.Event\", \"event_name\": \"athena.games.rpc.score\", \"winner\": \"%s\", \"score_1\": %d, \"score_2\": %d, \ 
   \"user_1_x\": %f, \"user_1_y\": %f, \"user_1_z\": %f, \"user_2_x\": %f, \"user_2_y\": %f, \"user_2_z\": %f }\n"""
event_format = "EVENT athena.games.rpc.score %d\n"
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

listening = True

body_1_track_id = None
body_2_track_id = None
body_1_pivot = Point()
body_2_pivot = Point()


def callback_timings(data):
    global listening
    listening = True

def callback(data):
    global listening, score_1, score_2, body_1_track_id, body_2_track_id, body_1_pivot, body_2_pivot
    if not listening:
        return
    else:
        if len(data.bodies) >= 2:
            body_1_track_id = data.bodies[0].trackingId
            body_2_track_id = data.bodies[1].trackingId
            gesture_1 = get_game_gesture(data.bodies[0])
            gesture_2 = get_game_gesture(data.bodies[1])

            if gesture_1 == None or gesture_2 == None:
                return

            if gesture_1 == "Paper" and gesture_2 == "Rock":
                score_1 += 1
                winner = "first"
            if gesture_1 == "Rock" and gesture_2 == "Paper":
                score_2 += 1
                winner = "second"

            if gesture_1 == "Paper" and gesture_2 == "Paper":
                pass
            if gesture_1 == "Rock" and gesture_2 == "Rock":
                pass
            if gesture_1 == "Pen" and gesture_2 == "Pen":
                pass

            if gesture_1 == "Paper" and gesture_2 == "Pen":
                score_2 += 1
                winner = "second"
            if gesture_1 == "Pen" and gesture_2 == "Paper":
                score_1 += 1
                winner = "first"

            if gesture_1 == "Pen" and gesture_2 == "Rock":
                score_2 += 1
                winner = "second"
            if gesture_1 == "Rock" and gesture_2 == "Pen":
                score_1 += 1
                winner = "first"

            print winner

            j =  json_format % (winner, score_1, score_2, body_1_pivot.x, body_1_pivot.y, body_1_pivot.z, body_2_pivot.x, body_2_pivot.y, body_2_pivot.z)
            # len is to get bytes in python 2.X
            full_event = event_format % len(j)
            sock.sendall(full_event)
            sock.sendall(j)
            listening = False


def callback_face(data):
    if data.trackingId == body_1_track_id:
        body_1_pivot = data.headPivotPoint
    elif data.trackingId == body_2_track_id:
        body_2_pivot = data.headPivotPoint


def get_game_gesture(body):
    right_hand_conf = body.handRightConfidence
    left_hand_conf = body.handLeftConfidence
    right_hand_state = body.handRightState
    right_hand_state = body.handLeftState
    
    if right_hand_conf == "1" and right_hand_state =="2":
        return "Paper"

    if right_hand_conf == "1" and right_hand_state =="3":
        return "Rock"

    if left_hand_conf == "1" and left_hand_state =="2":
        return "Paper"

    if left_hand_conf == "1" and left_hand_state =="3":
        return "Rock"

    if left_hand_conf == "1" and left_hand_state =="4":
        return "Pen"

    if right_hand_conf == "1" and right_hand_state =="4":
        return "Pen"

    # if right_hand_conf == "1" and right_hand_state =="4" and left_hand_conf == "1" and left_hand_state =="4":
    #     return "BothPen"

    return None

def gesture_sender():
    rospy.init_node('kinect_gestures', anonymous=True)

    rospy.Subscriber(topic_name, BodyArray, callback)
    rospy.Subscriber(head_topic_name, Face, callback_face)
    rospy.Subscriber(timings_topic_name, String, callback_timings)

    rospy.spin()


if __name__ == '__main__':
    sock.connect((server_ip, port))

    msg = 'CONNECT %s %s\n' % (ticket_name, iristk_name)
    
    sock.send(msg)

    data = sock.recv(10)

    sock.sendall('SUBSCRIBE athena.kinect_gesture.listen\n')
    print "SUCCESSFULLY SUBSCRIBED TO BROKER SKELETON SENDER"
    gesture_sender()
