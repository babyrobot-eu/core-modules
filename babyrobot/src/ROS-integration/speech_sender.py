#!/usr/bin/env python

import rospy
import socket
import sys
from std_msgs.msg import String
import numpy as np
import ast
import time 

translation_dict = {
    'paizwKithara': "Playing the guitar",
    'bafwToixo': "Painting the wall",
    "tzamia": "Cleaning the window",
    "odhghsh": "Driving the bus",
    "skabwTrypa": "Digging a hole",
    "skoupizwPatwma": "Wiping the floor",
    "kolympi": "Swimming",
    "xoreywXoro": "Dancing",
    "gymanzomai": "Working out",
    "siderwma": "Ironing a shirt",
    "karfwma": "Hammering a nail",
    "gyrnawSelides": "Reading a book",
    "nai": "Yes",
    "oxi": "No",
    "papia": "I think it's the duck",
    "kyknos": "I think it's the swan",
    "kota": "I think it's the chicken",
    "peristeri": "I think it's the pidgeon",
    "galopoyla": "I think it's the turkey",
    "pagoni": "I think it's the peacock",
    "cari": "I thinks it's the fish",
    "batraxos": "I think it's the toad",
    "gata": "I think its the cat",
    "skylos": "I think its the dog",
    "koyneli": "I think its the rabbit",
    "skioyros": "I think its the squirrel",
    "skantzoxoiros": "I think its the hedgehod",
    "goyroyni": "I think its the pig",
    "probato": "I think its the sheep",
    "katsika": "I think its the goat",
    "agelada": "I think its the cow",
    "gaidaros": "I think its the donkey",
    "alogo": "I think its the horse",
    "megalo": "It's a big size animal",
    "mesaio": "It's a medium size animal",
    "mikro": "It's a small size animal",
    "pthna": "It is a bird",
    "amfibia": "It is an amphibian animal",
    "uhlastika": "It is a mammal",
    "caria": "It is a fish",
    "kitrino": "It is yellow",
    "aspro": "It is white",
    "mayro": "It is black",
    "kafe": "It is brown",
    "gkri": "It is gray",
    "polyxrwmo": "It is multicolored",
    "roz": "It is pink",
    "prasino": "It is green",
    "podia4": "It has 4 legs",
    "podia2": "It has 2 legs",
    "podia0": "It has 0 legs",
    "laimo": "It has a long neck",
    "eirhnh": "It reminds us of peace",
    "xmas": "It reminds us of christmas",
    "entypOura": "It has an impressive tail",
    "gyala": "It also lives in a glass bowl",
    "prigkipissa": "It is kissed by a pricess",
    "spiti": "It also lives in a home",
    "filosAnur": "It is a man's best friend",
    "karota": "It eats carrots",
    "fountOyra": "It has a bushy tail",
    "agkauia": "It has spines",
    "trweiPanta": "It eats everything",
    "malli": "It gives us wool",
    "braxia": "It jumps on rocks",
    "mhrykastiko": "It is a ruminant",
    "anPetaei": "They ask if it flies",
    "kalpasmo": "It is known for its gallop",
    "ferxat": "I want to play with furhat",
    "nao": "I want to play with nao",
    "zeno": "I want to play with zeno"
}


##### customizable parameters #####

ticket_name = 'furhat'
iristk_name = 'asr_system' ###### for iristk only 
topic_name = '/visualization_cmds_audio'
server_ip = '192.168.0.105'
port = 1932

###################################

json_format = "{ \"class\": \"iristk.system.Event\", \"event_sender\": \"asr_system\" , \"event_name\": \"athena.asr.recognized\", \"text\": \"%s\" }\n"
event_format = "EVENT athena.asr.recognized %d\n"
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

send_gesture = False

listening = False
check_buffer = False
def_y = False
options = "nai"

buff = []

def callback(data):
    global listening, send_gesture, buff, check_buffer, options, def_y
    if listening:
        gesture = get_gesture_from_visualization(data)

    # rospy.loginfo(check_buffer)
    if check_buffer and len(buff) > 0:
        lx_buff = [x for x in buff if x in options]
        l_buff = [x for x in lx_buff if x != "mhrykastiko"]
        if len(l_buff) > 0:
            gesture = l_buff[-1]
        else:
            gesture = 'listen_again'

        if gesture == "ferxat":
            gesture = "furhat"
        elif gesture == "abatar":
            gesture = "avatar"
        elif gesture == "zino":
            gesture = "zeno"

        if "xorodia" in options:
            gesture = "xoreywXoro"
        if "vivlio" in options:
            gesture = "gyrnawSelides"

        try:
            # translated_buffer = [translation_dict[x] for x in buff]
            # print "*******************Selected: ", gesture, "from buffer: ", buff
            rospy.loginfo("Recognized utterance **{}**".format(translation_dict[gesture]))
        except Exception as e:
            pass

        j =  json_format % gesture
        full_event = event_format % len(j)
        sock.sendall(full_event)
        sock.sendall(j)
        listening = False
        check_buffer = False
        buff = []

    if check_buffer and len(buff) == 0:
        if "xorodia" in options:
            gesture = "xoreywXoro"
        elif "vivlio" in options:
            gesture = "gyrnawSelides"
        else:
            if def_y:
                gesture = 'nai'
                rospy.loginfo("Recognized utterance **{}**".format(translation_dict[gesture]))
            else:
                gesture = 'listen_again'
                rospy.loginfo("No utterance recognized")

        j =  json_format % gesture
        full_event = event_format % len(j)
        sock.sendall(full_event)
        sock.sendall(j)
        listening = False
        check_buffer = False
        buff = []



def callback_listen(data):
    global listening, check_buffer, options, listen_for, def_y
    if listening:
        return
    options = data.data.split("-")[0].split(",")
    listen_for = int(data.data.split("-")[1])
    timeout = int(data.data.split("-")[2])
    def_y = int(data.data.split("-")[3])
    if def_y == 1:
        def_y = True
    else:
        def_y = False
    # rospy.loginfo("DSR Module Sleeping for %d seconds" % timeout)
    time.sleep(timeout)
    listening = True
    # print "START LISTENING"
    rospy.loginfo("DSR Module Started Listening for %d seconds" % listen_for)
    time.sleep(listen_for)
    rospy.loginfo("DSR Module Ended Listening")
    listening = False
    check_buffer = True
    callback(None)



def get_gesture_from_visualization(msg):
    global send_gesture, buff, options
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
        # print(np_gest[m])
        if np_gest[m].strip() in options:
            # try:            
            #     rospy.loginfo("DSR Listened: **{}**".format(translation_dict[np_gest[m]]))
            # except Exception as e:
            #     pass
            # print "***************** Listened: ", np_gest[m]
            buff.append(np_gest[m])
        # return np_gest[m]


def gesture_sender():
    rospy.init_node('asr', anonymous=True)

    rospy.Subscriber(topic_name, String, callback)

    rospy.Subscriber('athena_asr', String, callback_listen)

    rospy.loginfo("Distant Speech Rec. Module is connected and ready")

    rospy.spin()


if __name__ == '__main__':
    sock.connect((server_ip, port))

    msg = 'CONNECT %s %s \n' % (ticket_name, iristk_name)
    
    sock.send(msg)

    # the number 10 is just a random thing

    data = sock.recv(10)

    sock.sendall('SUBSCRIBE athena.asr.** \n')

    # print "SUCCESSFULLY SUBSCRIBED TO BROKER ASR"

    gesture_sender()
