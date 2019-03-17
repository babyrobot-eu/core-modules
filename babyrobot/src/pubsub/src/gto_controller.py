#!/usr/bin/env python
# coding=utf-8

import rospy
import socket
import sys
import numpy as np
import ast

from babyrobot_msgs.msg import TimedString
from std_msgs.msg import String
import time


class IrisTK_Bridge(object):
    """docstring for IrisTK_Bridge"""
    def __init__(self, broker_ip, broker_port, ticket_name, iristk_name, ros_node_name):
        super(IrisTK_Bridge, self).__init__()
        self.ticket_name = ticket_name
        self.iristk_name = iristk_name
        self.ros_node_name = ros_node_name
        self.sent_object_found = False
        self.object_found = False
        self.audio_record_states = [
            'iccs.system.state.intro',
            'iccs.system.state.playmaybe',
            'iccs.system.state.gendermale',
            'iccs.system.state.first.firstproperty',
            'iccs.system.state.first.objectrecshow',
            'iccs.system.state.replayask',
            'iccs.system.state.second.firstproperty',
            'iccs.system.state.second.wrong1',
            'iccs.system.state.second.similar',
            'iccs.system.state.help.spatial_left',
            'iccs.system.state.third.firstproperty',
            'iccs.system.state.third.noresponse',
            'iccs.system.state.third.secondproperty',
            'iccs.system.state.second.confused'
        ]
        self.broker_ip = broker_ip
        self.broker_port = broker_port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.connect_to_broker()
        self.current_state = ''
        self.current_object = 'first'
        self.control_pub = rospy.Publisher("/iccs/gto/controller_commands", String, queue_size=10)
        rospy.init_node(self.ros_node_name, anonymous=True)
        rospy.Subscriber('/iccs/states', String, self.handle_states)
        rospy.Subscriber('/iccs/objects', String, self.handle_objects)
        rospy.Subscriber('/iccs/translate', TimedString, self.handle_asr)
        self.counter_pen = 0
        self.counter_notebook = 0
        self.counter_confused = 0
        self.counter_book = 0
        self.counter_replay = 0 

    def handle_asr(self, asr_out):
        asr = asr_out.data.lower()
        if (self.current_state == 'iccs.system.state.third.firstproperty' and
           len(asr) == 0):
            self.send_message('iccs.third.noresponse', '')
        # Wait for YES/NO
        if self.current_state == 'iccs.system.state.intro':
            self.counter_pen = 0
            self.counter_notebook = 0
            self.counter_confused = 0
            self.counter_book = 0
            self.counter_replay = 0 

        if (self.current_state == 'iccs.system.state.intro' or
           self.current_state == 'iccs.system.state.playmaybe'):
            if 'yes' in asr or 'okay' in asr or ('let' in asr and 'play' in asr):
                self.send_message('iccs.play', 'confirm')
            else:
                self.send_message('iccs.play', 'maybe')
        if self.current_state == 'iccs.system.state.gendermale':
            if ('yes' in asr or
               'ok' in asr or
               'okay' in asr or
               'ready' in asr or
               ('let' in asr and 'play' in asr)):
                self.send_message('iccs.ready', 'yes')
            else:
                self.send_message('iccs.asrkaput.gendermale', '')
        if self.current_state == 'iccs.system.state.first.objectrecshow':
            if 'yes' in asr or 'correct' in asr or 'right' in asr or 'ball' in asr:
                self.send_message('iccs.first.objectreccorrect', '')
            else:
                self.send_message('iccs.asrkaput.first.objectrecshow', '')
        if self.current_state == 'iccs.system.state.replayask':
            if self.counter_replay > 0:
                asr = 'yes I want to'
            if 'want' in asr or 'Yes' in asr or 'yes' in asr or 'ok' in asr or 'okay' in asr or 'play' in asr:
                self.send_message('iccs.replay.yes', '')
            else:
                self.send_message('iccs.asrkaput.replayask', '')
                self.counter_replay += 1
        if self.current_state == 'iccs.system.state.second.firstproperty':
            if self.counter_pen > 0:
               asr = 'I think it is the pen'
            if 'pen' in asr or 'notebook' in asr:
                self.send_message('iccs.second.wrong1', '')
            else:
                self.send_message('iccs.asrkaput.second.firstproperty', '')
                self.counter_pen += 1
 
        if self.current_state == 'iccs.system.state.second.wrong1':
            if self.counter_notebook > 0:
                asr = 'I think it is the notebook'
            if 'pen' in asr or 'notebook' in asr:
                self.send_message('iccs.second.similar', '')
            else:
                self.send_message('iccs.asrkaput.second.wrong1', '')
                self.counter_notebook += 1

        if self.current_state == 'iccs.system.state.second.similar':
            if self.counter_confused > 0:
                asr = 'I don\'t know. It is hard'
            if 'know' in asr or 'choose' in asr or 'conf' in asr or 'hard' in asr or 'don' in asr:
                self.send_message('iccs.second.confused', '')
            else:
                self.send_message('iccs.asrkaput.second.similar', '')
                self.counter_confused = 0

        if self.current_state == 'iccs.system.state.second.confused':
            if self.counter_book > 0:
                asr = 'I think it\'s the book'
            if 'book' in asr or 'think' in asr:
                self.send_message('iccs.second.correct', '')
            else:
                self.send_message('iccs.asrkaput.second.confused', '')
                self.counter_book = 0

        if self.current_state == 'iccs.system.state.third.noresponse':
            if 'yes' in asr or 'want' in asr:
                self.send_message('iccs.third.secondproperty', '')
            else:
                self.send_message('iccs.asrkaput', '')

        if self.current_state == 'iccs.system.state.third.secondproperty':
            if 'plane' in asr:
                self.send_message('iccs.third.correct', '')
            else:
                self.send_message('iccs.asrkaput', '')
 
    def handle_objects(self, found_objects):
        if self.current_state == 'iccs.system.state.first.firstproperty':
            if self.object_found and not self.sent_object_found:
                rospy.loginfo('sent iccs.first.objectrecshow')
                self.send_message('iccs.first.objectrecshow', '')
                self.sent_object_found = True
            
            if 'sports ball' in found_objects.data.split(','):
                self.object_found = True
        else:
            self.object_found = False
            self.sent_object_found = False

    def handle_states(self, state):
        self.current_state = state.data.split(' ')[1]
        if self.current_state in self.audio_record_states:
            self.control_pub.publish('asr.listen.start')
        if self.current_state == 'iccs.system.state.playconfirm':
            self.send_message('iccs.gender', 'male')
        if self.current_state == 'iccs.system.state.readyyes':
            self.send_message('iccs.first.start', '')
        if self.current_state == 'iccs.system.state.first.start':
            self.send_message('iccs.first.firstproperty', '')
        if self.current_state == 'iccs.system.state.first.objectreccorrect':
            self.send_message('iccs.replay.ask', '')
            self.current_object = 'second'
        if self.current_state == 'iccs.system.state.replayyes':
            if self.current_object == 'second':
                self.send_message('iccs.second.start', '')
            if self.current_object == 'third':
                self.send_message('iccs.third.start', '')
        if self.current_state == 'iccs.system.state.second.start':
            self.send_message('iccs.second.firstproperty', '')
        if self.current_state == 'iccs.system.state.second.correct':
            self.send_message('iccs.replay.ask', '')
            self.current_object = 'third'
        if self.current_state == 'iccs.system.state.third.start':
            self.send_message('iccs.third.firstproperty', '')
        if self.current_state == 'iccs.system.state.third.correct':
            self.send_message('iccs.replay.ask', '')
        if self.current_state == 'iccs.system.state.replayno':
            self.send_message('iccs.quit.end', '')

    def connect_to_broker(self):
        self.sock.connect((self.broker_ip, self.broker_port))

        msg = 'CONNECT %s %s \n' % (self.ticket_name, self.iristk_name)
        
        self.sock.send(msg)

        # 10 is a random num
        data = self.sock.recv(10)

        # subscribe to the event we want. In this case we only listen for athena.sloc.** events. ** is a wildcard
        self.sock.sendall('SUBSCRIBE athena.sloc.** \n')

        rospy.loginfo("SUCCESSFULLY SUBSCRIBED TO BROKER")

    # read line from socket
    def readlines(self, sock, recv_buffer=4096, delim='\n'):
        buffer = ''
        data = True
        while data:
            data = sock.recv(recv_buffer)
            buffer += data

            while buffer.find(delim) != -1:
                line, buffer = buffer.split('\n', 1)
                yield line
        return

    def send_message(self, event_name, text):
        json_format = "{ \"class\": \"iristk.system.Event\", \"event_sender\": \"%s\" , \"event_name\": \"%s\", \"text\": \"%s\" }\n"
        event_format = "EVENT {event_name} {byte_size}\n"

        j = json_format % (self.iristk_name, event_name, text)
        # len is to get bytes in python 2.X
        full_event = event_format.format(event_name=event_name, byte_size=len(j))
        self.sock.sendall(full_event)
        self.sock.sendall(j)
        rospy.loginfo("Sent event %s with json info %s" % (full_event, j))

    def run(self):
        r = rospy.Rate(100)
        while not rospy.is_shutdown():
            r.sleep()
            # if flag:
            #     self.send_message('iccs.quit.end', '')
            #     print 'Sent end'
            #     time.sleep(1)
            #     self.send_message('athena.admin.start_object', '')
            #     print 'Sent init'
            #     time.sleep(1)
            #     self.send_message('iccs.main.start', '')
            #     print 'Sent start'
            #     flag = False
            # for line in self.readlines(self.sock):
            #     pass
                # maybe send a message to iristk
                #self.send_message("event_name", "something_happened")
                #maybe publish something to ros
                #pub.publish("something")
                    

if __name__ == '__main__':
    ##### customizable parameters #####
    ticket_name = 'furhat'  # ticket name
    iristk_name = 'iccs_controller'  # name of the system in iristk
    ip = '192.168.0.105'  # broker ip
    port = 1932  # broker port
    ros_node_name = 'iccs_iristk_publisher'
    ###################################

    bridge = IrisTK_Bridge(ip,port,ticket_name,iristk_name,ros_node_name)
    bridge.run()
