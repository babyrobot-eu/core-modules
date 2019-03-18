#!/usr/bin/env python

#from nao_actions import nao_motion
import socket
import sys
from datetime import datetime
import rospy
from geometry_msgs.msg import PointStamped
from button_callbacks_gchal_v3 import *
import time
#import action_listener



class NAO(object):

	def __init__(self):
		# Default settings
		self.host = "192.168.0.105"
		self.port = 1932
		self.ip = "192.168.0.121"
		self.port2 = 9559

		# if len(sys.argv) > 2:
		self.connect_to_broker(self.host,self.port)
		#self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)


		# create publisher
		self.pub = rospy.Publisher('/robot_active', PointStamped)
		rospy.init_node('robot_active', anonymous=True)


		# start experiment
		global exper_id
		exper_id = 1 
		exp_init= self.send_event(event="iccs.actionNAO.recognized", timeset1=init_process(), action=-1, flag=-1, experiment= int(exper_id))


		# nao_posture(self.ip, int(self.port2), "Crouch")
		# time.sleep(2)
		nao_stiffness(self.ip, int(self.port2), 1.0)
		time.sleep(2)
		nao_posture(self.ip, int(self.port2), "Stand")
		time.sleep(1)
		nao_walk(self.ip, int(self.port2), float(0.25))
		
		msg_out = PointStamped()
		self.count = 0
		rospy.Subscriber("/robot_action", PointStamped, self.callback)
		rospy.spin()

		# rospy.init_node('action_listener', anonymous=True)
		#msg_out = PointStamped()
		#while not rospy.is_shutdown():	
		#	while count<2:		
		#		rospy.Subscriber("/robot_action", PointStamped, self.callback(msg_out))
		#		rospy.sleep(1)  # sleep for one second

		#		count +=1


	def callback(self,msg_out):

		rospy.loginfo("next action %d", msg_out.point.x)
		global exper_id
		action=int(msg_out.point.x)
		print(msg_out)
		if action!=0:
			self.count=self.count+1
		if action == 1:
			self.send_event(event="iccs.actionNAO.recognized", timeset1=init_process(), action=1, flag=1, experiment= exper_id) 
			self.send_event(event="iccs.actionNAO.recognized", timeset1=nao_action1(self.ip, int(self.port2)), action=1, flag=0, experiment=exper_id)
		elif action == 2: 
			self.send_event(event="iccs.actionNAO.recognized", timeset1=init_process(), action=2, flag=1, experiment= exper_id) 
			self.send_event(event="iccs.actionNAO.recognized", timeset1=nao_action2(self.ip, int(self.port2)), action=2, flag=0, experiment=exper_id)
		elif action == 3: 
			self.send_event(event="iccs.actionNAO.recognized", timeset1=init_process(), action=3, flag=1, experiment= exper_id) 
			self.send_event(event="iccs.actionNAO.recognized", timeset1=nao_action3(self.ip, int(self.port2)), action=3, flag=0, experiment=exper_id)
		elif action == 4: 
			self.send_event(event="iccs.actionNAO.recognized", timeset1=init_process(), action=4, flag=1, experiment= exper_id) 
			self.send_event(event="iccs.actionNAO.recognized", timeset1=nao_action4(self.ip, int(self.port2)), action=4, flag=0, experiment=exper_id)
		elif action == 5: 
			self.send_event(event="iccs.actionNAO.recognized", timeset1=init_process(), action=5, flag=1, experiment= exper_id) 
			self.send_event(event="iccs.actionNAO.recognized", timeset1=nao_action5(self.ip, int(self.port2)), action=5, flag=0, experiment=exper_id)
		elif action == 6: 
			self.send_event(event="iccs.actionNAO.recognized", timeset1=init_process(), action=6, flag=1, experiment= exper_id) 
			self.send_event(event="iccs.actionNAO.recognized", timeset1=nao_action6(self.ip, int(self.port2)), action=6, flag=0, experiment=exper_id)
		elif action == 7: 
			self.send_event(event="iccs.actionNAO.recognized", timeset1=init_process(), action=7, flag=1, experiment= exper_id) 
			self.send_event(event="iccs.actionNAO.recognized", timeset1=nao_action7(self.ip, int(self.port2)), action=7, flag=0, experiment=exper_id)
		elif action == 8: 
			self.send_event(event="iccs.actionNAO.recognized", timeset1=init_process(), action=8, flag=1, experiment= exper_id) 
			self.send_event(event="iccs.actionNAO.recognized", timeset1=nao_action8(self.ip, int(self.port2)), action=8, flag=0, experiment=exper_id)

		if self.count>2:
			self.send_event(event="iccs.actionNAO.recognized", timeset1=init_process(), action=-1, flag=-1, experiment= 1000)
			nao_open_hand(self.ip, int(self.port2), 0.8)
			nao_say(self.ip, int(self.port2), "dose")
			time.sleep(2)
			nao_grasp(self.ip, int(self.port2), 0.5)
			time.sleep(2)
			nao_walk(self.ip, int(self.port2), float(-0.35))
			nao_turn(self.ip, int(self.port2), float(0.7))
			nao_posture(self.ip, int(self.port2), "Crouch")
			
			self.send_event_iristk()


	
	def connect_to_broker(self,host,port):
		self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		# Connect the socket to the port where the server is listening
		server_address = (self.host, self.port)
		self.sock.connect(server_address)

		message = 'CONNECT furhat admin \n'
		self.sock.sendall(message)

		l = self.sock.recv(8192)
		print "SUCCESSFULLY SUBSCRIBED TO BROKER ENGAGE"

	def unix_time_millis(self, dt):
		epoch = datetime.utcfromtimestamp(0)
		return (dt - epoch).total_seconds() * 1000.0


	def send_event_iristk(self):
	
		#sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

		json_format_1 = """{ \"class\": \"iristk.system.Event\", \"event_name\": \"athena.engagement.end\" }\n"""
		event_format = "EVENT athena.engagement.end %d\n"
		j =  json_format_1
		# len is to get bytes in python 2.X
		full_event = event_format % len(j)
		self.sock.sendall(full_event)
		self.sock.sendall(j)
		
		
	def send_event(self, event, timeset1, action, flag, experiment):
		print(event, timeset1[0], action, flag, experiment)

		# json_format_1 = """{ \"class\": \"iristk.system.Event\", \"event_name\": \"%s\", \"time_start_end\": \"%d\", \"text\": \"%s\" }\n"""
		# event_format = "EVENT %s %f %d %s\n"

		# js = json_format_1 % (event, timeset1[0], action, flag)
		# self.sock.sendall(event_format % (event, len(js)))
		# self.sock.sendall(js)
		# print "Sending event %s with text %d and timestamps %f of type %d" % (event, action, timeset1[0], flag)

		msg = PointStamped()
		msg.point.x = flag
		msg.point.y = action
		msg.point.z = experiment
		msg.header.stamp = rospy.Time.now()
		print msg.point.z, msg.point.x, msg.point.y
		self.pub.publish(msg)

	def run(self):
		# Starts and runs the GUI
		self.root.mainloop()

	def combine_funcs(*funcs):
		def combined_func(*args, **kwargs):
			for f in funcs:
				f(*args, **kwargs)
		return combined_func
## AUTOSTART ##
if __name__ == "__main__":
	remote = NAO()
	remote.run()
