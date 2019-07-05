#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Bool, UInt8
import roslib
import sys 
import sensor_msgs
import time
import numpy as np
import argparse
from math import ceil, sqrt, pi, exp
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from kinect_parser.msg import myString
from kws.recorder.mySRP3D import *
import message_filters

class AudioLocal:
  def __init__(self): 
      self.srp1 = []#np.zeros(160000)
      self.srp2 = []#np.zeros(160000)
      self.srp3 = []#np.zeros(160000)
      self.loc = [0.0,0.0,0.0]
      self.step = 0.1
      #self.srp1_pub = rospy.Publisher('/kinect1/srp', myString)
      
      self.lsb=[0.0,0.0,0.0]
      self.lsb=np.array(self.lsb)
      self.usb=[2.5, 2, 2.7]
      self.usb=np.array(self.usb)
      self.step=0.1
      self.mic_loc=[[20.0, 187.0, 175.0],[20.0, 191.0, 175.0], [20.0, 195.0, 175.0], [20.0, 199.0, 175.0], [20.0, 0.0, 175.0], [20.0, 4.0, 175.0], [20.0, 8.0, 175.0], [20.0, 12.0, 175.0], [116.0, 104.0, 269.0], [120.0, 104.0, 269.0], [124.0, 104.0, 269.0], [128.0, 104.0, 269.0]]
      self.mic_loc=np.array(self.mic_loc)
      
      self.mic_loc1=self.mic_loc[0:4,:]
      self.mic_loc1=self.mic_loc1/100.0
      
      self.mic_loc2=self.mic_loc[4:8,:]
      self.mic_loc2=self.mic_loc2/100.0

      self.mic_loc3=self.mic_loc[8:12,:]
      self.mic_loc3=self.mic_loc3/100.0

      self.fs=16000
      self.mapx=[[0,0,0,0],[1,0,0,0],[1,1,0,0],[1,1,1,0]]
      self.mapx=np.array(self.mapx,int)
      
      rospy.init_node("local_node", anonymous=True)
      #self.srp1_sub = rospy.Subscriber('/kinect1/srp',  myString, self.srp_k1)
      #self.srp1_sub = rospy.Subscriber('/kinect1/srp',  numpy_msg(Floats), self.srp_k1)
      #self.srp2_sub = rospy.Subscriber('/kinect2/srp',  numpy_msg(Floats), self.srp_k2)
      #self.srp3_sub = rospy.Subscriber('/kinect3/srp',  numpy_msg(Floats), self.srp_k3)
      audio1_sub = message_filters.Subscriber('/kinect1/srp', myString)
      audio2_sub = message_filters.Subscriber('/kinect2/srp', myString)
      audio3_sub = message_filters.Subscriber('/kinect3/srp', myString)
      self.loc_publisher = rospy.Publisher('/localization_topic', String)
      ts = message_filters.TimeSynchronizer([audio1_sub,audio2_sub,audio3_sub], 100)
      ts.registerCallback(self.srp_all)
      #rospy.spin()
      rospy.on_shutdown(self.shutdown)
      
  def shutdown(self):
      # cv2.destroyAllWindows()
    rospy.loginfo("Shutting down node...")
  
  def srp_all(self,audio1,audio2,audio3):
      self.srp1 = np.fromstring(audio1.data, dtype=np.int32)
      self.srp2 = np.fromstring(audio2.data, dtype=np.int32)
      self.srp3 = np.fromstring(audio3.data, dtype=np.int32)
      srp_shape = rospy.get_param('srp_shape')
      self.srp1 = np.reshape(self.srp1,srp_shape)
      self.srp2 = np.reshape(self.srp2,srp_shape)
      self.srp3 = np.reshape(self.srp3,srp_shape)
      #print srp_shape
      #print self.srp1.shape
      #print self.srp2.shape
      #print self.srp3.shape
      Srp1=mySRP3D(self.srp1,self.mic_loc1,self.fs,self.lsb,self.usb,self.step,self.mapx)
      #print Srp1
      Srp2=mySRP3D(self.srp2,self.mic_loc2,self.fs,self.lsb,self.usb,self.step,self.mapx)
      Srp3=mySRP3D(self.srp3,self.mic_loc3,self.fs,self.lsb,self.usb,self.step,self.mapx)
      SRPout = Srp1+Srp2+Srp3
      i,j,k = np.unravel_index(SRPout.argmax(), SRPout.shape)
      i=float(i)
      j=float(j)
      k=float(k)

      pos=np.array([i,j,k],float)
      pos=pos*self.step+self.step

      #print pos

      ####compute sloc wrt Furhat
      loc_y = pos[1]-0.90
      loc_x = pos[0]
      loc_z = 0.10
      #loc_y = loc_y.item()
      #print type(loc_y)
      fur_msg = str(loc_y.item()) + ' ' + str(loc_z) + ' ' + str(loc_x.item())
      self.loc_publisher.publish(fur_msg)

  def srp_k1(self,msg):
      self.srp1 = np.fromstring(msg.data, dtype=np.int32)
      #self.srp2 = np.fromstring(audio2.data, dtype=np.int32)
      #self.srp3 = np.fromstring(audio3.data, dtype=np.int32)
      srp_shape = rospy.get_param('srp_shape')
      self.srp1 = np.reshape(self.srp1,srp_shape)
      #self.srp2 = np.reshape(self.srp2,srp_shape)
      #self.srp3 = np.reshape(self.srp3,srp_shape)
      #print srp_shape
      #print self.srp1.shape
      #print self.srp2.shape
      #print self.srp3.shape
      Srp1=mySRP3D(self.srp1,self.mic_loc1,self.fs,self.lsb,self.usb,self.step,self.mapx)
      #print Srp1
      #Srp2=mySRP3D(self.srp2,self.mic_loc2,self.fs,self.lsb,self.usb,self.step,self.mapx)
      #Srp3=mySRP3D(self.srp3,self.mic_loc3,self.fs,self.lsb,self.usb,self.step,self.mapx)
      SRPout = Srp1
      i,j,k = np.unravel_index(SRPout.argmax(), SRPout.shape)
      i=float(i)
      j=float(j)
      k=float(k)

      pos=np.array([i,j,k],float)
      pos=pos*self.step+self.step

      print pos

      ####compute sloc wrt Furhat
      loc_y = pos[0]-1.0
      loc_x = pos[1]
      loc_z = 1.60
      #loc_y = loc_y.item()
      #print type(loc_y)
      fur_msg = str(loc_x.item()) + ' ' + str(loc_y.item()) + ' ' + str(loc_z)
      self.loc_publisher.publish(fur_msg)


      #self.srp1 = msg.data
      #srp_shape = rospy.get_param('srp_shape') 
      #print srp_shape
      #print self.srp1.shape
      #print self.srp2.shape
      #print self.srp3.shape
      #Srp1 = np.reshape(self.srp1,srp_shape)
      #Srp2 = np.reshape(self.srp2,srp_shape)
      #Srp3 = np.reshape(self.srp3,srp_shape)
      #print Srp1,Srp2,Srp3
      ####compute sloc
      #SRPout = Srp1#+Srp2#+Srp3
      #i,j,k = np.unravel_index(SRPout.argmax(), SRPout.shape)
      #i=float(i)
      #j=float(j)
      #k=float(k)

      #pos=np.array([i,j,k],float)
      #pos=pos*self.step+self.step
      #print pos

      #theta = 80*pos[0]
      #print type(theta.item())
      #rospy.set_param('sloc_theta',theta.item())
      
      ####compute sloc wrt Furhat
      #loc_y = pos[0]-1.0
      #loc_x = pos[1]
      #loc_z = 1.60
      #`loc_y = loc_y.item()
      #print type(loc_y)
      #fur_msg = str(loc_x.item()) + ' ' + str(loc_y.item()) + ' ' + str(loc_z)
      #self.loc_publisher.publish(fur_msg)
      
  
  def srp_k2(self,msg):
      self.srp2 = msg.data
      
  def srp_k3(self,msg):   
      self.srp3 = msg.data
      
      
def main(args):
  r = AudioLocal()
  try:
    rospy.spin();
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


if __name__ == '__main__':
  main(sys.argv)
