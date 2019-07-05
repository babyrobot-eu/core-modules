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

class AudioLocal:
  def __init__(self): 
      self.srp1 = np.zeros((26,21,28))
      self.srp2 = np.zeros((26,21,28))
      self.srp3 = np.zeros((26,21,28))
      self.loc = [0.0,0.0,0.0]
      self.step = 0.1
      rospy.init_node("local_node", anonymous=True)
      self.srp1_sub = rospy.Subscriber('/kinect1/srp',  numpy_msg(Floats), self.srp_k1)
      self.srp2_sub = rospy.Subscriber('/kinect2/srp',  numpy_msg(Floats), self.srp_k2)
      self.srp3_sub = rospy.Subscriber('/kinect3/srp',  numpy_msg(Floats), self.srp_k3)
      self.loc_publisher = rospy.Publisher('/localization_topic', String)
      rospy.on_shutdown(self.shutdown)
      
  def shutdown(self):
      # cv2.destroyAllWindows()
    rospy.loginfo("Shutting down node...")
    
  def srp_k1(self,msg):
      self.srp1 = msg.data
      srp_shape = rospy.get_param('srp_shape') 
      print srp_shape
      Srp1 = np.reshape(self.srp1,srp_shape)
      Srp2 = np.reshape(self.srp2,srp_shape)
      Srp3 = np.reshape(self.srp3,srp_shape)
      #print Srp1,Srp2,Srp3
      ####compute sloc
      SRPout = Srp1#+Srp2#+Srp3
      i,j,k = np.unravel_index(SRPout.argmax(), SRPout.shape)
      i=float(i)
      j=float(j)
      k=float(k)

      pos=np.array([i,j,k],float)
      pos=pos*self.step+self.step
      #print pos

      #theta = 80*pos[0]
      #print type(theta.item())
      #rospy.set_param('sloc_theta',theta.item())
      
      ####compute sloc wrt Furhat
      loc_y = pos[0]-1.0
      loc_x = pos[1]
      loc_z = 1.60
      #`loc_y = loc_y.item()
      #print type(loc_y)
      fur_msg = str(loc_x.item()) + ' ' + str(loc_y.item()) + ' ' + str(loc_z)
      self.loc_publisher.publish(fur_msg)
      
  
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
