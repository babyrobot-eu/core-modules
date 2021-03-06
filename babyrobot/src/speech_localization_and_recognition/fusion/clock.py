#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import sys


def exit_gracefully():
        print "The Clock node is ending..."
        sys.exit()
        
        
pub = rospy.Publisher('spk_action_rec_audio', String)
rospy.init_node('clock')
rospy.on_shutdown(exit_gracefully)

        
while not rospy.is_shutdown():
  try:
      rospy.sleep(0.5)
      pub.publish('5 %s' % (rospy.Time().to_nsec()))
  except KeyboardInterrupt:
      sys.exit()
