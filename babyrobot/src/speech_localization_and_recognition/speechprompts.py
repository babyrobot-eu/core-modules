#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from std_msgs.msg import String
from kws.recorder.alwaysOnCommandReco import *
from  kws.recorder.cvsp_recorder_on import *

def sleep(t):
    try:
        rospy.sleep(t)
    except:
        pass
        
def talker(wav_dir):
    pub = rospy.Publisher(topic, String)
    rospy.init_node('speechhandler', anonymous=True)
    try:
       while not rospy.is_shutdown():
          time.sleep(0.2)
	  output = commandReco.get_rec_output()
	  print output
	  #print "counter: {}, output: {}".format(counter, output)
	  commandReco.empty_rec_output()
          #rospy.loginfo(output)
          if "mobot komm" in output:
              pub.publish(wav_dir + "ComeCloser.mp3")
          if "mobot wie sp" in output:
              pub.publish(wav_dir + "WhatTime.mp3")
          if "mobot ich will aufstehen" in output:
              pub.publish(wav_dir + "WantStandUp.mp3")
          if "mobot hilfe" in output:
              pub.publish(wav_dir + "Help.mp3")
          if "mobot gehe weg" in output:
              pub.publish(wav_dir + "GoAway.mp3")
          if "mobot parke" in output:
              pub.publish(wav_dir + "Park.mp3")
          if "wo bin ich" in output:
              pub.publish(wav_dir + "WhereAmI.mp3")
          sleep(1)
    except KeyboardInterrupt:
	commandReco.exit_handler()
        
if __name__ == '__main__':
    import os
    currpath = os.path.dirname(os.path.abspath(__file__))
    import ConfigParser
    Config = ConfigParser.ConfigParser()
    Config.read(currpath+"/wavplayer.ini")
    topic = Config.get('Settings','Topic')
    CR = CvspRecorder()
    CR.config()
    CR.setup_alsa()
    commandReco = AlwaysOnCommandReco(CR)
    commandReco.config()
    commandReco.start_recording()
    talker(currpath + '/kws/prompts/validation/en/')
