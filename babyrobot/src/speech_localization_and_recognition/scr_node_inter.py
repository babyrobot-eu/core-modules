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
from kws.recorder.cvsp_recorder_on import *

def sleep(t):
    try:
        rospy.sleep(t)
    except:
        pass
        
class talker():
    def __init__(self):
        pub = rospy.Publisher(topic, String)
        #rospy.init_node('speech_rec', anonymous=True)
        rospy.on_shutdown(self.exit_gracefully)
        try:
            while not rospy.is_shutdown():
                if CR.threads_die_now:
                    commandReco.exit_handler()
                
                time.sleep(0.2)
                #output = commandReco.get_rec_output()
                output = commandReco.get_msg()
                #print output
                #print "counter: {}, output: {}".format(counter, output)
                #commandReco.empty_rec_output()
                commandReco.empty_msg()
                fields = output.split()
                if len(fields)>0 and fields[3] != 'BG':
                    output = re.sub('BG \d+(\.\d+)?','',output)
                    output = re.sub('\s+',' ',output)
                    rospy.loginfo(output)
                    pub.publish(output)
        except KeyboardInterrupt:
            commandReco.exit_handler()
        
    def exit_gracefully(self):
        commandReco.exit_handler()
        print "Spoken Command Recognition node is ending..."
            
if __name__ == '__main__':
    import os
    
    #if len(sys.argv) > 1:
    #   config = sys.argv[1]
    #else:
    #   print 'A yaml config file is expected as input!!'

    #config = '/home/nick/catkin_ws/src/cvsp_audio_gesture/config/scr_greek.yaml'

    currpath = os.path.dirname(os.path.abspath(__file__))
    config = os.path.join(currpath, '../../config/babyrobot_interspeech.yaml')
    import ConfigParser
    Config = ConfigParser.ConfigParser()
    Config.read(currpath + "/fusion.ini")
    topic = Config.get('Settings','Topic')
    CR = CvspRecorder()
    CR.config(config)
    CR.setup_alsa()
    commandReco = AlwaysOnCommandReco(CR)
    commandReco.config(config)
    commandReco.start_recording()
    talker()
