#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from babyrobot_msgs.msg import TimedString
from babyrobot.asr import gcloud
import sys
reload(sys)
sys.setdefaultencoding("utf-8")


class ASR(object):
    def __init__(self):
        self.pub = rospy.Publisher('/iccs/asr', TimedString, queue_size=100)
        rospy.init_node('iccs_asr', anonymous=True)
        rospy.Subscriber("/iccs/audio/recorder", String, self.handle_audio)
        self.transcription = TimedString()
        self.transcribed = False

    def handle_audio(self, audio):
        self.transcription.header.stamp = rospy.Time().now()
        self.transcription.data = gcloud.text2speech(audio.data, sr=16000)
        self.transcribed = True
        # rospy.loginfo(u"Continuous ASR (el): {}".format(self.transcription))

    def run(self):
        r = rospy.Rate(100)
        while not rospy.is_shutdown():
            if self.transcribed:
                self.pub.publish(self.transcription)
                self.transcribed = False
            r.sleep()


if __name__ == '__main__':
    ASR().run()
