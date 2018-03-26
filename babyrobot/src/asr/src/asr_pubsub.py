#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from babyrobot.asr import gcloud


class ASR(object):
    def __init__(self):
        self.pub = rospy.Publisher('/iccs/asr', String, queue_size=100)
        rospy.init_node('iccs_asr', anonymous=True)
        rospy.Subscriber("/iccs/audio/recorder", String, self.handle_audio)
        self.transcription = ''
        self.transcribed = False

    def handle_audio(self, audio):
        self.transcription = gcloud.text2speech(audio.data, sr=16000)
        self.transcribed = True
        print(u"Continuous ASR out: {}".format(self.transcription))

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.transcribed:
                self.pub.publish(self.transcription)
                self.transcribed = False
                r.sleep()


if __name__ == '__main__':
    ASR().run()
