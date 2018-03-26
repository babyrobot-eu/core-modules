#!/usr/bin/env python
import rospy
from babyrobot_msgs.msg import TimedString
from std_msgs.msg import String
from babyrobot.lib import utils as br_utils


class Translate(object):
    def __init__(self):
        self.pub = rospy.Publisher('/iccs/translate', TimedString, queue_size=100)
        rospy.init_node('iccs_translate', anonymous=True)
        rospy.Subscriber("/iccs/asr", TimedString, self.handle_transcription)
        self.translation = TimedString()
        self.translated = False

    def handle_transcription(self, transcription):
        self.translation.header.stamp = rospy.Time().now()
        self.translation.data = br_utils.translate(transcription.data, src='el', dest='en')
        self.translated = True
        rospy.loginfo(u"Transcription (en): {}".format(self.translation.data))

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.translated:
                self.pub.publish(self.translation)
                self.translated = False
                r.sleep()


if __name__ == '__main__':
    Translate().run()
