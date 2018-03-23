#!/usr/bin/env python
import csv
import os
import rospy
import uuid

from babyrobot.lib import utils as br_utils
from babyrobot.speech_features import config as sf_config
from babyrobot_msgs.msg import SpeechFeatures, Feature
from babyrobot_msgs.srv import SpeechFeatureExtraction
from babyrobot_msgs.srv import SpeechFeatureExtractionResponse


def handle_speech_features(req):
    '''
    Extract speech features upon a client request.

    Args:
        req: A request object containing the following fields:
            audio_segment
            opensmile_config
            response_format
            metadata
    Returns:
        A SpeechFeatureExtractionResponse containing a SpeechFeatures
        ROS message.
    '''
    br_utils.write_wav(req.audio_segment.clip, sf_config.TEMP_FILE.WAV)
    if req.response_format == 'arff':
        out_file = sf_config.TEMP_FILE.ARFF_OUTPUT
        out_file_cmd = '--arffoutput {}'.format(out_file)
    else:
        out_file = sf_config.TEMP_FILE.CSV_OUTPUT
        out_file_cmd = '--csvoutput {}'.format(out_file)

    opensmile_conf = os.path.join(sf_config.OPENSMILE.CONFIG_DIR,
                                  req.opensmile_conf)
    if not opensmile_conf.endswith('.conf'):
        opensmile_conf += '.conf'
    cmd = '{0}/SMILExtract -C {1} -I {2} {3}'.format(
        sf_config.OPENSMILE.ROOT_DIR,
        opensmile_conf,
        sf_config.TEMP_FILE.WAV,
        out_file_cmd)
    rospy.loginfo('Extracting features using "{}"'.format(cmd))

    ret_code, stdout, stderr = br_utils.run_cmd(cmd)
    if ret_code != 0:
        rospy.logerr('Failed to extract features. Returning empty message')
        rospy.logerr(stdout)
        rospy.logerr(stderr)
        return SpeechFeatureExtractionResponse(SpeechFeatures())

    msg = SpeechFeatures()
    msg.related_segment_id = req.audio_segment.header.id
    msg.header.id = str(uuid.uuid1())
    msg.header.timestamp = rospy.Time.now()

    with open(out_file, 'r') as out_fd:
        if req.response_format == 'arff':
            msg.arff_file = out_fd.read()
        elif req.response_format in ['list', '']:
            reader = csv.DictReader(out_fd)
            data = reader.next()
            msg.features = []
            for d in data.iteritems():
                feat = Feature()
                feat.feature_name = d[0]
                feat.feature_value = float(d[1])
                msg.features.append(feat)
        else:
            rospy.logerr('Invalid response format. Returning empty message')
            return SpeechFeatureExtractionResponse(SpeechFeatures())

    return SpeechFeatureExtractionResponse(msg)


def speech_features_server():
    '''
    Initialize a ROS node and run the SpeechFeatureExtraction service

    Args:

    Returns:
    '''
    rospy.init_node(sf_config.ROS_CONFIG.SERVER_NODE)
    rospy.Service(sf_config.ROS_CONFIG.SERVICE_NAME,
                  SpeechFeatureExtraction,
                  handle_speech_features)
    rospy.loginfo("Speech Features server started.")
    rospy.spin()


if __name__ == "__main__":
    speech_features_server()
