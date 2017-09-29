#!/usr/bin/env python
import rospy
import uuid

from babyrobot.emorec import config as emorec_config
from babyrobot.emorec import utils as emorec_utils
from babyrobot.speech_features import client as speech_feat_client
from babyrobot_msgs.msg import RecognizedEmotion
from babyrobot_msgs.msg import EmotionRecognitionResult
from babyrobot_msgs.srv import SpeechEmotionRecognition
from babyrobot_msgs.srv import SpeechEmotionRecognitionResponse


OPENEAR_MODELS = {}
CLASSES = {}
SCALES = {}


def handle_emorec(req):
    '''
    Emotion Recognition using libsvm or Weka models on an AudioSegment
    Args:
        req: A request object containing the following fields:
            audio_segment
            metadata
    Returns:
        A SpeechFeatureExtractionResponse containing an
        EmotionRecognitionResult ROS message.
    '''
    arff_features = None
    libsvm_features = None

    msg = EmotionRecognitionResult()
    msg.header.id = str(uuid.uuid1())
    msg.related_segment_id = req.audio_segment.header.id
    msg.header.timestamp = rospy.Time.now()

    for model, model_config in emorec_config.MODELS.ZOO.iteritems():
        recognized = RecognizedEmotion()
        if model_config['classifier'] == 'libsvm':
            if libsvm_features is None:
                libsvm_features = speech_feat_client.extract_speech_features(
                    req.audio_segment.clip,
                    opensmile_config=model_config['opensmile_config'],
                    response_format='list')
            prediction, confidence = emorec_utils.classify_libsvm(
                OPENEAR_MODELS[model],
                libsvm_features.features,
                scale=SCALES[model],
                classes=CLASSES[model])
        else:
            if arff_features is None:
                arff_features = speech_feat_client.extract_speech_features(
                    req.audio_segment.clip,
                    opensmile_config=model_config['opensmile_config'],
                    response_format='arff')
            prediction, confidence = emorec_utils.classify_weka(
                model_config,
                arff_features.arff_file,
                classes=CLASSES[model])
        recognized.model = model
        recognized.prediction = str(prediction)
        recognized.confidence = float(confidence)
        msg.emotions.append(recognized)

    return SpeechEmotionRecognitionResponse(msg)


def emorec_server():
    global OPENEAR_MODELS, CLASSES, SCALES
    rospy.init_node(emorec_config.ROS_CONFIG.SERVER_NODE)
    rospy.Service(
        emorec_config.ROS_CONFIG.SERVICE_NAME,
        SpeechEmotionRecognition, handle_emorec)
    for model, model_config in emorec_config.MODELS.ZOO.iteritems():
        if model_config['classifier'] == 'libsvm':
            classes_file = model_config.get('classes', None)
            print(classes_file)
            print(model)
            svm_model, scale, classes = emorec_utils.load_libsvm_model(
                model_config['path'],
                scale_file=model_config.get('scale', None),
                classes_file=model_config.get('classes', None))
            OPENEAR_MODELS[model] = svm_model
            CLASSES[model] = classes
            SCALES[model] = scale
        else:
            CLASSES[model] = (
                emorec_utils.parse_classes(model_config['classes'])
                if 'classes' in model_config else None)
    rospy.loginfo("Emotion Recognition server started.")
    rospy.spin()


if __name__ == "__main__":
    emorec_server()
