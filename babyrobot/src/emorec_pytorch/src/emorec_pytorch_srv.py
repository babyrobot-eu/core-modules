#!/usr/bin/env python
import base64
import pickle
import sys
import uuid
import io

import numpy as np
import rospy
from scipy.io.wavfile import read as wav_read
from scipy.io.wavfile import write as wav_write

import torch
from babyrobot.emorec_pytorch import config as emorec_pytorch_config
from babyrobot.speech_features import client as speech_feat_client
from babyrobot_msgs.msg import EmotionRecognitionResult
from babyrobot_msgs.srv import SpeechEmotionRecognition
from babyrobot_msgs.srv import SpeechEmotionRecognitionResponse
from speech_features import frame_breaker
from torch.autograd import Variable

sys.path.append(emorec_pytorch_config.Paths.src)


# def extract_features(clip):
#     """
#     Feature extraction from an audio clip
#     Args:
#         clip ():
#
#     Returns: A list of feature vectors
#
#     """
#
#     extracted_feats = speech_feat_client.extract_speech_features(
#         clip,
#         opensmile_config=emorec_pytorch_config.ModelBaseline.opensmile_config,
#         response_format='list'
#     )
#     feats = numpy.array([f.feature_value for f in extracted_feats])
#     return feats


def extract_feats_for_segment(s):
    return speech_feat_client.extract_speech_features(
            s,
            opensmile_config=emorec_pytorch_config.ModelBaseline.opensmile_config,
            response_format='list'
        )


def np2base64(s, sr):
    wav_write('/tmp/tmp.wav', sr, s)
    with open('/tmp/tmp.wav', 'r') as fd:
        s2 = fd.read()
    return s2


def extract_features(clip):
    """
    Feature extraction from an audio clip
    Args:
        clip ():

    Returns: A list of feature vectors

    """
    sr, clip_array = wav_read(io.BytesIO(clip))
    if clip_array.ndim > 1:
        clip_array = clip_array[:, 0]
    segments = frame_breaker.get_frames(clip_array, sample_rate=sr)
    segments_encoded = [np2base64(s, sr) for s in segments]
    segment_features = [
        [f.feature_value for f in extract_feats_for_segment(s).features]
        for s in segments_encoded
    ]
    # extracted_feats = speech_feat_client.extract_speech_features(
    #     clip,
    #     opensmile_config=emorec_pytorch_config.ModelBaseline.opensmile_config,
    #     response_format='list'
    # )
    # feats = np.array([f.feature_value for f in extracted_feats])
    return segment_features


def handle_emorec(req):
    '''
    Emotion Recognition using pytorch model, trained on IEMOCAP
    Args:
        req: A request object containing the following fields:
            audio_segment
            metadata
    Returns:
        A SpeechFeatureExtractionResponse containing an
        EmotionRecognitionResult ROS message.
    '''
    msg = EmotionRecognitionResult()
    msg.header.id = str(uuid.uuid1())
    msg.related_segment_id = req.audio_segment.header.id
    msg.header.timestamp = rospy.Time.now()

    # extract features from the clip
    clip = extract_features(req.audio_segment.clip)

    # pass the features to the data_manager, in order to convert the sample
    # to the format in which the pytorch model expects it to be
    input, length = _data_manager.prep_sample(clip)

    input = input.astype('float32')

    # convert the input to torch tensors
    value = torch.from_numpy(input)
    length = torch.from_numpy(np.array([length]))

    test_value = Variable(value, volatile=True)
    test_length = Variable(length, volatile=True)

    # make it float and insert a fake batch dimension
    test_value = test_value.float()
    test_value = test_value.unsqueeze(0)

    # cast the length tensor tos a LongTensor
    test_length = test_length.long()

    # pass the sample and it's length to the model and get the predictions
    cat_outputs, cont_outputs = _model(test_value, test_length)

    # move the output tensors to the cpu and convert them to numpy arrays
    cat_outputs = cat_outputs.data.cpu().numpy()
    intensities = cont_outputs.data.cpu().numpy()

    # get the predicted polarity
    pred_cat = np.argmax(cat_outputs)
    polarity = _data_manager.label_cat_encoder.inverse_transform(pred_cat)

    msg.polarity = polarity
    msg.intensities = intensities

    return SpeechEmotionRecognitionResponse(msg)


def emorec_server():
    rospy.init_node(emorec_pytorch_config.ROS_CONFIG.SERVER_NODE)
    rospy.Service(emorec_pytorch_config.ROS_CONFIG.SERVICE_NAME,
                  SpeechEmotionRecognition, handle_emorec)
    global _model, _data_manager

    model_path = emorec_pytorch_config.Paths.checkpoint
    data_manager_path = emorec_pytorch_config.Paths.data_manager
    _model = torch.load(model_path)
    _data_manager = pickle.load(open(data_manager_path, 'rb'))
    _model.eval()

    rospy.loginfo("Emotion Recognition server started.")
    rospy.spin()


if __name__ == "__main__":
    emorec_server()
