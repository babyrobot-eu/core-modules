from flask import Flask, request, Response
from flask_api import status
from rospy_message_converter import json_message_converter

from babyrobot.api.utils import json_to_image, save_wav
from babyrobot.objectrec.client import objectrec
from babyrobot.emorec.client import emorec
from babyrobot.speech_features.client import extract_speech_features


app = Flask(__name__)


@app.route('/v1/recognize/objects', methods=['POST'])
def object_recognition():
    if request.json is None or not request.json['image']:
        resp = {
            "error": "No JSON POSTed containing the image"
        }
        return Response(
            resp,
            status=status.HTTP_400_BAD_REQUEST,
            mimetype="application/json")
    # Decode base64 image and convert it to PIL image format
    pil_img = json_to_image(base64_image=request.json['image'])
    # Call client object recognition
    recognized = objectrec(pil_img)
    # Convert ROS message to JSON object
    json_response = json_message_converter.\
        convert_ros_message_to_json(recognized)
    # Respond with JSON back to caller
    return Response(
        json_response,
        status=status.HTTP_200_OK,
        mimetype="application/json")


@app.route('/v1/recognize/speechfeatures', methods=['POST'])
def speech_feature_recognition():
    if request.json is None or not request.json['clip']:
        resp = {
            "error": "No JSON POSTed containing the wav clip"
        }
        return Response(
            resp,
            status=status.HTTP_400_BAD_REQUEST,
            mimetype="application/json")
    # Save local copy of wav data
    save_wav(base64_wav=request.json['clip'],
             save_dir='/tmp/wav_clip.wav')
    # Call client emorec_pytorch
    recognized = extract_speech_features('/tmp/wav_clip.wav')
    # Convert ROS message to JSON object
    json_response = json_message_converter. \
        convert_ros_message_to_json(recognized)
    # Respond with JSON back to caller
    return Response(
        json_response,
        status=status.HTTP_200_OK,
        mimetype="application/json")


@app.route('/v1/recognize/emotions', methods=['POST'])
def emotion_recognition():
    if request.json is None or not request.json['clip']:
        resp = {
            "error": "No JSON POSTed containing the wav clip"
        }
        return Response(
            resp,
            status=status.HTTP_400_BAD_REQUEST,
            mimetype="application/json")
    # Save local copy of wav data
    save_wav(base64_wav=request.json['clip'],
             save_dir='/tmp/wav_clip.wav')
    # Call client emorec_pytorch
    recognized = emorec('/tmp/wav_clip.wav')
    # Convert ROS message to JSON object
    json_response = json_message_converter. \
        convert_ros_message_to_json(recognized)
    # Respond with JSON back to caller
    return Response(
        json_response,
        status=status.HTTP_200_OK,
        mimetype="application/json")


if __name__ == '__main__':
    app.run(port=4444, debug=True)
