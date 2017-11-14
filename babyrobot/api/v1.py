from flask import Flask, request, Response
from flask_api import status
from rospy_message_converter import json_message_converter

from babyrobot.api.utils import json_to_image, save_wav
from babyrobot.lib.utils import run_cmd

import cPickle
import json

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
    with open('/tmp/image.pkl','wb') as f:
        cPickle.dump(pil_img, f)
    # Call client object recognition
    code, out, err = run_cmd(('python /babyrobot-integration/babyrobot'
	'/src/objectrec/src/objectrec_client.py'))
    # Read JSON response
    with open('/tmp/image.json') as f:
        json_response = json.load(f)
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
    # Call client speech_features
    code, out, err = run_cmd(('python /babyrobot-integration/babyrobot'
	'/src/speech_features/src/speech_feature_client.py'))
    # Read JSON response
    with open('/tmp/speechfeatures.json') as f:
        json_response = json.load(f)
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
    code, out, err = run_cmd(('python /babyrobot-integration/babyrobot'
	'/src/emorec/src/emorec_client.py'))
    # Read JSON response
    with open('/tmp/emotions.json') as f:
        json_response = json.load(f)
    # Respond with JSON back to caller
    return Response(
        json_response,
        status=status.HTTP_200_OK,
        mimetype="application/json")


if __name__ == '__main__':
    app.run(port=4444, debug=False)
