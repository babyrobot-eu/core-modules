from flask import Flask, request, Response
from flask_api import status

from babyrobot.api import config as api_config
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
    with open('/tmp/image.pkl', 'wb') as f:
        cPickle.dump(pil_img, f)
    # Call client object recognition
    code, out, err = run_cmd(
        'python {client}'.format(client=api_config.OBJECTREC_CLIENT))
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
    code, out, err = run_cmd(
        'python {client}'.format(client=api_config.SPEECH_FEAT_CLIENT))
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
    # Call client emorec
    code, out, err = run_cmd(
        'python {client}'.format(client=api_config.EMOREC_CLIENT))
    # Read JSON response
    with open('/tmp/emotions.json') as f:
        json_response = json.load(f)
    # Respond with JSON back to caller
    return Response(
        json_response,
        status=status.HTTP_200_OK,
        mimetype="application/json")


@app.route('/v1/get/semanticembeddings', methods=['POST'])
def get_semantic_embeddings():
    if request.json is None or not request.json['word']:
        resp = {
            "error": "No JSON POSTed containing the wav clip"
        }
        return Response(
            resp,
            status=status.HTTP_400_BAD_REQUEST,
            mimetype="application/json")
    # Get word from json message
    word = request.json['word']
    # Call client concept_space
    code, out, err = run_cmd(
        'python {client} {word}'.format(
            client=api_config.CONCEPT_SPACE_CLIENT,
            word=word))
    # Read JSON response
    with open('/tmp/semantic_embeddings.json') as f:
        json_response = json.load(f)
    # Respond with JSON back to caller
    return Response(
        json_response,
        status=status.HTTP_200_OK,
        mimetype="application/json")


@app.route('/v1/get/fusedembeddings', methods=['POST'])
def fuse_semantic_embeddings():
    if request.json is None or not request.json['text']:
        resp = {
            "error": "No JSON POSTed containing the wav clip"
        }
        return Response(
            resp,
            status=status.HTTP_400_BAD_REQUEST,
            mimetype="application/json")
    # Get embeddings from json message
    text = request.json['text']
    visual = request.json['visual']
    audio = request.json['audio']
    # Call client concept_space
    code, out, err = run_cmd(
        'python {client} {text} {visual} {audio}'.format(
            client=api_config.EMBEDDINGS_FUSION_CLIENT,
            text=text,
            visual=visual,
            audio=audio))
    # Read JSON response
    with open('/tmp/fused_embeddings.json') as f:
        json_response = json.load(f)
    # Respond with JSON back to caller
    return Response(
        json_response,
        status=status.HTTP_200_OK,
        mimetype="application/json")


@app.route('/v1/get/semanticsimilarity', methods=['POST'])
def get_semantic_similarity():
    if request.json is None or not request.json['vector1']:
        resp = {
            "error": "No JSON POSTed containing the wav clip"
        }
        return Response(
            resp,
            status=status.HTTP_400_BAD_REQUEST,
            mimetype="application/json")
    # Get embeddings from json message
    v1 = request.json['vector1']
    v2 = request.json['vector2']
    # Call client concept_space
    code, out, err = run_cmd(
        'python {client} {vector1} {vector2}'.format(
            client=api_config.SEMANTIC_SIMILARITY_CLIENT,
            vector1=v1,
            vector2=v2))
    # Read JSON response
    with open('/tmp/semantic_similarity.json') as f:
        json_response = json.load(f)
    # Respond with JSON back to caller
    return Response(
        json_response,
        status=status.HTTP_200_OK,
        mimetype="application/json")


if __name__ == '__main__':
    app.run(port=4444, debug=False)
