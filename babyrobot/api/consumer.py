import pprint

from flask import Flask, request, Response
from flask_api import status
import pickleshare as psdb

from babyrobot.api import config as api_config
from babyrobot.api import utils as api_utils
from flask import jsonify

from babyrobot.lib.utils import run_cmd

try:
    import cPickle as pickle
except ImportError:
    import pickle

try:
    import ujson as json
except ImportError:
    import json

app = Flask(__name__)

DB = psdb.PickleShareDB(api_config.DB)


@app.route('/objectrec', methods=['GET'])
def object_recognition():
    """
    Returns json in format
    {
        'image': <base64 RGB Image>
    }
    """
    json_response = {}
    if 'objectrec/image' in DB.keys():
        json_response['image'] = api_utils.np_to_base64(DB['objectrec/image'])

    return Response(
        json_response,
        status=status.HTTP_200_OK,
        mimetype="application/json")


@app.route('/speech_affect', methods=['GET'])
def speech_affect():
    """
    Returns json in format
    {
        'emotion': <recognized emotion>,
        'valence': <predicted valence>
    }
    """
    json_response = {}
    if 'speech/emotion' in DB.keys():
        json_response['emotion'] = DB['speech/emotion']
    if 'speech/valence' in DB.keys():
        json_response['valence'] = DB['speech/valence']
    return Response(
        json_response,
        status=status.HTTP_200_OK,
        mimetype="application/json")


@app.route('/transcription', methods=['GET'])
def transcription():
    """
    Returns json in format
    {
        'transcription': <transcription of ASR in english>
    }
    """
    json_response = {}
    if 'speech/translated' in DB.keys():
        json_response['transcription'] = DB['speech/translated']
    pprint.pprint(json_response)
    return Response(
        jsonify(json_response),
        status=status.HTTP_200_OK,
        mimetype="application/json")


@app.route('/text_affect', methods=['GET'])
def text_affect():
    """
    Returns json in format
    {
        'valence': <predicted valence>
    }
    """
    json_response = {}
    if 'text/valence' in DB.keys():
        json_response['valence'] = DB['text/valence']
    return Response(
        json_response,
        status=status.HTTP_200_OK,
        mimetype="application/json")


@app.route('/cognistates', methods=['GET'])
def cognitive_states():
    """
        Returns json in format
        {
            'affect': <predicted affect score>,
            'drives': <predicted drives score>,
            'load': <predicted cognitive load score>
            'percept': <predicted percept score>,
            'social': <predicted social score>,
            'stress': <predicted stress score>,
        }
        """
    json_response = {}
    if 'cogni/anx' in DB.keys():
        json_response['stress'] = DB['cogni/anx']
    if 'congi/affect' in DB.keys():
        json_response['affect'] = DB['cogni/affect']
    if 'congi/percept' in DB.keys():
        json_response['percept'] = DB['cogni/percept']
    if 'congi/proc' in DB.keys():
        json_response['load'] = DB['cogni/proc']
    if 'congi/drives' in DB.keys():
        json_response['drives'] = DB['cogni/drives']
    if 'congi/social' in DB.keys():
        json_response['social'] = DB['cogni/social']
    return Response(
        json_response,
        status=status.HTTP_200_OK,
        mimetype="application/json")


if __name__ == '__main__':
    app.run(port=4444, host='0.0.0.0', debug=False)
