from flask import Flask, request, Response
from flask_api import status

app = Flask(__name__)


@app.route('/v1/recognize/objects', methods=['POST'])
def object_recognition():
    print(request.json)
    if request.json is None or not request.json['image']:
        resp = {
            "error": "No JSON POSTed containing the image"
        }
        return Response(
            resp,
            status=status.HTTP_400_BAD_REQUEST,
            mimetype="application/json")
    return Response(
        {"result": "Hello World"},
        status=status.HTTP_200_OK,
        mimetype="application/json")


@app.route('/v1/recognize/speech', methods=['POST'])
def speech_recognition():
    if request.json is None or not request.json['wav']:
        resp = {
            "error": "No JSON POSTed containing the wav clip"
        }
        return Response(
            resp,
            status=status.HTTP_400_BAD_REQUEST,
            mimetype="application/json")
    return Response(
        {"result": "Hello World"},
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
    return Response(
        {"result": "Hello World"},
        status=status.HTTP_200_OK,
        mimetype="application/json")


if __name__ == '__main__':
    app.run(port=4444, debug=True)
