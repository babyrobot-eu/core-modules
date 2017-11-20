import os
import sys
import requests
from babyrobot.lib.config import BASE_PATH


def send_emotion_request(clip_path):
    with open(clip_path, 'rb') as wav_fd:
        wav_string = wav_fd.read()
        r = requests.post("http://127.0.0.1:4444/v1/recognize/emotions",
                          json={"clip": wav_string.encode('base64')})
        print r.text
        print r.status_code


def send_speech_feature_request(clip_path):
    with open(clip_path, 'rb') as wav_fd:
        wav_string = wav_fd.read()
        r = requests.post(("http://127.0.0.1:4444/v1/"
                           "recognize/speechfeatures"),
                          json={"clip": wav_string.encode('base64')})
        print r.text
        print r.status_code


def send_semantic_embedding_request(word):
    r = requests.post("http://127.0.0.1:4444/v1/get/semanticembeddings",
                      json={"word": word})
    print r.text
    print r.status_code


def send_fusion_request(word, text, visual, audio):
    r = requests.post("http://127.0.0.1:4444/v1/get/fusedembeddings",
                      json={"word": word,
                            "text": text,
                            "visual": visual,
                            "audio": audio})
    print r.text
    print r.status_code


def send_semantic_similarity_request(vector1, vector2):
    r = requests.post("http://127.0.0.1:4444/v1/get/semanticsimilarity",
                      json={"vector1": vector1,
                            "vector2": vector2})
    print r.text
    print r.status_code


def send_image_request(image_path):
    with open(image_path, 'rb') as img_fd:
        image_string = img_fd.read()
        r = requests.post("http://127.0.0.1:4444/v1/recognize/objects",
                          json={"image": image_string.encode('base64')})
        print r.text
        print r.status_code


if __name__ == '__main__':
    arg = sys.argv[1]
    if arg == 'emotion':
        send_emotion_request(
            os.path.join(BASE_PATH,
                         ('babyrobot/src/'
                          'speech_features/test.wav')))
    elif arg == 'image':
        send_image_request(
            os.path.join(BASE_PATH,
                         ('babyrobot/'
                          'objectrec/images/image2.jpg')))
    elif arg == 'features':
        send_speech_feature_request(
            os.path.join(BASE_PATH,
                         ('babyrobot/src/'
                          'speech_features/test.wav')))
    elif arg == 'embedding':
        send_semantic_embedding_request("sportsball")
    elif arg == 'fusion':
        send_fusion_request('test', [1, 2, 3], [4, 5, 6], [7, 8, 9])
    elif arg == 'similarity':
        send_semantic_similarity_request([1, 2, 3], [4, 5, 6])
