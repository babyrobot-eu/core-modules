import os
import requests
from babyrobot.lib.config import BASE_PATH


def send_wav_request(clip_path):
    with open(clip_path, 'rb') as wav_fd:
        wav_string = wav_fd.read()
        r = requests.post("http://127.0.0.1:4444/v1/recognize/emotions",
                          json={"clip": wav_string.encode('base64')})
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
    send_wav_request(os.path.join(BASE_PATH,
                                  ('babyrobot/src/'
                                   'speech_features/test.wav')))
    send_image_request(os.path.join(BASE_PATH,
                                    ('babyrobot/'
                                     'objectrec/images/image1.jpg')))
