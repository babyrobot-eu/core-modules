import base64
import numpy as np
import os
import pickleshare as psdb
import time

from babyrobot.api import config as api_config
from babyrobot.api import timer
from babyrobot.lib import config as br_config
from PIL import Image
from threading import Event

DB = psdb.PickleShareDB(api_config.DB)
DB.clear()


def handle_text_affect(text):
    del text
    valence = np.random.uniform(-1, 1)
    DB['text/valence'] = valence
    print('Text valence: {}'.format(valence))


def handle_cogstates(text):
    del text
    anx = np.random.uniform(0, 1)
    DB['cogni/anx'] = anx
    print('Cogni Stress: {}'.format(anx))
    affect = np.random.uniform(0, 1)
    DB['cogni/affect'] = affect
    print('Cogni affect: {}'.format(affect))
    percept = np.random.uniform(0, 1)
    DB['cogni/percept'] = percept
    print('Cogni percept: {}'.format(percept))
    proc = np.random.uniform(0, 1)
    DB['cogni/proc'] = proc
    print('Cogni proc: {}'.format(proc))
    drives = np.random.uniform(0, 1)
    DB['cogni/drives'] = drives
    print('Cogni drives: {}'.format(drives))
    social = np.random.uniform(0, 1)
    DB['cogni/social'] = social
    print('Cogni social: {}'.format(social))


def handle_speech_affect(audio):
    del audio
    emotions = ['negative', 'neutral', 'positive']
    emotion = np.random.choice(emotions)
    DB['speech/emotion'] = emotion
    print('Speech emotion: {}'.format(emotion))
    valence = np.random.uniform(-1, 1)
    DB['speech/valence'] = valence
    print('Speech valence: {}'.format(valence))


def handle_asr(audio):
    del audio
    text_path = os.path.join(
        br_config.BASE_PATH, 'models', 'test_text')
    selected = np.random.choice(os.listdir(text_path))
    txt_file = os.path.join(text_path, selected)
    with open(txt_file, 'r') as fd:
        transcription = [l.strip() for l in fd.readlines()][0]
    DB['speech/translated'] = transcription
    print('Speech transcription: {}'.format(transcription))


def handle_objectrec(img):
    del img
    img_path = os.path.join(
        br_config.BASE_PATH, 'models', 'test_images')
    selected = np.random.choice(os.listdir(img_path))
    image_file = os.path.join(img_path, selected)
    image = Image.open(image_file)
    image.load()
    data = np.asarray(image, dtype="int32")
    DB['objectrec/image'] = data
    print('Image: {}'.format(image_file))


def main():
    event = Event()
    timer.RepeatedTimer(2, handle_asr, {})
    timer.RepeatedTimer(2, handle_text_affect, {})
    timer.RepeatedTimer(2, handle_objectrec, {})
    timer.RepeatedTimer(2, handle_cogstates, {})
    timer.RepeatedTimer(2, handle_speech_affect, {})
    event.wait()


if __name__ == '__main__':
    main()
