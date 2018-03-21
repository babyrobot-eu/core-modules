import argparse
import base64
import httplib2
import json
import numpy as np

from googleapiclient import discovery
from oauth2client.client import GoogleCredentials
from scipy.io.wavfile import read as wavfile_read


DISCOVERY_URL = 'https://{api}.googleapis.com/$discovery/rest?version={apiVersion}'


def get_speech_service():
    credentials = GoogleCredentials.get_application_default().create_scoped(
        ['https://www.googleapis.com/auth/cloud-platform'])
    http = httplib2.Http()
    credentials.authorize(http)

    return discovery.build(
        'speech', 'v1beta1', http=http, discoveryServiceUrl=DISCOVERY_URL)


def text2speech_wav(speech_file):
    """Transcribe the given audio file.

    Args:
        speech_file: the name of the audio file.
    """
    with open(speech_file, 'rb') as speech:
        sr, s = wavfile_read(speech)
        if s.ndim > 1:
            s = np.ascontiguousarray(s[:, 0])
        speech_content = base64.b64encode(s)
    return text2speech(speech_content)


def text2speech(base64_clip):
    service = get_speech_service()
    service_request = service.speech().syncrecognize(
        body={
            'config': {
                'encoding': 'LINEAR16',  # raw 16-bit signed LE samples
                'sampleRate': sr,  # 16 khz
                'languageCode': 'el-GR',  # a BCP-47 language tag
            },
            'audio': {
                'content': base64_clip.decode('UTF-8')
                }
            })
    response = service_request.execute()
    return response['results'][0]['alternatives'][0]['transcript']

