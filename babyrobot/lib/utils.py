import functools
import shutil
import subprocess
import urllib
import yaml
import os

import rospy
from googletrans import Translator
from googleapiclient.discovery import build
import codecs
import sys

reload(sys)
sys.setdefaultencoding('utf8')


translator = Translator()


def translate(query, src='el', dest='en', paid=True):
    if paid:
        try:
            service = discovery.build(
                'translate', 'v2',
                developerKey=os.environ['TRANSLATE_KEY'])
            response  = service.translations().list(
                source=src,
                target=dest,
                q=[query]
            ).execute()
            return response['translations'][0]['translatedText'].encode('utf-8')
        except:
            translation = translator.translate(query, src=src, dest=dest)
            return translation.encode('utf-8')
    else:
        translation = translator.translate(query, src=src, dest=dest)
        return translation.encode('utf-8')


def yaml2dict(filename):
    with open(filename) as fd:
        yamldict = yaml.safe_load(fd)
    return yamldict


def get_parameter_from_yaml(parameter, ymldict=None, filename=None):
    """
    Returns the value of a given parameter in the dict ymldict
    which is constructed by calling yaml.safe_load in a YAML file.
    Alternatively you can pass the filename of the YAML file and
    the dict will be constructed in place.

    Parameter must be given in string format with dots
    Example: general.admin_nlp_server.ip
    """
    if ymldict is None:
        if filename is not None:
            ymldict = yaml2dict(filename)
        else:
            rospy.logerr("You must pass either ymldict or filename kwargs")
    value = ymldict
    for element in parameter.split("."):
        value = value.get(element)
        if value is None:
            raise ValueError("The parameter {0} is not defined"
                             .format(parameter))
    return value


def timethis(func):
    """
    Decorator that measure the time it takes for a function to complete
    Usage:
      @babyrobot.lib.utils.timethis
      def time_consuming_function(...):
    """
    @functools.wraps(func)
    def timed(*args, **kwargs):
        ts = time.time()
        result = func(*args, **kwargs)
        te = time.time()
        elapsed = '{0}'.format(te - ts)
        rospy.loginfo('{f}(*{a}, **{kw}) took: {t} sec'.format(
            f=func.__name__, a=args, kw=kwargs, t=elapsed))
        return result, elapsed
    return timed


def run_cmd(cmd):
    """
    Run given command locally
    Return a tuple with the return code, stdout, and stderr of the command
    """
    pipe = subprocess.Popen(cmd,
                            shell=True,
                            stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE)

    stdout, stderr = [stream.strip() for stream in pipe.communicate()]
    output = ' - STDOUT: "%s"' % stdout if len(stdout) > 0 else ''
    error = ' - STDERR: "%s"' % stdout if len(stderr) > 0 else ''
    rospy.logdebug("Running [{command}] returns: [{rc}]{output}{error}".format(
        command=cmd,
        rc=pipe.returncode,
        output=output,
        error=error))

    return pipe.returncode, stdout, stderr


def download_url(url, dest_path):
    """
    Download a file to a destination path given a URL
    """
    name = url.rsplit('/')[-1]
    dest = dest_path + "/" + name
    try:
        response = urllib.request.urlopen(url)
    except (urllib.error.HTTPError, urllib.error.URLError):
        return False

    with open(dest, 'wb') as fd:
        shutil.copyfileobj(response, fd)
    return True


def suppress_print(func):
    @functools.wraps(func)
    def func_wrapper(*args, **kwargs):
        with open('/dev/null', 'w') as sys.stdout:
            ret = func(*args, **kwargs)
        sys.stdout = sys.__stdout__
        return ret
    return func_wrapper


def write_wav(byte_str, wav_file):
    '''
    Write a hex string into a wav file

    Args:
        byte_str: The hex string containing the audio data
        wav_file: The output wav file

    Returns:
    '''
    with open(wav_file, 'w') as fd:
        fd.write(byte_str)


def mock_audio_segment(wav_sample):
    '''
    Mock an audio segment. Reads a test wav clip into a string
    and returns the hex string.
    Args:

    Returns:
        A hex string with the audio information.
    '''
    with open(wav_sample, 'r') as wav_fd:
        clip = wav_fd.read()
    return clip
