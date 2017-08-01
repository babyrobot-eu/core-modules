from babyrobot.speech_features import config as sf_config


def mock_audio_segment():
    '''
    Mock an audio segment. Reads a test wav clip into a string
    and returns the hex string.
    Args:

    Returns:
        A hex string with the audio information.
    '''
    with open(sf_config.TEST.WAV_SAMPLE, 'r') as wav_fd:
        clip = wav_fd.read()
    return clip


def write_wav(byte_str, wav_file):
    '''
    Write a hex string into a wav file

    Args:
        byte_str: The hex string containing the audio data
        wav_file: The output wav file

    Returns:
    '''
    with open(wav_file, 'w') as f:
        f.write(byte_str)
