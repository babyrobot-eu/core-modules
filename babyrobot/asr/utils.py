from babyrobot.asr import config as asr_config


def mock_audio_segment():
    """
    Mock an audio segment. Reads a test wav clip into a string
    and returns the hex string.
    Args:

    Returns:
        A hex string with the audio information.
    """
    with open(asr_config.TEST.WAV_SAMPLE, 'r') as wav_fd:
        clip = wav_fd.read()
    return clip
