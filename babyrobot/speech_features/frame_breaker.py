import numpy as np


def pad_signal(s, padded_size):
    if s.size == padded_size:
        return s
    if s.size > padded_size:
        return s[:padded_size]
    return np.append(s, np.zeros(padded_size - s.size))


def get_frames(s,
               frame_duration=0.5,
               frame_stride=0.4,
               sample_rate=16000,
               max_sample_size=-1):
    if max_sample_size < 0:
        sample_size = s.size
    else:
        sample_size = max_sample_size
    # convert to mono
    if s.ndim > 1:
        s = s[:, 0]
    frame_size = int(frame_duration * sample_rate)
    frame_step = int(frame_stride * sample_rate)
    N_f = int(np.ceil((sample_size - frame_size) / frame_step))

    padded_size = N_f * frame_step + frame_size

    indices = np.tile(np.arange(0, frame_size), (N_f, 1))
    stride = np.reshape(
        np.arange(0, N_f * frame_step, frame_step), (N_f, 1))
    frame_indices = (indices + stride).astype(np.int32, copy=False)

    s_padded = pad_signal(s, padded_size)
    return s_padded[frame_indices]
