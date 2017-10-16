import os

import glob2
import numpy as np
from sklearn.datasets import load_svmlight_file

BASE_PATH = "/home/christos/datasets/IEMOCAP"
UTTERANCES_PARTIAL_PATH = "Statistical_Features_for_all_IEMOCAP/seglen_3/"
INDEX_PARTIAL_PATH = "All_labels_and_features_with_ids/data/" \
                     "IEMOCAP_wavid_emotion_valence_arousal_dominance.txt"

UTTERANCES_PATH = os.path.join(BASE_PATH, UTTERANCES_PARTIAL_PATH)
INDEX_FILE = os.path.join(BASE_PATH, INDEX_PARTIAL_PATH)


def parse_index(index_file):
    with open(index_file, 'r') as f:
        utterances = [utterance.strip().split(',')
                      for utterance in f.readlines()]
        index = {
            utt[0]: {
                'emotion': utt[1],
                'valence': float(utt[2]),
                'arousal': float(utt[3]),
                'dominance': float(utt[4])
            }
            for utt in utterances
        }
    return index


def parse_feature_file(feature_file):
    x, _ = load_svmlight_file(feature_file)
    return x.toarray().flatten()


def parse_utterance(utterance_path):
    utterance = []
    append = utterance.append
    segment_files = glob2.glob('{0}/*'.format(utterance_path))
    segment_files.sort(key=lambda x: int(x.split('/')[-1].split('_')[1]))
    for segment_file in segment_files:
        segment_features = parse_feature_file(segment_file)
        append(segment_features)
    return np.array(utterance)


def parse_utterances(utterances_path, index_file):
    for utterance_id in index_file.keys():
        utterance = parse_utterance(os.path.join(utterances_path,
                                                 utterance_id))
        if utterance.size > 0:
            yield utterance_id, utterance, index_file[utterance_id]
        else:
            print("{} is empty!".format(utterance_id))


def get_emotion_data():
    index = parse_index(INDEX_FILE)
    return parse_utterances(UTTERANCES_PATH, index)
