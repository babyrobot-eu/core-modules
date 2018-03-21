import os
from collections import defaultdict
import csv


def read_msol():
    '''
    Macquarie Semantic Orientation Lexicon (MSOL) (76.400 entries)
    --------------------------------------
    format = dictionary with entries like this:
    word1={'positive': 1, 'negative': 0}
    word2={'positive': 0, 'negative': 1}
    '''
    fname = os.path.join(os.path.dirname(__file__), "MSOL-June15-09.txt")
    with open(fname, "r") as f:
        lines = f.read().splitlines()
        entries = [line.split("\t") for line in lines]
        lexicon = defaultdict()
        for entry in entries:
            lexicon[entry[0].replace("_", " ")] = {
                "positive": 1 if entry[1] == "positive" else 0,
                "negative": 1 if entry[1] == "negative" else 0,
                "polarity": 1 if entry[1] == "positive" else -1
            }
        return lexicon


def read_bing_liu():
    """
    Opinion Lexicon (or Sentiment Lexicon) -  Bing Liu (~6.800 entries)
    --------------------------------------
    format = dictionary with entries like this:
    word1={'positive': 1, 'negative': 0}
    word2={'positive': 0, 'negative': 1}
    :return:
    """
    fp = os.path.join(os.path.dirname(__file__), "positive-words.txt")
    fn = os.path.join(os.path.dirname(__file__), "positive-words.txt")

    lexicon = defaultdict()

    with open(fp, "r") as raw_file:
        reader = csv.reader(raw_file)
        for i in range(35):
            next(reader)
        for row in reader:
            lexicon[row[0].replace("-", " ")] = {"positive": 1,
                                                 "negative": 0,
                                                 "polarity": 1}

    with open(fn, "r") as raw_file:
        reader = csv.reader(raw_file)
        for i in range(35):
            next(reader)
        for row in reader:
            lexicon[row[0].replace("-", " ")] = {"positive": 0,
                                                 "negative": 1,
                                                 "polarity": -1}
    return lexicon


def read_mpqa():
    """
    MPQA (Multi-Perspective Question Answering) Subjectivity Lexicon
    (8.222 entries)
    --------------------------------------
    format = dictionary with entries like this:
    ('amaze', 'VERB')={'positive': 1, 'negative': 0, 'strength': 'strongsubj'}
    :return:
    """
    fname = "subjclueslen1-HLTEMNLP05.csv"
    with open(os.path.join(os.path.dirname(__file__), fname),
              "r") as f:
        lines = f.read().splitlines()
        entries = [tuple(line.split("\t")) for line in lines]
        lexicon = defaultdict(dict)
        for entry in entries:
            priorpolarity = entry[5].split('=')[1]

            pos_map = {'noun': 'NOUN', 'verb': 'VERB', 'anypos': '_',
                       'adverb': 'ADV', 'adj': 'ADJ'}
            pos = pos_map[entry[3].split('=')[1]]

            word = entry[2].split('=')[1]
            word = word.replace("-", " ")

            valence = 0
            if priorpolarity == "positive":
                valence = 1
            elif priorpolarity == "negative":
                valence = -1
            polarity = valence

            strength = 0
            subjectivity = entry[0].split('=')[1]
            if subjectivity == "strongsubj":
                strength = 2
            elif subjectivity == "weaksubj":
                strength = 1

            valence *= strength

            lexicon[word] = {
                "positive": 1 if priorpolarity == "positive" else 0,
                "negative": 1 if priorpolarity == "negative" else 0,
                "strength": subjectivity,
                "valence": valence,
                "polarity": polarity,
                "pos": pos
            }

        for key, value in lexicon.items():
            if " " in key:
                values = list(value.values())
                lexicon[key] = {"_": values[0]}

        return lexicon


def read_affin():
    """
    AFINN is a list of English words rated for valence with an integer
    between minus five (negative) and plus five (positive). The words have
    been manually labeled by Finn Arup Nielsen in 2009-2011. The file
    is tab-separated
    2477 words and phrases.
    --------------------------------------
    format = dictionary with entries like this:
    word1={-5,5}
    :return:
    """
    filename = "AFINN-111.txt"
    with open(os.path.join(os.path.dirname(__file__), filename),
              "r") as f:
        reader = csv.reader(f, delimiter="\t")
        lexicon = defaultdict(dict)
        for row in reader:
            word = row[0]
            score = row[1]
            lexicon[word] = {
                "positive": 1 if score > 0 else 0,
                "negative": -1 if score < 0 else 0,
                "polarity": score
            }

        return lexicon
