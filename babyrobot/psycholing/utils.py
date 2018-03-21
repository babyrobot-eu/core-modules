import babyrobot.psycholing.config as psy_config


def load_lexicon():
    with open(psy_config.PSY_LEXICON) as fd:
        lines = [l.strip().split(' ') for l in fd.readlines()]
    lexicon = {l[0]: map(int, l[1:]) for l in lines}
    return lexicon


def load_dimensions():
    with open(psy_config.PSY_DIMENSIONS) as fd:
        lines = [l.strip().split('. ') for l in fd.readlines()]
    dimensions = {l[1]: int(l[0]) - 1 for l in lines}
    return dimensions
