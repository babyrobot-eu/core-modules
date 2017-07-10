import functools
import logging
import time
import yaml


logger = logging.getLogger(__name__)


def yaml2dict(filename):
    with open(filename) as f:
        yamldict = yaml.safe_load(f)
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
            logger.error("You must pass either ymldict or filename kwargs")
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
        logger.info('{f}(*{a}, **{kw}) took: {t} sec'.format(
            f=func.__name__, a=args, kw=kwargs, t=elapsed))
        return result, elapsed
    return timed
