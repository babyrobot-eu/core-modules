import functools
import logging
import shutil
import subprocess
import sys
import time
import urllib
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
    logger.debug("Running [{command}] returns: [{rc}]{output}{error}".format(
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

    with open(dest, 'wb') as f:
        shutil.copyfileobj(response, f)
    return True


def suppress_print(func):
    @functools.wraps(func)
    def func_wrapper(*args, **kwargs):
        with open('/dev/null', 'w') as sys.stdout:
            ret = func(*args, **kwargs)
        sys.stdout = sys.__stdout__
        return ret
    return func_wrapper
