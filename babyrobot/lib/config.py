import os
import six

import babyrobot.lib.utils as br_utils


class Config(object):
    """
    Parse a YAML configuration into a python object.
    The YAML properties will be stored in the Config class attributes.
    For the example YAML file (e.g., config.yaml):

    item1:
      property1: value1
      property2:
        subproperty: value2

    item2:
      moduleproperty: value3

    Usage:
    from babyrobot.lib import config as br_config
    CONF = br_config.Config(config_file='config.yaml')
    print(CONF.item1.property1)
    >>> value1
    print(CONF.item1.property2.subproperty)
    >>> value2
    print(CONF.item2.moduleproperty)
    >>> value3
    """
    def __init__(self, config_file=None, config_dict=None):
        try:
            self.config_file = (os.environ['BABYROBOT_CONFIG']
                                if config_file is None else config_file)
            if config_dict:
                self._parse(config_dict)
            else:
                self.yaml_conf = br_utils.yaml2dict(self.config_file)
                self._parse(self.yaml_conf)
        except Exception as error:
            raise Exception('Parse config failed: {}'.format(str(error)))

    def _parse(self, left_parametes):
        for param_n, param_v in six.iteritems(left_parametes):
            if isinstance(param_v, dict):
                # Recurse creating a new object
                self.__setattr__(
                    param_n,
                    Config(config_file=self.config_file, config_dict=param_v))
            else:
                self.__setattr__(param_n, param_v)
