# Copyright (c) 2021 PAL Robotics S.L.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import copy

import yaml
import tempfile


def _merge_dictionaries(dict1, dict2):
    """
    Recursive merge dictionaries.

    :param dict1: Base dictionary to merge.
    :param dict2: Dictionary to merge on top of base dictionary.
    :return: Merged dictionary
    """
    for key, val in dict1.items():
        if isinstance(val, dict):
            dict2_node = dict2.setdefault(key, {})
            _merge_dictionaries(val, dict2_node)
        else:
            if key not in dict2:
                dict2[key] = val

    return dict2


def insert_ros_param_prefix(data, prefix):
    if type(data) != dict:
        return data

    for k in data.keys():
        if k == 'ros__parameters':
            d = {}
            d[prefix] = copy.deepcopy(data[k])
            data[k] = d
        else:
            data[k] = insert_ros_param_prefix(data[k], prefix)
    return data


def merge_param_files(yaml_files):
    """
    Merge multiple param yaml files.

    Substitution in ROS2 launch can only return a string. The way to combine multiple parameter
    files is to create a single temporary file and return the path to it, this path is passed as
    the "parameters" argument of a Node

    yaml_files is a list of either paths to yaml files or pairs of two strings (path, prefix),
    so the file is loaded inside the provided prefix, inside the ros__parameters field
    """
    concatenated_dict = {}
    for e in yaml_files:
        if type(e) == str:
            yaml_file = e
            prefix = None
        else:
            yaml_file = e[0]
            prefix = e[1]
        data = yaml.safe_load(open(yaml_file, 'r'))
        if prefix:
            data = insert_ros_param_prefix(data, prefix)

        _merge_dictionaries(concatenated_dict, data)
        # Second arg takes precedence on merge, and result is stored there
        concatenated_dict = data
    rewritten_yaml = tempfile.NamedTemporaryFile(mode='w', delete=False)
    yaml.dump(concatenated_dict, rewritten_yaml)
    rewritten_yaml.close()
    return rewritten_yaml.name
