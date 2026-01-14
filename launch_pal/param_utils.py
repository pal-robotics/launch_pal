# Copyright (c) 2022 PAL Robotics S.L. All rights reserved.
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
import re
import tempfile
from typing import Dict, List, Text

import ament_index_python as aip

from launch import LaunchDescription
from launch.actions import LogInfo

import yaml


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
    if not isinstance(data, dict):
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
        if isinstance(e, str):
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


def _parse_config(path, param_rewrites):
    """Use substitute_variables instead."""
    data, _ = substitute_variables(path, param_rewrites)
    return data


def substitute_variables(file_path: str, variables: dict, ld: LaunchDescription = None):
    """
    Load a yaml configuration file and resolve any variables.

    It allows to get the share directory of a package too.

    The variables must be in this format to be parsed:
    ${VAR_NAME}.
    E.g.:
    host: ${HOST}
    The pkg name must be in this format to be parsed:
    ${find PKG_NAME}.
    E.g.:
    pkg_path:${find PKG_NAME}

    Parameters
    ----------
    file_path : Text
        The path to the yaml file.
    variables : Dict
        Dictionary with the substitution variables.
    ld : LaunchDescription
        The launch description to log messages to.
        If None, no messages are logged.

    Returns
    -------
        A parsed dictionary where the matched variables are substituted.
        A dictionary with the matched variables and their values.

    """
    var_pattern = re.compile(r'\$\{([^}]+)\}')  # matches ${...}
    find_pkg_pattern = re.compile(r'\$\{find ([^}]+)\}')  # matches ${find ...}

    matched_vars = {}

    # read the YAML file
    with open(file_path, 'r') as file:
        content = file.read()

    # Replace all occurrences of the package pattern with the full path of the package
    def replace_pkg_path(match):
        pkg = match.group(1)
        try:
            pkg_path = aip.get_package_share_directory(pkg)
            matched_vars[match.group(0)] = pkg_path
        except aip.PackageNotFoundError:
            if ld:
                ld.add_action(LogInfo(msg='WARNING: during variable substitution,'
                                          f' package {pkg} not found. Ignoring it.'))
        # Assuming pkg_path is not empty if the path exists
        return pkg_path

    content = find_pkg_pattern.sub(replace_pkg_path, content)

    # Replace all matches of the pattern with their corresponding values from param_rewrites
    def replace_variables(match):
        var = match.group(1)
        # Check if the variable exists in the substitution dictionary
        if var in variables:
            matched_vars[match.group(0)] = variables[var]
            return str(variables[var])
        else:
            if ld:
                ld.add_action(LogInfo(msg='WARNING: during variable substitution,'
                                          f' variable {var} not found in robot info.'
                                          ' Ignoring it.'))

    content = var_pattern.sub(replace_variables, content)
    return yaml.safe_load(content), matched_vars


def parse_parametric_yaml(
    source_files: List[Text], param_rewrites: Dict, ld: LaunchDescription = None
):
    """
    Parse a list of Parametric YAML files into a single one.

    Substitutes parameters into several different YAML files in the form

    parameter_node:
        ros__parameters:
            parameter_name: ${PARAMETER_VALUE}

    by taking the value of the ${PARAMETER_VALUE} variable from the
    param_rewrites Dictionary

    Parameters
    ----------
    source_files : List[Text]
        List of paths of Parametric YAML files.
    param_rewrites : Dict
        Dictionary with the name and the value of
        each variable.
    ld : LaunchDescription
        The launch description to log messages to.
        If None, no messages are logged.

    Returns
    -------
        rewritten_yaml: Text
        Path to the Full YAML file containing all the parameters.

    """
    rewritten_yaml = tempfile.NamedTemporaryFile(mode='w', delete=False)
    full_yaml = {}

    for source_file in source_files:
        data, _ = substitute_variables(source_file, param_rewrites, ld)
        full_yaml.update(data)

    yaml.dump(full_yaml, rewritten_yaml)
    rewritten_yaml.close()
    return rewritten_yaml.name
