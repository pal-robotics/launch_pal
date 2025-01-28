# Copyright (c) 2023 PAL Robotics S.L. All rights reserved.
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

import os
from pathlib import Path
import yaml

import ament_index_python as aip
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration

from launch_pal.param_utils import _merge_dictionaries

DEFAULT_USER_PARAMETER_PATH = Path(os.environ["HOME"], ".pal", "config")


def get_dotted_value(dotted_arg: str, d: dict):
    """Access a nested dictionary member using a dotted string."""
    for key in dotted_arg.split('.'):
        if key in d:
            d = d.get(key)
        else:
            return None
    return d


def merge_template(d: dict[str, dict], templates: dict[str, Path], ld: LaunchDescription = None):
    """
    For each node in the dictionary merge, if existing, the template into the node.
    
    The template is indicated by the 'template' key in the node dictionary,
    which is removed after the merge.
    First the template is applied, then the node dictionary elements override the template ones.
    """
    tmpl_used_per_node: dict[str, str] = {}
    for node_name, node_data in d.items():
        if 'template' in node_data.keys():
            t = node_data['template']
            if isinstance(t, str) and t in templates:
                with open(templates[t], 'r') as f:
                    node_data = _merge_dictionaries(yaml.load(f, yaml.Loader), node_data)
                    node_data.pop('template')
                d[node_name] = node_data
                tmpl_used_per_node[node_name] = t
            else:
                if ld:
                    ld.add_action(LogInfo(msg=f'WARN: template {t} not found. Skipping it.'))
    return d, tmpl_used_per_node


def merge_configs(config: dict[str, dict], srcs: dict[str, Path], templates: dict[str, Path],
                  ld: LaunchDescription = None):
    """
    Merge YAML files into a single config dictionary.
    
    The configuration files are merged in the order they are provided in the srcs dictionary.
    The single elements of the later ones override the previous ones.
    """
    src_used: dict[str, list[Path]] = {}
    tmpl_used: dict[Path, dict[str, str]] = {}
    for cfg_file in sorted(srcs.keys()):
        cfg_path = srcs[cfg_file]
        with open(cfg_path, 'r') as f:
            data = yaml.load(f, yaml.Loader)
            data, tmpl_used[cfg_path] = merge_template(data, templates, ld)
            for node in data.keys():
                if node not in src_used:
                    src_used[node] = [cfg_path]
                else:
                    src_used[node].append(cfg_path)
            config = _merge_dictionaries(config, data)
    return config, src_used, tmpl_used


def get_pal_configuration(pkg, node, ld=None, cmdline_args=True):
    """
    Get the configuration for a node from the PAL configuration files.

    :param pkg: The package name
    :param node: The node name
    :param ld: The launch description to log messages to.
                If None, no messages are logged.
    :param cmdline_args: A boolean or a list of arguments that will be added as command-line launch
                arguments. If True (default), all arguments will be added. If False, none will be
                added. If a list, only the arguments in the list will be added.

    :return: A dictionary with the parameters, remappings and arguments
    """
    PAL_USER_PARAMETERS_PATH = Path(os.environ.get(
        'PAL_USER_PARAMETERS_PATH', DEFAULT_USER_PARAMETER_PATH))

    if not PAL_USER_PARAMETERS_PATH.exists():
        if ld:
            ld.add_action(LogInfo(
                msg='WARNING: user configuration path '
                f'{PAL_USER_PARAMETERS_PATH} does not exist. '
                'User overrides will not be available.'))

    # load the package templates
    tmpl_srcs_pkgs = aip.get_resources(f'pal_configuration_templates.{pkg}')
    tmpl_srcs = {}
    for tmpl_srcs_pkg, _ in tmpl_srcs_pkgs.items():
        tmpl_files, _ = aip.get_resource(
            f'pal_configuration_templates.{pkg}', tmpl_srcs_pkg)
        for tmpl_file in tmpl_files.strip().split('\n'):
            share_path = aip.get_package_share_path(tmpl_srcs_pkg)
            path = share_path / tmpl_file
            if not path.exists():
                if ld:
                    ld.add_action(LogInfo(msg=f'WARNING: template file {path} does not exist.'
                                          ' Skipping it.'))
                continue
            if path.name in tmpl_srcs:
                if ld:
                    ld.add_action(LogInfo(msg='WARNING: two packages provide the same'
                                          f' template {path.name} for {pkg}:'
                                          f' {tmpl_srcs[path.name]} and {path}. Skipping {path}'))
                continue
            tmpl_srcs[path.name] = path

    # use ament_index to retrieve all configuration files pertaining to a pkg
    cfg_srcs = {}
    cfg_srcs_pkgs = aip.get_resources(f'pal_configuration.{pkg}')
    for cfg_srcs_pkg, _ in cfg_srcs_pkgs.items():
        cfg_files, _ = aip.get_resource(
            f'pal_configuration.{pkg}', cfg_srcs_pkg)
        for cfg_file in cfg_files.strip().split('\n'):
            share_path = aip.get_package_share_path(cfg_srcs_pkg)
            path = share_path / cfg_file
            if not path.exists():
                if ld:
                    ld.add_action(LogInfo(msg=f'WARNING: configuration file {path} does not exist.'
                                          ' Skipping it.'))
                continue
            if path.name in cfg_srcs:
                if ld:
                    ld.add_action(LogInfo(msg='WARNING: two packages provide the same'
                                          f' configuration {path.name} for {pkg}:'
                                          f' {cfg_srcs[path.name]} and {path}. Skipping {path}'))
                continue
            cfg_srcs[path.name] = path

    # retrieve all user configuration files
    user_cfg_srcs = {}
    if PAL_USER_PARAMETERS_PATH.exists():
        # list of (*.yml, *.yaml) in any subdirectory under PAL_USER_PARAMETERS_PATH:
        all_user_cfg_paths = {(e, f) for e in ["*.yml", "*.yaml"]
                              for f in PAL_USER_PARAMETERS_PATH.glob("**/" + e)}
        for _, path in all_user_cfg_paths:
            with open(path, 'r') as f:
                content = yaml.load(f, yaml.Loader)
                if not content or not isinstance(content, dict):
                    if ld:
                        ld.add_action(LogInfo(msg=f'WARN: configuration file {path.name} is empty'
                                              ' or not a dictionary. Skipping it.'))
                    continue
                user_cfg_srcs[path.name] = path

    # load and merge the configuration files
    config = {}
    config, sys_used_src, sys_used_tmpl = merge_configs(config, cfg_srcs, tmpl_srcs, ld)
    config, user_used_src, user_used_tmpl = merge_configs(config, user_cfg_srcs, tmpl_srcs, ld)

    if not config:
        return {'parameters': [], 'remappings': [], 'arguments': []}

    # finally, return the configuration for the specific node
    node_fqn = None
    for k in config.keys():
        if k.split('/')[-1] == node:
            if not node_fqn:
                node_fqn = k
            else:
                if ld:
                    ld.add_action(LogInfo(msg=f'WARN: found two configuration '
                                          'files with node {node} in different namespaces: '
                                          f'{node_fqn} and {k}.'
                                          f' Ignoring {k} for now, but you probably '
                                          'have an error in your configuration files.'))

    if not node_fqn:
        if ld:
            ld.add_action(LogInfo(msg='ERROR: configuration files found, but'
                                  f' node {node} has no entry!\nI looked into the following'
                                  ' configuration files:'
                                  f' {[str(p) for k, p in cfg_srcs.items()]}\n'
                                  ' Returning empty parameters/remappings/arguments'))
        return {'parameters': [], 'remappings': [], 'arguments': []}

    if cmdline_args:
        if ld is None:
            raise ValueError(
                "cmdline_args can only be used if argument 'ld' is not None")

        # if cmdline_args is True, add all arguments
        if not isinstance(cmdline_args, list):
            cmdline_args = config[node_fqn].setdefault(
                "ros__parameters", {}).keys()

        for arg in cmdline_args:
            default = get_dotted_value(arg, config[node_fqn].setdefault(
                "ros__parameters", {}))
            if default is None:
                ld.add_action(LogInfo(msg=f"WARNING: no default value defined for cmdline "
                                          f"argument '{arg}'. As such, it is mandatory to "
                                          "set this argument when launching the node. Consider "
                                          "adding a default value in the configuration file of "
                                          "the node."))

            ld.add_action(DeclareLaunchArgument(
                arg,
                description=f"Start node and run 'ros2 param describe {node} {arg}' for more "
                            "information.",
                default_value=str(default)))
            config[node_fqn]["ros__parameters"][arg] = LaunchConfiguration(arg, default=[default])

    res = {'parameters': [dict(config[node_fqn].setdefault('ros__parameters', {}))],
           'remappings': list(config[node_fqn].setdefault('remappings', {}).items()),
           'arguments': config[node_fqn].setdefault('arguments', []),
           }

    if not isinstance(res['arguments'], list):
        res['arguments'] = []
        if ld:
            ld.add_action(LogInfo(msg='ERROR: \'arguments\' field in configuration'
                                  f' for node {node} must be a _list_ of arguments'
                                  ' to be passed to the node. Ignoring it.'))

    def str_config(used_srcs: dict[str, list[Path]], used_tmpl: dict[Path, dict[str, str]],
                   node_fqn: str):
        return ('\n'.join([f'\t- {src}'
                          + (('\n' + '\n'.join([f"\t\t- {t}"
                              for t in used_tmpl[src].values()])
                              ) if node_fqn in used_tmpl[src] else '')
                          for src in reversed(used_srcs[node_fqn])]
                          ) if node_fqn in used_srcs else "\t- (none)")

    if ld:
        ld.add_action(
            LogInfo(msg=f'Loaded configuration for <{node}>:'
                    '\n- User overrides '
                    '(from higher to lower precedence, listing the used templates indented):\n'
                    + str_config(user_used_src, user_used_tmpl, node_fqn) +
                    '\n- System configuration '
                    '(from higher to lower precedence, listing the used templates indented):\n'
                    + str_config(sys_used_src, sys_used_tmpl, node_fqn)
                    ))
        if res['parameters']:
            # create an empty launch context to get the default values of the parameters
            lc = LaunchContext()

            param_list = ""
            for k, v in res['parameters'][0].items():
                if isinstance(v, LaunchConfiguration):
                    param_list += f'- {k}: {v.perform(lc)} (can be overridden with {k}:=...)\n'
                else:
                    param_list += f'- {k}: {v}\n'
            ld.add_action(LogInfo(msg='Parameters:\n' + param_list))
        if res['remappings']:
            ld.add_action(LogInfo(msg='Remappings:\n' +
                                  '\n'.join([f'- {a} -> {b}' for a, b in res['remappings']])))
        if res['arguments']:
            ld.add_action(LogInfo(msg='Arguments:\n' +
                                  '\n'.join([f"- {a}" for a in res['arguments']])))

    return res
