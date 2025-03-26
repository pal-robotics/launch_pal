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

import json
import os
from pathlib import Path
import re
from tempfile import NamedTemporaryFile
import yaml

import ament_index_python as aip
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration

from launch_pal.param_utils import _merge_dictionaries, substitute_variables
import launch_pal.logging_utils as log

DEFAULT_PAL_USER_PATH = Path(os.environ["HOME"], ".pal")
SYSTEM_ROBOT_INFO_PATH = Path("/etc", "robot_info", "conf.d")


# adapted from https://stackoverflow.com/a/6027615
def flatten(dictionary: dict, parent_key: str = '', separator: str = '.'):
    items = []
    for key, value in dictionary.items():
        new_key = parent_key + separator + str(key) if parent_key else str(key)
        if isinstance(value, dict):
            items.extend(flatten(value, new_key, separator=separator).items())
        else:
            items.append((new_key, value))
    return dict(items)


def find_yaml_files_in_dir(dir: Path, recursive: bool = False):
    """Get all YAML files in a directory."""
    yaml_files = []
    if dir.exists():
        glob_f = dir.rglob if recursive else dir.glob
        yaml_files = [p for f in ["*.yml", "*.yaml"] for p in glob_f(f) if p.is_file()]
    return yaml_files


def load_pal_robot_info(ld: LaunchDescription = None):
    """Load the PAL robot_info from the configuration files."""
    robot_info = {}
    pal_user_robot_info_path = Path(os.environ.get('PAL_USER_PATH', DEFAULT_PAL_USER_PATH),
                                    'robot_info', 'conf.d')

    for path in [*sorted(find_yaml_files_in_dir(SYSTEM_ROBOT_INFO_PATH)),
                 *sorted(find_yaml_files_in_dir(pal_user_robot_info_path))]:
        with open(path, 'r') as f:
            data = yaml.load(f, yaml.Loader)
            try:
                robot_info_update = flatten(data['robot_info_publisher']['ros__parameters'])
                robot_info = _merge_dictionaries(robot_info, robot_info_update)
            except (KeyError, TypeError):
                if ld:
                    ld.add_action(LogInfo(msg=log.COLOR_YELLOW +
                                          f'WARNING: in robot info configuration {path},'
                                          ' expected item "robot_info_publisher: ros__parameters"'
                                          ' not found. Skipping it.'))
    return robot_info


def merge_preset(d: dict[str, dict], presets: dict[str, Path], ld: LaunchDescription = None):
    """
    For each node in the dictionary merge, if existing, the preset into the node.

    The preset is indicated by the 'use_preset' key in the node dictionary,
    which is removed after the merge.
    First the preset is applied, then the node dictionary elements override the preset ones.
    """
    preset_used_per_node: dict[str, str] = {}
    for node_name, node_data in d.items():
        if 'use_preset' in node_data.keys():
            p = node_data['use_preset']
            if isinstance(p, str) and p in presets:
                with open(presets[p], 'r') as f:
                    node_data = _merge_dictionaries(yaml.load(f, yaml.Loader), node_data)
                    node_data.pop('use_preset')
                d[node_name] = node_data
                preset_used_per_node[node_name] = p
            else:
                if ld:
                    ld.add_action(LogInfo(msg=log.COLOR_YELLOW +
                                          f'WARNING: preset {p} not found. Skipping it.'))
    return d, preset_used_per_node


def merge_configs(config: dict[str, dict], srcs: dict[str, Path], presets: dict[str, Path],
                  config_vars: dict, ld: LaunchDescription = None):
    """
    Merge YAML files into a single config dictionary.

    The configuration files are merged in the order they are provided in the srcs dictionary.
    The single elements of the later ones override the previous ones.
    Within a configuration file, the presets are merged before the rest of the configuration.

    If a configuration has a 'use_if' dictionary, the configuration is only used if all its items
    find a matching one in the config_vars.
    """
    src_used: dict[str, list[Path]] = {}
    src_filtered: dict[str, list[Path]] = {}
    preset_used: dict[Path, dict[str, str]] = {}
    for cfg_file in sorted(srcs.keys()):
        cfg_path = srcs[cfg_file]
        with open(cfg_path, 'r') as f:
            data = yaml.load(f, yaml.Loader)
            data, preset_used[cfg_path] = merge_preset(data, presets, ld)
            filtered_data = {}
            for node in data.keys():
                req_vars = data[node].get('use_if', {})
                is_dict = isinstance(req_vars, dict)
                if (is_dict and not all(i in config_vars.items() for i in req_vars.items())):
                    src_filtered.setdefault(node, []).append(cfg_path)
                else:
                    node_data = data[node]
                    node_data.pop('use_if', None)
                    filtered_data[node] = node_data
                    src_used.setdefault(node, []).append(cfg_path)
            config = _merge_dictionaries(config, filtered_data)
    return config, src_used, src_filtered, preset_used


def list_pal_resources(resource: str, pkg=None, ld=None):
    """
    List the available '{resource}.*' ament_index resources.

    :param resource: The resource name to search for.
    :param pkg: Only list '{resource}.{pkg}.*' resources.
                If None, all resources are listed.
    :param ld: The launch description to log messages to.
                If None, no messages are logged.
    :return: A set with the available resources.
    """
    resources = set()
    for aip_path in aip.get_search_paths():
        resources_index = os.path.join(aip_path, 'share', 'ament_index', 'resource_index')
        for entry in os.scandir(resources_index):
            # Skip if the entry is a file or doesn't match the PAL configuration pattern
            if entry.is_file() or resource not in entry.name:
                continue

            # Filter by package, if provided
            res_info = entry.name.split('.')
            if pkg and pkg != res_info[1]:
                continue

            # Add the configuration to the list
            resources.add(entry.name)

    return resources


def get_pal_resources(res_name, ld=None):
    """
    Get the specified resource from ament_index.

    :param res_name: The ament_index resource name
    :param ld: The launch description to log messages to.
                If None, no messages are logged.

    :return: A dictionary with the parameters, remappings and arguments
    """
    srcs_pkgs = aip.get_resources(res_name)
    srcs = {}
    for srcs_pkg, _ in srcs_pkgs.items():
        files, _ = aip.get_resource(res_name, srcs_pkg)
        for file in re.split('\n|;', files.strip()):
            share_path = aip.get_package_share_path(srcs_pkg)
            path = share_path / file
            if not path.exists():
                if ld:
                    ld.add_action(
                        LogInfo(msg=log.COLOR_YELLOW +
                                f'WARNING: file {path} does not exist for {res_name}.'
                                ' Skipping it.')
                    )
                continue
            if path.name in srcs:
                if ld:
                    ld.add_action(
                        LogInfo(msg=log.COLOR_YELLOW +
                                'WARNING: two packages provide the same'
                                f' name {path.name} for {res_name}:'
                                f' {srcs[path.name]} and {path}. Skipping {path}'))
                continue
            srcs[path.name] = path

    return srcs


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
    # load the configuration variables from robot info and PAL_CONFIGURATION_FLAGS
    node_sub_vars = {'parameters': {}, 'remappings': {}, 'arguments': {}}
    robot_info = load_pal_robot_info(ld)

    pal_configuration_flags = {}
    pal_configuration_flags_str = os.getenv("PAL_CONFIGURATION_FLAGS", None)
    if pal_configuration_flags_str:
        try:
            pal_configuration_flags = json.loads(pal_configuration_flags_str)
            if not isinstance(pal_configuration_flags, dict):
                raise json.JSONDecodeError('not a dictionary', pal_configuration_flags_str, 0)
        except json.JSONDecodeError:
            if ld:
                ld.add_action(LogInfo(msg=log.COLOR_RED +
                                      'ERROR: PAL_CONFIGURATION_FLAGS environmental variable is'
                                      ' not a valid JSON dictionary. Ignoring it.'))
    config_vars = {}
    config_vars = _merge_dictionaries(config_vars, robot_info)
    config_vars = _merge_dictionaries(config_vars, pal_configuration_flags)

    # load the package preset
    preset_resources = list_pal_resources('pal_configuration_presets.', pkg, ld)
    preset_results = [get_pal_resources(res, ld) for res in preset_resources]
    preset_srcs = {k: v for srcs in preset_results for k, v in srcs.items()}

    # use ament_index to retrieve all configuration files pertaining to a pkg
    cfg_resources = list_pal_resources('pal_configuration.', pkg, ld)
    cfg_results = [get_pal_resources(res, ld) for res in cfg_resources]
    cfg_srcs = {k: v for srcs in cfg_results for k, v in srcs.items()}

    # retrieve all user configuration files
    user_cfg_srcs = {}
    pal_user_parameters_path = Path(os.environ.get('PAL_USER_PATH', DEFAULT_PAL_USER_PATH),
                                    'config')

    if pal_user_parameters_path.exists():
        # list of (*.yml, *.yaml) in any subdirectory under PAL_USER_PARAMETERS_PATH:
        all_user_cfg_paths = {(p.name, p) for p in
                              find_yaml_files_in_dir(pal_user_parameters_path, True)}
        for _, path in all_user_cfg_paths:
            with open(path, 'r') as f:
                content = yaml.load(f, yaml.Loader)
                if not content or not isinstance(content, dict):
                    if ld:
                        ld.add_action(LogInfo(msg=log.COLOR_YELLOW +
                                              f'WARNING: configuration file {path.name}'
                                              ' is empty or not a dictionary. Skipping it.'))
                    continue
                user_cfg_srcs[path.name] = path
    else:
        if ld:
            ld.add_action(LogInfo(msg=log.COLOR_YELLOW +
                                  'WARNING: user configuration path '
                                  f'{pal_user_parameters_path} does not exist. '
                                  'User overrides will not be available.'))

    # load and merge the configuration files
    config = {}
    config, sys_used_src, sys_filtered_src, sys_used_preset = merge_configs(
        config, cfg_srcs, preset_srcs, config_vars, ld)
    config, user_used_src, user_filtered_src, user_used_preset = merge_configs(
        config, user_cfg_srcs, preset_srcs, config_vars, ld)
    filtered_src = _merge_dictionaries(sys_filtered_src, user_filtered_src)

    # get the configuration for the specific node
    node_fqn = None
    for k in config.keys():
        if k.split('/')[-1] == node:
            if not node_fqn:
                node_fqn = k
            else:
                if ld:
                    ld.add_action(LogInfo(msg=log.COLOR_YELLOW +
                                          'WARNING: found two configuration ' +
                                          f'files with node {node} in different namespaces: '
                                          f'{node_fqn} and {k}.'
                                          f' Ignoring {k} for now, but you probably '
                                          'have an error in your configuration files.' +
                                          log.COLOR_RESET))

    if not node_fqn:
        if ld:
            ld.add_action(LogInfo(msg=log.COLOR_RED + 'ERROR: configuration files found, but'
                                  f' node {node} has no entry!' + log.COLOR_RESET +
                                  '\nI looked into the following'
                                  ' configuration files:'
                                  f' {[str(p) for k, p in cfg_srcs.items()]}\n'
                                  ' Returning empty parameters/remappings/arguments'))
        return {'parameters': [], 'remappings': [], 'arguments': []}

    node_config = {'parameters': {}, 'remappings': {}, 'arguments': {}}

    node_config['parameters'] = flatten(config[node_fqn].get('ros__parameters', {}))

    node_config['remappings'] = config[node_fqn].get('remappings', {})
    if not isinstance(node_config['remappings'], dict):
        node_config['remappings'] = {}
        if ld:
            ld.add_action(LogInfo(msg=log.COLOR_RED +
                                  'ERROR: \'remappings\' field in configuration'
                                  f' for node {node} must be a _dictionary_ of remappings'
                                  ' to be passed to the node. Ignoring it.'))
    else:
        for k, v in node_config['remappings'].items():
            if isinstance(v, (list, dict)):
                if ld:
                    ld.add_action(LogInfo(msg=log.COLOR_RED +
                                          f'ERROR: \'remappings[{k}]\' field in configuration'
                                          f' for node {node} cannot be a list or dictionary.'
                                          ' Ignoring it.'))
                node_config['remappings'].pop(k)

    if not isinstance(config[node_fqn].get('arguments', []), list):
        if ld:
            ld.add_action(LogInfo(msg=log.COLOR_RED + 'ERROR: \'arguments\' field in configuration'
                                  f' for node {node} must be a _list_ of arguments'
                                  ' to be passed to the node. Ignoring it.'))
    else:
        # saved ad dict for easier manipulation, later converted back to list
        node_config['arguments'] = {i: v for i, v in
                                    enumerate(config[node_fqn].get('arguments', []))}

    # substitute variables using the robot info and the PAL_CONFIGURATION_FLAGS
    tmp_file = NamedTemporaryFile(mode="w", delete=False)
    yaml.dump(node_config, tmp_file)
    node_config_sub, sub_vars = substitute_variables(tmp_file.name, config_vars, ld)

    for t in ['parameters', 'remappings', 'arguments']:
        for k, v in node_config[t].items():
            for sub_var_key in sub_vars.keys():
                if isinstance(v, str) and sub_var_key in v:
                    node_sub_vars[t][k] = {v}

    # add launch command-line arguments
    if cmdline_args:
        if ld is None:
            raise ValueError(
                "cmdline_args can only be used if argument 'ld' is not None")

        # if cmdline_args is True, add all arguments
        if not isinstance(cmdline_args, list):
            cmdline_args = node_config_sub['parameters'].keys()

        for arg in cmdline_args:
            default = node_config_sub['parameters'].get(arg, None)
            if default is None:
                ld.add_action(LogInfo(msg=log.COLOR_YELLOW +
                                      f"WARNING: no default value defined for cmdline "
                                      f"argument '{arg}'. As such, it is mandatory to "
                                      "set this argument when launching the node. Consider "
                                      "adding a default value in the configuration file of "
                                      "the node."))

            ld.add_action(DeclareLaunchArgument(
                arg,
                description=f"Start node and run 'ros2 param describe {node} {arg}' for more "
                            "information.",
                default_value=str(default)))
            node_config_sub['parameters'][arg] = LaunchConfiguration(arg, default=[default])

    # log configuration
    if ld:
        if len(config_vars):
            ld.add_action(
                LogInfo(
                    msg=log.COLOR_CYAN_BOLD + 'Loaded configuration variables:' + log.COLOR_RESET +
                    '\n- From robot info:\n' + json.dumps(robot_info, indent=4) +
                    (('\n- From PAL_CONFIGURATION_FLAGS (overriding the robot info):\n' +
                     json.dumps(pal_configuration_flags, indent=4))
                     if pal_configuration_flags else '')
                ))

        def str_config(used_srcs: dict[str, list[Path]], used_presets: dict[Path, dict[str, str]],
                       node_fqn: str):
            return ('\n'.join([f'\t- {src}'
                    + (('\n' + '\n'.join([f"\t\t- {t}"
                        for t in used_presets[src].values()])
                        ) if node_fqn in used_presets.get(src, {}) else '')
                    for src in reversed(used_srcs[node_fqn])]
                    ) if node_fqn in used_srcs else "\t- (none)")

        ld.add_action(
            LogInfo(
                msg=log.COLOR_CYAN_BOLD + f'Loaded configuration for <{node}>:' + log.COLOR_RESET +
                '\n- User overrides '
                '(from higher to lower precedence, listing the used presets indented):\n'
                + str_config(user_used_src, user_used_preset, node_fqn) +
                '\n- System configuration '
                '(from higher to lower precedence, listing the used presets indented):\n'
                + str_config(sys_used_src, sys_used_preset, node_fqn) +
                ((log.COLOR_YELLOW + '\n- Filtered out configuration:\n' + log.COLOR_RESET +
                  str_config(filtered_src, {}, node_fqn))
                 if node_fqn in filtered_src else '')
            ))

        def str_sub_vars(sub_vars: set[str]):
            if not sub_vars:
                return ''
            return log.COLOR_GREEN + f' (substituted: {sub_vars})' + log.COLOR_RESET

        if node_config_sub['parameters']:
            # create an empty launch context to get the default values of the parameters
            lc = LaunchContext()

            log_param = ""
            for k, v in node_config_sub['parameters'].items():
                log_param += f'- {k}'
                if isinstance(v, LaunchConfiguration):
                    log_param += (
                        log.COLOR_YELLOW
                        + " [overridable]"
                        + log.COLOR_RESET
                        + f": {v.perform(lc)}"
                    )
                else:
                    log_param += f': {v}'
                log_param += (
                    f"{str_sub_vars(node_sub_vars['parameters'][k])}\n"
                    if k in node_sub_vars['parameters']
                    else '\n'
                )
            ld.add_action(LogInfo(msg=log.COLOR_CYAN + 'Parameters' + log.COLOR_RESET +
                                  ' (if "overridable", can be overridden with'
                                  ' <full.name>:=<value>):\n' + log_param))
        if node_config_sub['remappings']:
            ld.add_action(
                LogInfo(
                    msg="Remappings:\n" + "\n".join([
                        f"- {k} -> {v} {str_sub_vars(node_sub_vars['remappings'][k])}"
                        if k in node_sub_vars['remappings']
                        else f"- {k} -> {v}"
                        for k, v in node_config_sub['remappings'].items()
                    ])
                )
            )

        if node_config_sub['arguments']:
            ld.add_action(
                LogInfo(
                    msg="Arguments:\n" + "\n".join([
                        f"- {v} {str_sub_vars(node_sub_vars['arguments'][k])}"
                        if k in node_sub_vars['remappings']
                        else f"- {v}"
                        for k, v in node_config_sub['arguments'].items()
                    ])
                )
            )

    file_paths = [str(x) for x in user_used_src[node_fqn]] if node_fqn in user_used_src else []
    file_paths.extend([str(x) for x in sys_used_src.get(node_fqn, [])[::-1]])

    res = {'parameters': [dict(node_config_sub['parameters'])],
           'remappings': [(k, v) for k, v in node_config_sub['remappings'].items()],
           'arguments': node_config_sub['arguments'].values(),
           'file_paths': file_paths,
           }

    return res
