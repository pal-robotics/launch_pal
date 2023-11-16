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

from typing import List, Dict, Optional
import copy
from launch import SomeSubstitutionsType
from launch.actions import IncludeLaunchDescription, GroupAction, DeclareLaunchArgument
from launch import Action
from launch import Condition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def include_launch_py_description(
        pkg_name: SomeSubstitutionsType,
        paths: List[SomeSubstitutionsType],
        **kwargs) -> Action:
    """
    Return IncludeLaunchDescription for the file inside pkg at paths.

    Example:
    -------
        include_launch_py_description('my_pkg', ['launch', 'my_file.launch.py'])
        returns file IncludeLaunchDescription from PATH_TO_MY_PKG_SHARE/launch/my_file.launch.py

    """
    pkg_dir = FindPackageShare(pkg_name)
    full_path = PathJoinSubstitution([pkg_dir] + paths)

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            full_path),
        **kwargs)


def include_scoped_launch_py_description(
        pkg_name: SomeSubstitutionsType,
        paths: List[SomeSubstitutionsType],
        launch_args: List[DeclareLaunchArgument] = [],
        launch_configurations: Dict = {},
        condition: Optional[Condition] = None,
        **kwargs) -> Action:
    """

    Return a GroupAction for the launch file inside pkg at paths.

    Example:
    -------
        include_scoped_launch_py_description('my_pkg', ['launch', 'my_file.launch.py'],
        launch_args= [List of DeclareLaunchArgument(arg_a)] that are required for the launch file.
          Environment variables should be passeed here as well,
        launch_configuration={'arg_b':  LaunchConfiguration('arg_a')}: If the given launch argument
          needs to be renamed for the included launch file,
        condition=IfCondition(LaunchConfiguration('arg_a')): set a specific condition for loading
          this launch file,
        returns a scoped python launch file
    """

    # In case the given launch configuration contain substitutions,
    # get the launch configs for these substitutions as well.
    updated_launch_configs = get_nested_launch_configurations(
        launch_configurations)

    launch_file = include_launch_py_description(
        pkg_name, paths,
        launch_arguments=updated_launch_configs.items(),
        **kwargs)

    actions = []

    # First add launch argument if provided
    actions.extend(launch_args)
    # Add the launch file
    actions.append(launch_file)

    scoped_launch_file = GroupAction(actions,
                forwarding=False,
                condition=condition,
                launch_configurations=updated_launch_configs)

    return scoped_launch_file


def get_nested_launch_configurations(configuration_list: Dict):

    nested_launch_configs = {}
    nested_launch_configs = nested_launch_configs | configuration_list

    for config_value in configuration_list.values():
        if not hasattr(config_value, 'substitutions'):
            continue

        substitutions = copy.deepcopy(config_value.substitutions)
        while substitutions:
            sub = substitutions.pop()
            if isinstance(sub, LaunchConfiguration):
                nested_launch_configs = {sub.variable_name[0].text: sub} | nested_launch_configs

            if hasattr(sub, 'expression'):
                substitutions.extend(sub.expression)

    return nested_launch_configs
