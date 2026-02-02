# Copyright (c) 2025 PAL Robotics S.L. All rights reserved.
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
import tempfile
from typing import Dict, Text

from launch.actions import DeclareLaunchArgument
from launch.launch_context import LaunchContext
from launch.substitution import Substitution
from launch.substitutions import LaunchConfiguration
from launch.utilities import perform_substitutions
import yaml


class RobotInfoFile(Substitution):
    """
    A substitution that creates a temporary YAML file with robot information.

    This class is used to generate a temporary file containing robot parameters
    based on the provided content dictionary. The file is created in a temporary
    directory and is intended to be used for launching robot-related nodes
    with the necessary parameters.
    """

    def __init__(self, content: Dict) -> None:
        """Create a RobotInfoFile substitution."""
        self.content = content
        self.robot_info = {
            'robot_info_publisher': {'ros__parameters': {}}
        }
        super().__init__()

    def describe(self) -> Text:
        """Return a description of this substitution as a string."""
        return 'RobotInfoFile({})'.format(self.content)

    def perform(self, context: LaunchContext) -> Text:
        """Perform the substitution by loading the file."""
        for name, arg_value in self.content.items():
            if isinstance(arg_value, DeclareLaunchArgument):
                arg_value = LaunchConfiguration(arg_value.name)
            if isinstance(arg_value, Substitution):
                arg_value = perform_substitutions(context, [arg_value])
            if isinstance(arg_value, str) and arg_value in ('True', 'False'):
                # robot_info_publisher expects plain booleans
                arg_value = (arg_value == 'True')

            self.robot_info['robot_info_publisher']['ros__parameters'].update({name: arg_value})

        temp_yaml = tempfile.mkdtemp()
        # robot_info_publisher excpects a file named 99_robot_info.yaml
        temp_robot_info = os.path.join(temp_yaml, '99_robot_info.yaml')
        with open(temp_robot_info, 'w') as temp_robot_info_file:
            yaml.safe_dump(self.robot_info, temp_robot_info_file)
        return temp_yaml
