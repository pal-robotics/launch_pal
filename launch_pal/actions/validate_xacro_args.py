# Copyright 2019 Open Source Robotics Foundation, Inc.
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

"""Module for the Shutdown action."""

import logging
from typing import Dict
import xml.etree.ElementTree as ET

from launch.actions import EmitEvent
from launch.events import Shutdown as ShutdownEvent
from launch.launch_context import LaunchContext

_logger = logging.getLogger(name='launch')


class ValidateXacroArgs(EmitEvent):
    """Action that check that all the launch arguments match the xacro args."""

    def __init__(self, *, xacro_path, xacro_input_args: Dict, **kwargs):
        self.xacro_path = xacro_path
        self.xacro_input_args = list(xacro_input_args.keys())
        super().__init__(event=ShutdownEvent(reason='Xacro arguments not valid'), **kwargs)

    def execute(self, context: LaunchContext):
        """Execute the action."""
        namespaces = {'xacro': 'http://ros.org/wiki/xacro'}

        with open(self.xacro_path) as xacro_entrypoint:

            # Extract the name of all the xacro:arg tags in the entrypoint
            xacro_entrypoint_str = xacro_entrypoint.read()
            robot_xml = ET.fromstring(xacro_entrypoint_str)
            expected_args = robot_xml.findall(".//xacro:arg", namespaces=namespaces)
            expected_args_names = [arg.get('name') for arg in expected_args]

            # Fail if one of the input arguments doesn't exist in the xacro
            for input_arg_name in self.xacro_input_args:
                if input_arg_name not in expected_args_names:
                    _logger.error(f"{input_arg_name} is not an expected xacro argument")
                    super().execute(context)

            # Give a warning if one of the expected arguments is not present (maybe intentional)
            for expected_arg_name in expected_args_names:
                if expected_arg_name not in self.xacro_input_args:
                    _logger.warn(
                        f"{expected_arg_name} is expected in the xacro but not present in the input arguments, \
                        the default value will be used")
