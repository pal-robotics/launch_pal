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
from typing import Text, Dict
import xml.etree.ElementTree as ET


from launch.frontend import Entity
from launch.frontend import expose_action
from launch.frontend import Parser

from launch.actions import EmitEvent
from launch.events import Shutdown as ShutdownEvent
from launch.events.process import ProcessExited
from launch.launch_context import LaunchContext

_logger = logging.getLogger(name='launch')


class ValidateXacroArgs(EmitEvent):
    """Action that check that all the launch arguments match the xacro args."""

    def __init__(self, *, msg: Text = 'Arguments do not match', xacro_path, args_dict: Dict, **kwargs):
        self.xacro_path = xacro_path
        self.args_dict = args_dict
        super().__init__(event=ShutdownEvent(reason=msg), **kwargs)

    @classmethod
    def parse(cls, entity: Entity, parser: Parser):
        """Return `ValidateXacroArgs` action and kwargs for constructing it."""
        _, kwargs = super().parse(entity, parser)
        msg = entity.get_attr('reason', optional=True)
        if msg:
            kwargs['msg'] = parser.parse_substitution(msg)

        xacro_path = entity.get_attr('xacro_path', optional=False)
        kwargs['xacro_path'] = parser.parse_substitution(xacro_path)

        args_dict = entity.get_attr('args_dict', optional=False)
        kwargs['args_dict'] = parser.parse_substitution(args_dict)

        return cls, kwargs

    def execute(self, context: LaunchContext):
        """Execute the action."""

        namespaces = {'xacro': 'http://ros.org/wiki/xacro'}

        with open(self.xacro_path) as xacro_entrypoint:

            xacro_entrypoint_str = xacro_entrypoint.read()

            # Extract the name of all the xacro:arg tags in the entrypoint
            robot_xml = ET.fromstring(xacro_entrypoint_str)
            xacro_declared_args = robot_xml.findall(".//xacro:arg", namespaces=namespaces)
            declared_arguments = [arg.get('name') for arg in xacro_declared_args]

            for arg in self.args_dict.keys():
                if arg not in declared_arguments:
                    _logger.error(f"{arg} is not a valid xacro argument")
                    super().execute(context)
