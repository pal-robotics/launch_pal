# Copyright 2024 Open Source Robotics Foundation, Inc.
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

"""Module for the ValidateLaunchArgs action."""

import logging
from dataclasses import fields

from launch.actions import EmitEvent
from launch.events import Shutdown as ShutdownEvent
from launch.launch_context import LaunchContext

_logger = logging.getLogger(name='launch')


class ValidateLaunchArgs(EmitEvent):
    """Checks that all the passed arguments are declared in the launch file."""

    def __init__(self, *, launch_args, **kwargs):
        self.launch_args = launch_args
        super().__init__(event=ShutdownEvent(reason='Launch arguments not valid'), **kwargs)

    def execute(self, context: LaunchContext):
        """Execute the action."""
        # Extract all the arguments names
        launch_args_names = [arg.name for arg in fields(self.launch_args)]
        input_args_names = [s.split(':=')[0] for s in context.argv]

        # Check if all the input args are in the launch_arg list
        for input_arg in input_args_names:
            if input_arg not in launch_args_names:
                _logger.error(f"{input_arg} is not a valid launch argument")
                super().execute(context)
