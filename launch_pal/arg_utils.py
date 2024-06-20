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

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.utilities import perform_substitutions
from launch.actions import DeclareLaunchArgument
from dataclasses import dataclass



@dataclass(frozen=True, kw_only=True)
class LaunchArgumentsBase:
    """This class is a dataclass containing only DeclareLaunchArgument objects."""

    def __init_subclass__(cls, **kwargs):
        annotations = getattr(cls, '__annotations__', {})
        for attr, type_ in annotations.items():
            if not issubclass(type_, DeclareLaunchArgument):
                raise TypeError(
                    f"All attributes in dataclass {cls.__name__} must have type \
                          DeclareLaunchArgument")

    def add_to_launch_description(self, launch_description: LaunchDescription):
        """
        Load a yaml configuration file given by the robot name.

        Parameters
        ----------
        launch_description : LaunchDescription
            The launch description that the Launch Arguments will be added to


        """
        annotations = getattr(self, '__annotations__', {})
        for attr, type_ in annotations.items():
            launch_description.add_action(getattr(self, attr))
        return


def read_launch_argument(arg_name, context):
    """
    Use in Opaque functions to read the value of a launch argument.

    Parameters
    ----------
    arg_name : String
        Name of the launch argument
    context : LaunchContext
        The launch context

    Returns
    -------
    value : String
        The value of the launch argument

    """
    return perform_substitutions(context,
                                 [LaunchConfiguration(arg_name)])
