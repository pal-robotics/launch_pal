# Copyright (c) 2026 PAL Robotics S.L. All rights reserved.
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

from dataclasses import dataclass
import unittest

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_pal.arg_utils import LaunchArgumentsBase


@dataclass(frozen=True, kw_only=True)
class BaseLaunchArguments(LaunchArgumentsBase):
    base_arg: DeclareLaunchArgument = DeclareLaunchArgument(
        name='base_arg', default_value='base_default', description='base arg')


@dataclass(frozen=True, kw_only=True)
class DerivedLaunchArguments(BaseLaunchArguments):
    derived_arg: DeclareLaunchArgument = DeclareLaunchArgument(
        name='derived_arg', default_value='derived_default', description='derived arg')


class TestLaunchArgumentsBase(unittest.TestCase):

    def test_add_to_launch_description_includes_inherited_arguments(self):
        # Regression test: LaunchArgumentsBase subclasses composed through
        # inheritance (e.g. lyrical's launch arguments) must contribute both
        # their own and their parent's DeclareLaunchArgument fields.
        launch_arguments = DerivedLaunchArguments()
        launch_description = LaunchDescription()

        launch_arguments.add_to_launch_description(launch_description)

        argument_names = [action.name for action in launch_description.entities]
        self.assertIn('base_arg', argument_names)
        self.assertIn('derived_arg', argument_names)
