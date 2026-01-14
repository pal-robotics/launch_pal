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
import shutil
from typing import Any, Dict
import unittest

from launch.actions import DeclareLaunchArgument
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_pal.substitutions import RobotInfoFile
import yaml


class TestRobotInfoFile(unittest.TestCase):

    def setUp(self):
        """Set up for each test."""
        self.context = LaunchContext()
        self.temp_dirs_created = []

    def tearDown(self):
        """Clean up after each test."""
        for temp_dir in self.temp_dirs_created:
            if os.path.exists(temp_dir):
                shutil.rmtree(temp_dir)
        self.temp_dirs_created = []

    def _perform_and_check(self, robot_info_sub: RobotInfoFile, expected_params: Dict[str, Any]):
        """Perform substitution and check YAML content."""
        temp_dir_path = robot_info_sub.perform(self.context)
        self.temp_dirs_created.append(temp_dir_path)

        self.assertTrue(os.path.isdir(temp_dir_path),
                        'Perform should return a valid directory path.')

        expected_file_path = os.path.join(temp_dir_path, '99_robot_info.yaml')
        self.assertTrue(os.path.isfile(expected_file_path),
                        "Expected YAML file '99_robot_info.yaml' not found.")

        with open(expected_file_path, 'r') as f:
            loaded_data = yaml.safe_load(f)

        expected_full_data = {
            'robot_info_publisher': {
                'ros__parameters': expected_params
            }
        }
        self.assertEqual(loaded_data, expected_full_data, 'YAML does not match expected content.')
        return temp_dir_path

    def test_describe_method(self):
        """Test the describe() method."""
        content = {'param1': 'value1'}
        robot_info_sub = RobotInfoFile(content=content)
        self.assertEqual(robot_info_sub.describe(), "RobotInfoFile({'param1': 'value1'})")

    def test_perform_simple_content(self):
        """Test perform with simple string key-value pairs."""
        content = {'robot_name': 'test_robot', 'port': '1234'}
        robot_info_sub = RobotInfoFile(content=content)
        expected_params = {'robot_name': 'test_robot', 'port': '1234'}
        self._perform_and_check(robot_info_sub, expected_params)

    def test_perform_boolean_strings(self):
        """Test perform with 'True' and 'False' string values."""
        content = {'is_enabled': 'True', 'has_feature': 'False', 'is_active': 'True'}
        robot_info_sub = RobotInfoFile(content=content)
        expected_params = {'is_enabled': True, 'has_feature': False, 'is_active': True}
        self._perform_and_check(robot_info_sub, expected_params)

    def test_perform_actual_booleans(self):
        """Test perform with actual boolean values (should be stringified then re-evaluated)."""
        # The original class's perform method implies it expects string inputs for boolean.
        # If actual booleans are passed, they'd be str(True) -> "True" then converted.
        content = {'is_enabled': True, 'has_feature': False}
        robot_info_sub = RobotInfoFile(content=content)
        # str(True) is 'True', str(False) is 'False'
        expected_params = {'is_enabled': True, 'has_feature': False}
        self._perform_and_check(robot_info_sub, expected_params)

    def test_perform_integer_values(self):
        """Test perform with integer values (should be stringified)."""
        content = {'count': 10, 'max_value': -5}
        robot_info_sub = RobotInfoFile(content=content)
        # Integers will be converted to strings by str(arg_value) then used as is
        # if not 'True' or 'False'.
        self._perform_and_check(robot_info_sub, content)

    def test_perform_with_launch_configuration(self):
        """Test perform with LaunchConfiguration values."""
        # Set up launch arguments in the context
        self.context.launch_configurations['robot_model'] = 'fetch'
        self.context.launch_configurations['use_sim_time'] = 'True'

        content = {
            'model': LaunchConfiguration('robot_model'),
            'sim_time': LaunchConfiguration('use_sim_time')
        }
        robot_info_sub = RobotInfoFile(content=content)
        expected_params = {'model': 'fetch', 'sim_time': True}
        self._perform_and_check(robot_info_sub, expected_params)

    def test_perform_with_declare_launch_argument(self):
        """Test perform with DeclareLaunchArgument values."""
        declared_arg_name = DeclareLaunchArgument(
            name='robot_name_arg',
            default_value='default_robot'
        )
        declared_arg_bool = DeclareLaunchArgument(
            name='is_real_arg',
            default_value='False'
        )

        # Simulate launch system adding declared arguments to context if not already set
        self.context.extend_locals({
            declared_arg_name.name: declared_arg_name.default_value,
            declared_arg_bool.name: 'True'
        })
        # Or, more directly for testing LaunchConfiguration resolution:
        self.context.launch_configurations['robot_name_arg'] = 'my_lovely_robot'
        self.context.launch_configurations['is_real_arg'] = 'True'

        content = {
            'name_from_arg': declared_arg_name,
            'real_hw': declared_arg_bool
        }
        robot_info_sub = RobotInfoFile(content=content)

        # The RobotInfoFile will use LaunchConfiguration(arg_name).perform(context)
        expected_params = {
            'name_from_arg': 'my_lovely_robot',
            'real_hw': True
        }
        self._perform_and_check(robot_info_sub, expected_params)

    def test_perform_with_other_substitution(self):
        """Test perform with another type of Substitution (TextSubstitution)."""
        self.context.launch_configurations['prefix'] = 'robot'

        content_single_sub = {
             'description_val': TextSubstitution(text='MyBot')
        }
        robot_info_sub = RobotInfoFile(content=content_single_sub)
        expected_params = {'description_val': 'MyBot'}
        self._perform_and_check(robot_info_sub, expected_params)

    def test_file_structure_and_naming(self):
        """Explicitly test the generated file structure and name."""
        content = {'check_structure': 'ok'}
        robot_info_sub = RobotInfoFile(content=content)

        temp_dir_path = robot_info_sub.perform(self.context)
        self.temp_dirs_created.append(temp_dir_path)

        expected_file_path = os.path.join(temp_dir_path, '99_robot_info.yaml')
        self.assertTrue(os.path.isfile(expected_file_path),
                        f"File '99_robot_info.yaml' not found in '{temp_dir_path}'.")

        with open(expected_file_path, 'r') as f:
            data = yaml.safe_load(f)

        self.assertIn('robot_info_publisher', data)
        self.assertIn('ros__parameters', data['robot_info_publisher'])
        self.assertIn('check_structure', data['robot_info_publisher']['ros__parameters'])
        self.assertEqual(data['robot_info_publisher']['ros__parameters']['check_structure'], 'ok')

    def test_empty_content(self):
        """Test perform with empty content dictionary."""
        content = {}
        robot_info_sub = RobotInfoFile(content=content)
        expected_params = {}
        self._perform_and_check(robot_info_sub, expected_params)

    def test_multiple_perform_calls_on_same_instance_effect(self):
        """
        Test the effect of calling perform multiple times on the same instance.

        The provided RobotInfoFile re-initializes robot_info_data in perform,
        so it should be fresh.
        """
        content1 = {'param_run1': 'value1'}
        robot_info_sub = RobotInfoFile(content=content1)

        # First perform
        expected_params1 = {'param_run1': 'value1'}
        _ = self._perform_and_check(robot_info_sub, expected_params1)

        # If we were to change content_input and call perform again (not typical for Substitutions)
        # Or if the context changes. Let's simulate a context change.
        self.context.launch_configurations['dynamic_param'] = 'first_val'
        robot_info_sub_dynamic = RobotInfoFile(
            content={'dp': LaunchConfiguration('dynamic_param')})

        expected_params_dyn1 = {'dp': 'first_val'}
        temp_dir_dyn1 = self._perform_and_check(robot_info_sub_dynamic, expected_params_dyn1)

        self.context.launch_configurations['dynamic_param'] = 'second_val'
        expected_params_dyn2 = {'dp': 'second_val'}
        temp_dir_dyn2 = self._perform_and_check(robot_info_sub_dynamic, expected_params_dyn2)

        self.assertNotEqual(temp_dir_dyn1, temp_dir_dyn2,
                            'Each perform call should create a new temp directory.')


if __name__ == '__main__':
    unittest.main()
