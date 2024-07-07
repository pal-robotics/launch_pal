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


import unittest
import os
from launch_pal.include_utils import include_launch_py_description
from launch_pal.include_utils import include_scoped_launch_py_description
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare


class TestIncludeLaunch(unittest.TestCase):

    def test_include_python_file(self):
        context = LaunchContext()

        pkg_name = "launch_pal"
        file_path = ['launch', 'test.launch.py']
        launch_arguments = {'arg_1': 'value_1'}

        pkg_folder = FindPackageShare(pkg_name)
        folder_path = pkg_folder.perform(context)

        launch_file = include_launch_py_description(
            pkg_name=pkg_name, paths=file_path, launch_arguments=launch_arguments.items())

        lds = launch_file.launch_description_source
        path_substitution = lds._LaunchDescriptionSource__location[0]

        # Assert path is correct
        self.assertEqual(path_substitution.perform(
            context), os.path.join(folder_path, *file_path))

        # Assert launch arguments are passed on correct
        self.assertEqual(tuple(launch_arguments.items()),
                         launch_file.launch_arguments)

        return

    def test_include_python_file_with_args(self):
        pkg_name = "launch_pal"
        file_path = ['launch', 'test.launch.py']
        launch_arguments = {'arg_1': 'value_1'}

        launch_file = include_launch_py_description(
            pkg_name=pkg_name, paths=file_path, launch_arguments=launch_arguments.items())

        # Assert launch arguments are passed on correct
        self.assertEqual(tuple(launch_arguments.items()),
                         launch_file.launch_arguments)

        return

    def test_include_scoped_python_file(self):

        pkg_name = "launch_pal"
        file_path = ['launch', 'test.launch.py']

        # Create scoped launch file
        scoped_launch_file = include_scoped_launch_py_description(
            pkg_name=pkg_name,
            paths=file_path)

        # Assert scoping is true, but forwarding is false
        self.assertTrue(scoped_launch_file._GroupAction__scoped)
        self.assertFalse(scoped_launch_file._GroupAction__forwarding)

        scoped_actions = scoped_launch_file._GroupAction__actions
        # final action should be the launch file
        nested_launch_file = scoped_actions[-1]

        # Assert that final action is the included launch file
        self.assertTrue(isinstance(
            nested_launch_file, IncludeLaunchDescription))

        return

    def test_include_scoped_python_file_with_args(self):

        pkg_name = "launch_pal"
        file_path = ['launch', 'test.launch.py']

        arg_4 = DeclareLaunchArgument(name='arg_4', default_value="value_4")
        arg_e = DeclareLaunchArgument(name='arg_e', default_value="value_5")

        # Create launch arguments two cases of remapping
        launch_arguments = {'arg_1': 'value_1',
                            'arg_2': LaunchConfiguration('arg_2'),
                            'arg_3': LaunchConfiguration('arg_c'),
                            'arg_4': arg_4,
                            'arg_5': arg_e}

        # Create environment variable
        env_var = SetEnvironmentVariable("ENV_TEST", 'value')

        # Create scoped launch file
        scoped_launch_file = include_scoped_launch_py_description(
            pkg_name=pkg_name,
            paths=file_path,
            launch_arguments=launch_arguments,
            env_vars=[env_var])

        # Get the scoped actions
        scoped_actions = scoped_launch_file._GroupAction__actions
        nested_launch_file = scoped_actions[-1]

        # Assert environment variable is in the scoped action
        self.assertEqual(env_var, scoped_actions[0])
        # Assert that remapped declared argument is in the scoped action
        self.assertEqual(arg_e, scoped_actions[1])

        # Assert that final action is the included launch file
        self.assertTrue(isinstance(
            nested_launch_file, IncludeLaunchDescription))

        # get launch arguments
        nested_launch_args = dict(nested_launch_file.launch_arguments)
        grouped_launch_args = scoped_launch_file._GroupAction__launch_configurations

        self.assertDictEqual(nested_launch_args, grouped_launch_args)

        # Check types of arguments
        self.assertTrue(isinstance(
            nested_launch_args['arg_1'], str))
        self.assertTrue(isinstance(
            nested_launch_args['arg_2'], LaunchConfiguration))
        self.assertTrue(isinstance(
            nested_launch_args['arg_3'], LaunchConfiguration))
        self.assertTrue(isinstance(
            nested_launch_args['arg_4'], LaunchConfiguration))
        self.assertTrue(isinstance(
            nested_launch_args['arg_5'], LaunchConfiguration))

        # Check values of arguments
        self.assertEqual(nested_launch_args['arg_1'], 'value_1')
        self.assertEqual(
            nested_launch_args['arg_2'].variable_name[0].text, 'arg_2')
        self.assertEqual(
            nested_launch_args['arg_3'].variable_name[0].text, 'arg_c')
        self.assertEqual(
            nested_launch_args['arg_4'].variable_name[0].text, 'arg_4')
        self.assertEqual(
            nested_launch_args['arg_5'].variable_name[0].text, 'arg_e')

        return
