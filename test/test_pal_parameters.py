# Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
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
import unittest

from launch import LaunchDescription
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration
from launch_pal.pal_parameters import get_pal_configuration


class TestPalGetConfiguration(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        TestPalGetConfiguration.prev_ament_prefix_path = os.environ.get(
            'AMENT_PREFIX_PATH', None)

        os.environ['AMENT_PREFIX_PATH'] = os.path.join(
            os.getcwd(), 'test', 'mock_rosroot_pal_parameters')

    @classmethod
    def tearDownClass(cls):
        if TestPalGetConfiguration.prev_ament_prefix_path is not None:
            os.environ['AMENT_PREFIX_PATH'] = TestPalGetConfiguration.prev_ament_prefix_path
        else:
            del os.environ['AMENT_PREFIX_PATH']

    def test_get_configuration(self):

        config = get_pal_configuration(pkg='test_node', node='test_node', cmdline_args=False)

        self.assertEqual(len(config['parameters']), 1)
        self.assertDictEqual(
            config['parameters'][0],
            {
                'param_default': 'default',
                'param_template': 'template',
                'param_base': 'base',
                'param_robot': 'robot',
                'param_user': 'robot',
            }
        )

        self.assertCountEqual(
            config['remappings'],
            [
                ('remap_default', 'default'),
                ('remap_template', 'template'),
                ('remap_base', 'base'),
                ('remap_robot', 'robot'),
                ('remap_user', 'robot'),
            ]
        )

        self.assertCountEqual(
            config['arguments'],
            ['--arg_robot_1', '--arg_robot_2']
        )

    def test_get_configuration_user_overrides(self):

        os.environ['PAL_USER_PARAMETERS_PATH'] = os.path.join(
            os.getcwd(), 'test', 'mock_rosroot_pal_parameters', 'home', '.pal', 'config')

        config = get_pal_configuration(pkg='test_node', node='test_node', cmdline_args=False)

        self.assertEqual(len(config['parameters']), 1)
        self.assertDictEqual(
            config['parameters'][0],
            {
                'param_default': 'default',
                'param_template': 'template',
                'param_base': 'base',
                'param_robot': 'robot',
                'param_user': 'user',
            }
        )

        self.assertCountEqual(
            config['remappings'],
            [
                ('remap_default', 'default'),
                ('remap_template', 'template'),
                ('remap_base', 'base'),
                ('remap_robot', 'robot'),
                ('remap_user', 'user_subdir'),
            ]
        )

        self.assertCountEqual(
            config['arguments'],
            ['--arg_user']
        )

    def test_get_configuration_cmdline_overrides_single_param(self):

        ld = LaunchDescription()
        lc = LaunchContext()

        config = get_pal_configuration(pkg='test_node',
                                       node='test_node',
                                       cmdline_args=['param_default'],
                                       ld=ld)

        for k, v in config['parameters'][0].items():
            if k == 'param_default':
                self.assertTrue(isinstance(v, LaunchConfiguration))
                self.assertEquals(v.perform(lc), 'default')
            else:
                self.assertFalse(isinstance(v, LaunchConfiguration))

    def test_get_configuration_cmdline_overrides_all_params(self):

        ld = LaunchDescription()

        config = get_pal_configuration(pkg='test_node', node='test_node', ld=ld)

        for v in config['parameters'][0].values():
            self.assertTrue(isinstance(v, LaunchConfiguration))

    def test_get_configuration_cmdline_overrides_no_params(self):

        ld = LaunchDescription()

        config = get_pal_configuration(pkg='test_node',
                                       node='test_node',
                                       ld=ld,
                                       cmdline_args=False)

        for v in config['parameters'][0].values():
            self.assertFalse(isinstance(v, LaunchConfiguration))


if __name__ == '__main__':
    unittest.main()
