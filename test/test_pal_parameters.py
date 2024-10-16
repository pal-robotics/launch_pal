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

import unittest
import os
from launch_pal.pal_parameters import get_pal_configuration
from launch.substitutions import LaunchConfiguration
from launch.launch_context import LaunchContext
from launch import LaunchDescription


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

        self.assertCountEqual(
            config['parameters'],
            [
                {'param1': 'base value'},
                {'param2': 'robot-cfg value'},
                {'param3': 'test-cfg value'},
            ]
        )

        self.assertCountEqual(
            config['remappings'],
            [
                ('remap1', '/robot-remap'),
                ('remap2', '/test-remap'),
                ('remap3', '/robot-remap'),
            ]
        )

    def test_get_configuration_user_overrides(self):

        os.environ['PAL_USER_PARAMETERS_PATH'] = os.path.join(
            os.getcwd(), 'test', 'mock_rosroot_pal_parameters', 'home', 'pal')

        config = get_pal_configuration(pkg='test_node', node='test_node', cmdline_args=False)

        self.assertCountEqual(
            config['parameters'],
            [
                {'param1': 'test_node user param1 value high precedence'},
                {'param2': 'robot-cfg value'},
                {'param3': 'test-cfg value'},
            ]
        )

        self.assertCountEqual(
            config['remappings'],
            [
                ('remap1', '/user-robot-remap'),
                ('remap2', '/test-remap'),
                ('remap3', '/robot-remap'),
            ]
        )

    def test_get_configuration_cmdline_overrides_single_param(self):

        ld = LaunchDescription()

        config = get_pal_configuration(pkg='test_node',
                                       node='test_node',
                                       cmdline_args=['param1'],
                                       ld=ld)

        self.assertTrue(isinstance(config['parameters'][0]['param1'], LaunchConfiguration))

        lc = LaunchContext()

        self.assertEquals(config['parameters'][0]['param1'].perform(lc), 'base value')

        self.assertFalse(isinstance(config['parameters'][1]['param2'], LaunchConfiguration))
        self.assertFalse(isinstance(config['parameters'][2]['param3'], LaunchConfiguration))

    def test_get_configuration_cmdline_overrides_all_params(self):

        ld = LaunchDescription()

        config = get_pal_configuration(pkg='test_node', node='test_node', ld=ld)

        self.assertTrue(isinstance(config['parameters'][0]['param1'], LaunchConfiguration))
        self.assertTrue(isinstance(config['parameters'][1]['param2'], LaunchConfiguration))
        self.assertTrue(isinstance(config['parameters'][2]['param3'], LaunchConfiguration))

    def test_get_configuration_cmdline_overrides_no_params(self):

        ld = LaunchDescription()

        config = get_pal_configuration(pkg='test_node',
                                       node='test_node',
                                       ld=ld,
                                       cmdline_args=False)

        self.assertFalse(isinstance(config['parameters'][0]['param1'], LaunchConfiguration))
        self.assertFalse(isinstance(config['parameters'][1]['param2'], LaunchConfiguration))
        self.assertFalse(isinstance(config['parameters'][2]['param3'], LaunchConfiguration))


if __name__ == '__main__':
    unittest.main()
