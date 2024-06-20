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

os.environ['AMENT_PREFIX_PATH'] = os.path.join(
    os.getcwd(), 'test', 'mock_rosroot_pal_parameters')


class TestPalGetConfiguration(unittest.TestCase):
    def test_get_configuration(self):
        config = get_pal_configuration(pkg='test_node', node='test_node')

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


if __name__ == '__main__':
    unittest.main()
