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
from launch_pal.robot_config import RobotSetting, RobotConfiguration
from typing import List


class TestRobotConfiguration(unittest.TestCase):
    def test_load_configuration(self):
        robot_name = "tiago"
        RobotConfiguration(robot_name)

        faulty_robot_name = "ogait"

        with self.assertRaises(FileNotFoundError):
            RobotConfiguration(faulty_robot_name)

    def test_find_setting(self):
        robot_name = "tiago"
        robot_config = RobotConfiguration(robot_name)

        setting_name = "wheel_model"
        robot_setting = robot_config.get_setting(setting_name)
        self.assertIsInstance(robot_setting, RobotSetting)
        self.assertEqual(robot_setting.name, setting_name)
        self.assertIsInstance(robot_setting.default, str)
        self.assertIsInstance(robot_setting.description, str)
        self.assertIsInstance(robot_setting.values, List)

        invalid_setting_name = "can_fly"
        with self.assertRaises(ValueError):
            robot_setting = robot_config.get_setting(invalid_setting_name)
