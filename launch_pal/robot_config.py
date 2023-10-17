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


import yaml
from dataclasses import dataclass
from typing import List, Union, Dict
from launch_ros.substitutions import FindPackageShare


@dataclass
class RobotSetting:
    name: str
    description: str
    values: List[Union[str, bool]]
    default: Union[str, bool]


class RobotConfiguration:
    def __init__(self, robot_name: str):
        self.robot_name: str = robot_name
        self.configuration: Dict[str, RobotSetting] = self.load_configuration(
            robot_name)

    def load_configuration(self, robot_name: str) -> Dict[str, RobotSetting]:

        pkg_dir = FindPackageShare("launch_pal").find("launch_pal")

        config_file = f"{pkg_dir}/config/{robot_name}_configuration.yaml"

        configurations_raw = yaml.load(open(config_file), Loader=yaml.FullLoader)[
            'robot_configuration']

        configuration = {}
        for setting_name in configurations_raw.keys():
            description = configurations_raw[setting_name]["description"]
            values = configurations_raw[setting_name]["values"]
            default_value = configurations_raw[setting_name]["default"]
            setting = RobotSetting(
                setting_name, description, values, default_value)
            configuration[setting_name] = setting

        return configuration

    def has_setting(self, setting_name: str) -> bool:

        setting_exists = setting_name in self.configuration.keys()
        return setting_exists

    def get_setting(self, setting_name: str) -> RobotSetting:

        if not self.has_setting(setting_name):
            raise ValueError(f'The robot {self.robot_name} \
                             does not have the setting {setting_name}')

        return self.configuration[setting_name]
