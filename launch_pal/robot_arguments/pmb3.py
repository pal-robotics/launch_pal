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


from dataclasses import dataclass
from launch.actions import DeclareLaunchArgument as DLA
from .robot import create_robot_arg


ROBOT_NAME = "pmb3"


@dataclass(frozen=True)
class PMB3Args:
    """This dataclass contains launch arguments for PMB3."""

    wheel_model: DLA = create_robot_arg('wheel_model', ROBOT_NAME)
    laser_model: DLA = create_robot_arg('laser_model', ROBOT_NAME)
    camera_model: DLA = create_robot_arg('camera_model', ROBOT_NAME)
