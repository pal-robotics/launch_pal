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


ROBOT_NAME = "tiago_pro"


@dataclass(frozen=True)
class TiagoProArgs:
    """This dataclass contains launch arguments for TIAGo Pro."""
    base_type: DLA = create_robot_arg('base_type', ROBOT_NAME)
    arm_type_right: DLA = create_robot_arg('arm_type_right', ROBOT_NAME)
    arm_type_left: DLA = create_robot_arg('arm_type_left', ROBOT_NAME)
    end_effector_right: DLA = create_robot_arg('end_effector_right', ROBOT_NAME)
    end_effector_left: DLA = create_robot_arg('end_effector_left', ROBOT_NAME)
    ft_sensor_right: DLA = create_robot_arg('ft_sensor_right', ROBOT_NAME)
    ft_sensor_left: DLA = create_robot_arg('ft_sensor_left', ROBOT_NAME)
    camera_model: DLA = create_robot_arg('camera_model', ROBOT_NAME)
    laser_model: DLA = create_robot_arg('laser_model', ROBOT_NAME)

