# Copyright (c) 2022 PAL Robotics S.L.
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

from launch.actions import DeclareLaunchArgument


def get_robot_name():
    declare_robot_name = DeclareLaunchArgument(
        'robot_name',
        default_value='pmb2',
        description='Name of the robot. ',
        choices=['pmb2', 'tiago'])

    return declare_robot_name


def get_wheel_model(robot):
    if (robot == 'pmb2') or (robot == 'tiago'):
        declare_wheel_model = DeclareLaunchArgument(
            'wheel_model',
            default_value='moog',
            description='Wheel model, ',
            choices=['nadia', 'moog'])

    else:
        raise ValueError('The robot ' + robot + ' has not the argument wheel_model')

    return declare_wheel_model


def get_laser_model(robot):
    if (robot == 'pmb2') or (robot == 'tiago'):
        declare_laser_model = DeclareLaunchArgument(
            'laser_model',
            default_value='sick-571',
            description='Base laser model. ',
            choices=['no-laser', 'sick-571', 'sick-561', 'sick-551', 'hokuyo'])

    else:
        raise ValueError('The robot ' + robot + ' has not the argument laser_model')

    return declare_laser_model


def get_courier_rgbd_sensors(robot):
    if (robot == 'pmb2'):
        declare_courier_rgbd_sensors = DeclareLaunchArgument(
            'courier_rgbd_sensors',
            default_value='False',
            description='Whether the base has RGBD sensors or not. ',
            choices=['True', 'False'])

    else:
        raise ValueError('The robot ' + robot + ' has not the argument courier_rgbd_sensors')

    return declare_courier_rgbd_sensors


def get_arm(robot):
    if (robot == 'tiago'):
        declare_arm = DeclareLaunchArgument(
            'arm',
            default_value='right-arm',
            description='Which type of arm TIAGo has. ',
            choices=['no-arm', 'left-arm', 'right-arm'])

    else:
        raise ValueError('The robot ' + robot + ' has not the argument arm')

    return declare_arm


def get_wrist_model(robot):
    if (robot == 'tiago'):
        declare_wrist_model = DeclareLaunchArgument(
            'wrist_model',
            default_value='wrist-2010',
            description='Wrist model. ',
            choices=['wrist-2010', 'wrist-2017'])

    else:
        raise ValueError('The robot ' + robot + ' has not the argument wrist_model')

    return declare_wrist_model


def get_end_effector(robot):
    if (robot == 'tiago'):
        declare_end_effector = DeclareLaunchArgument(
            'end_effector',
            default_value='pal-gripper',
            description='End effector model.',
            choices=['pal-gripper', 'pal-hey5', 'schunk-wsg',
                     'custom', 'no-end-effector'])

    else:
        raise ValueError('The robot ' + robot + ' has not the argument end_effector')

    return declare_end_effector


def get_ft_sensor(robot):
    if (robot == 'tiago'):
        declare_ft_sensor = DeclareLaunchArgument(
            'ft_sensor',
            default_value='schunk-ft',
            description='FT sensor model. ',
            choices=['schunk-ft', 'no-ft-sensor'])

    else:
        raise ValueError('The robot ' + robot + ' has not the argument ft_sensor')

    return declare_ft_sensor


def get_camera_model(robot):
    if (robot == 'tiago'):
        declare_camera_model = DeclareLaunchArgument(
            'camera_model',
            default_value='orbbec-astra',
            description='Head camera model. ',
            choices=['no-camera', 'orbbec-astra', 'orbbec-astra-pro', 'asus-xtion'])

    else:
        raise ValueError('The robot ' + robot + ' has not the argument camera_model')

    return declare_camera_model
