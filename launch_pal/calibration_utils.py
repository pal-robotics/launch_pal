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

from launch_pal.param_utils import merge_param_files
import os
import yaml
import tempfile
from jinja2 import Environment, FileSystemLoader
from pathlib import Path

MASTER_CALIBRATION_FILE = "/etc/calibration/master_calibration.yaml"
ROS_PARAM_KEY = 'ros__parameters'


def apply_master_calibration(param_file: str) -> str:

    node_name_list = get_node_names_from_yaml(param_file)
    if len(node_name_list) != 1:
        raise ValueError("A node parameter file should contain parameters of exactly one node")
    node_name = node_name_list[0]

    yaml_files = [param_file]

    node_calibration_data = get_master_calibration_params(node_name)

    # Create tmp file with only relevant params
    tmp_file = tempfile.NamedTemporaryFile(suffix='.yaml')
    with open(tmp_file.name, 'w') as f:
        yaml.safe_dump(node_calibration_data, f)

    yaml_files.append(tmp_file.name)

    updated_param_file = merge_param_files(yaml_files)

    return updated_param_file


def apply_urdf_calibration(template_folder: Path, output_folder: Path) -> dict:

    calibration_xacro_args = {}

    # Get the calibration data for the robot_state_publisher node
    robot_state_publisher_node = "robot_state_publisher"
    master_calibration_data = get_master_calibration_params(
        robot_state_publisher_node)

    if not master_calibration_data:
        return calibration_xacro_args

    check_param_file_layout(master_calibration_data)

    node_calibration_data = master_calibration_data[robot_state_publisher_node][ROS_PARAM_KEY]

    # List template files
    template_files = {f.name.split('.')[0]: f for f in list(template_folder.glob("*.j2"))}
    if node_calibration_data:

        for key, value in node_calibration_data.items():
            if key not in template_files:
                continue

            # Parse template
            output_file = output_folder / template_files[key].stem
            input_file = template_files[key]
            parse_jinja_template(input_file, output_file, value)

            # Add xacro arg
            xacro_arg_name = f"{key}_dir"
            calibration_xacro_args[xacro_arg_name] = str(output_folder)

    return calibration_xacro_args


def get_master_calibration_params(node_name: str):

    node_calibration_data = {}
    if not os.path.exists(MASTER_CALIBRATION_FILE):
        return node_calibration_data

    # Ensure only the params of node are updated
    master_calibration_data = load_yaml(MASTER_CALIBRATION_FILE)
    master_calibration_data_nodes = list(master_calibration_data.keys())

    if node_name not in master_calibration_data_nodes:
        return node_calibration_data

    node_calibration_data[node_name] = master_calibration_data[node_name]
    return node_calibration_data


def get_node_names_from_yaml(yaml_file: str):

    data = load_yaml(yaml_file)

    check_param_file_layout(data)

    return list(data.keys())


def check_param_file_layout(data: dict):

    for key, value in data.items():
        if not isinstance(value, dict) or ROS_PARAM_KEY not in value or \
                not isinstance(value[ROS_PARAM_KEY], dict):
            raise ValueError(
                f"ROS 2 Param file for node '{key}' does not have the required layout")

    return


def parse_jinja_template(template_file: str, output_file: str, params: dict):

    template_env = Environment(loader=FileSystemLoader(template_file))

    with open(template_file, 'r') as f:
        template = template_env.from_string(f.read())
        generated_file = template.render(params)

    with open(output_file, 'w') as f:
        f.write(generated_file)

    return


def load_yaml(yaml_file: str):
    with open(yaml_file, 'r') as file:
        return yaml.safe_load(file)
