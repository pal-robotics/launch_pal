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

import unittest
from launch_pal import calibration_utils
from launch_pal.calibration_utils import ROS_PARAM_KEY
import tempfile
import yaml
from pathlib import Path


class TestCalibration(unittest.TestCase):

    def setUp(self):
        # Setup the mock default configuration file with ros parameters
        self.node_name = "node_name"
        self.params = {ROS_PARAM_KEY:
                       {"param_1": 1.0,
                        "param_2": 2.0,
                        "param_3": 3.0}
                       }
        self.default_param_file = self.create_param_yaml(self.node_name, self.params)

    def tearDown(self):
        # Clean up the temporary file
        self.default_param_file.close()

    def create_param_yaml(self, node_name, params):

        temp_file = tempfile.NamedTemporaryFile(suffix='.yaml')
        yaml_content = {
            node_name: params
        }
        with open(temp_file.name, 'w') as file:
            yaml.dump(yaml_content, file)
        return temp_file

    def test_calibration(self):

        # Create a mock master calibration file

        node_name = self.node_name
        master_calibration_params = {
            "param_2": 20.0,
        }

        master_calibration_path = self.create_param_yaml(node_name, master_calibration_params)

        # Overwrite the master_calibration_file for mocking purposes
        calibration_utils.MASTER_CALIBRATION_FILE = master_calibration_path.name

        # Apply the master calibration
        updated_param_file = calibration_utils.apply_master_calibration(
            self.default_param_file.name)

        updated_params = calibration_utils.load_yaml(
            updated_param_file)[self.node_name]
        # Check that the updated parameters are correct

        self.assertEqual(updated_params[ROS_PARAM_KEY]['param_1'],
                         self.params[ROS_PARAM_KEY]['param_1'])
        self.assertEqual(updated_params[ROS_PARAM_KEY]['param_2'],
                         master_calibration_params['param_2'])
        self.assertEqual(updated_params[ROS_PARAM_KEY]['param_3'],
                         self.params[ROS_PARAM_KEY]['param_3'])

    def test_no_calibration_parameters(self):

        node_name = "different_node_name"
        params = {
            "param_test": 20.0,
        }

        master_calibration_path = self.create_param_yaml(node_name, params)

        # Overwrite the master_calibration_file for mocking purposes
        calibration_utils.MASTER_CALIBRATION_FILE = master_calibration_path.name

        # Apply the master calibration
        updated_param_file = calibration_utils.apply_master_calibration(
            self.default_param_file.name)

        updated_params = calibration_utils.load_yaml(
            updated_param_file)[self.node_name]
        # Check that the updated parameters are correct

        self.assertEqual(updated_params[ROS_PARAM_KEY]['param_1'],
                         self.params[ROS_PARAM_KEY]['param_1'])
        self.assertEqual(updated_params[ROS_PARAM_KEY]['param_2'],
                         self.params[ROS_PARAM_KEY]['param_2'])
        self.assertEqual(updated_params[ROS_PARAM_KEY]['param_3'],
                         self.params[ROS_PARAM_KEY]['param_3'])

        pass

    def test_no_calibration_file(self):
        # Use default master_calibration_file that does not exist
        updated_param_file = calibration_utils.apply_master_calibration(
            self.default_param_file.name)

        updated_params = calibration_utils.load_yaml(
            updated_param_file)[self.node_name]
        # Check that the updated parameters are correct

        self.assertEqual(updated_params[ROS_PARAM_KEY]['param_1'],
                         self.params[ROS_PARAM_KEY]['param_1'])
        self.assertEqual(updated_params[ROS_PARAM_KEY]['param_2'],
                         self.params[ROS_PARAM_KEY]['param_2'])
        self.assertEqual(updated_params[ROS_PARAM_KEY]['param_3'],
                         self.params[ROS_PARAM_KEY]['param_3'])

        pass

    def test_urdf_calibration(self):

        # Create a mock master calibration file
        calibration_method = "mock_calibration"
        urdf_file = f"{calibration_method}.urdf.xacro"
        master_calibration_params = {
            calibration_method: {
                "param_1": 1.0,
                "param_2": 5.0
            },
        }
        node_name = "robot_state_publisher"
        master_calibration_path = self.create_param_yaml(node_name, master_calibration_params)

        # Overwrite the master_calibration_file for mocking purposes
        calibration_utils.MASTER_CALIBRATION_FILE = master_calibration_path.name

        input_folder = Path(__file__).resolve().parent / 'mock_calibration_template'
        output_folder = tempfile.TemporaryDirectory()
        output_folder_path = Path(output_folder.name)
        calibration_xacro_args = calibration_utils.apply_urdf_calibration(
            input_folder, output_folder_path)

        calibration_dir = calibration_xacro_args[f"{calibration_method}_dir"]

        # Check if output dir is correct
        self.assertEqual(calibration_dir, output_folder.name)

        with open(output_folder_path / urdf_file, 'r') as file:
            updated_urdf = file.read()

        with open(input_folder / urdf_file, 'r') as file:
            expected_result = file.read()

        # Check if result is as expected
        self.maxDiff = None
        self.assertEqual(updated_urdf, expected_result)
        return


if __name__ == '__main__':
    unittest.main()
