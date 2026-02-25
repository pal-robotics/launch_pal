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

import os
from typing import Dict
import unittest
from unittest import mock

from launch_pal.param_utils import merge_param_files, parse_parametric_yaml
import yaml


class TestParseParametricYaml(unittest.TestCase):
    def test_param_substitution(self):
        parametric_yaml = os.path.join(os.getcwd(), 'test', 'parametric.yaml')
        parametric_variables = {'name': 'John', 'age': 42, 'married': True}
        rewritten_yaml = parse_parametric_yaml(
            [parametric_yaml], parametric_variables
        )
        self.assertTrue(rewritten_yaml)

        with open(rewritten_yaml, 'r') as params_yaml:
            params: Dict = yaml.safe_load(params_yaml)
            self.assertEqual(len(params), 4)
            self.assertTrue('name' in params)
            self.assertTrue('surname' in params)
            self.assertTrue('age' in params)
            self.assertTrue('married' in params)
            self.assertEqual(params['name'], parametric_variables['name'])
            self.assertEqual(params['surname'], 'Doe')
            self.assertEqual(params['age'], parametric_variables['age'])
            self.assertEqual(params['married'], parametric_variables['married'])

    def test_params_merge(self):
        parametric_yaml = os.path.join(os.getcwd(), 'test', 'parametric.yaml')
        other_parametric_yaml = os.path.join(
            os.getcwd(), 'test', 'other_parametric.yaml'
        )
        parametric_variables = {
            'name': 'John',
            'surname': 'Black',
            'age': 42,
            'married': False,
            'weight': 70.3,
        }
        rewritten_yaml = parse_parametric_yaml(
            [parametric_yaml, other_parametric_yaml], parametric_variables
        )
        self.assertTrue(rewritten_yaml)

        with open(rewritten_yaml, 'r') as params_yaml:
            params: Dict = yaml.safe_load(params_yaml)
            self.assertEqual(len(params), 6)
            self.assertTrue('name' in params)
            self.assertTrue('surname' in params)
            self.assertTrue('age' in params)
            self.assertTrue('married' in params)
            self.assertTrue('birthday' in params)
            self.assertTrue('weight' in params)
            self.assertEqual(params['name'], parametric_variables['name'])
            self.assertEqual(params['surname'], parametric_variables['surname'])
            self.assertEqual(params['age'], parametric_variables['age'])
            self.assertEqual(params['married'], parametric_variables['married'])
            self.assertEqual(params['birthday'], '01/01/1970')
            self.assertEqual(params['weight'], parametric_variables['weight'])

    @mock.patch('builtins.open')
    @mock.patch('yaml.safe_load')
    def test_merge_param_files_invalid_yaml_raises_error(self, mock_safe_load, mock_open):
        yaml_file_path = 'file.yaml'
        mock_safe_load.return_value = [1, 2, 3]

        # The function should raise a ValueError
        with self.assertRaises(ValueError) as context:
            merge_param_files([yaml_file_path])

            mock_open.assert_called_once_with(yaml_file_path, 'r')
            self.assertIn('not a dictionary', str(context.exception))
            self.assertIn(yaml_file_path, str(context.exception))

    @mock.patch('builtins.open')
    @mock.patch('yaml.safe_load')
    @mock.patch('yaml.dump')
    def test_merge_param_files_valid_yaml(self, mock_dump, mock_safe_load, mock_open):
        yaml_file_path = 'file.yaml'
        expected_data = {'key': 'value', 'number': 42}
        mock_safe_load.return_value = expected_data

        # The function should return the path to the rewritten yaml file
        result = merge_param_files([yaml_file_path])
        self.assertIsInstance(result, str)

        mock_open.assert_called_once_with(yaml_file_path, 'r')
        mock_dump.assert_called_once()
        args = mock_dump.call_args[0]
        self.assertEqual(args[0], expected_data)
