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
import yaml
import os
from typing import Dict
from launch_pal.param_utils import parse_parametric_yaml


class TestParseParametricYaml(unittest.TestCase):
    def test_param_substitution(self):
        parametric_yaml = os.path.join(os.getcwd(), "test", "parametric.yaml")
        parametric_variables = {"name": "John", "age": 42, "married": True}
        rewritten_yaml = parse_parametric_yaml(
            [parametric_yaml], parametric_variables
        )
        self.assertTrue(rewritten_yaml)

        with open(rewritten_yaml, "r") as params_yaml:
            params: Dict = yaml.safe_load(params_yaml)
            self.assertEqual(len(params), 4)
            self.assertTrue("name" in params)
            self.assertTrue("surname" in params)
            self.assertTrue("age" in params)
            self.assertTrue("married" in params)
            self.assertEqual(params["name"], parametric_variables["name"])
            self.assertEqual(params["surname"], "Doe")
            self.assertEqual(params["age"], parametric_variables["age"])
            self.assertEqual(params["married"], parametric_variables["married"])

    def test_params_merge(self):
        parametric_yaml = os.path.join(os.getcwd(), "test", "parametric.yaml")
        other_parametric_yaml = os.path.join(
            os.getcwd(), "test", "other_parametric.yaml"
        )
        parametric_variables = {
            "name": "John",
            "surname": "Black",
            "age": 42,
            "married": True,
            "weight": 70.3,
        }
        rewritten_yaml = parse_parametric_yaml(
            [parametric_yaml, other_parametric_yaml], parametric_variables
        )
        self.assertTrue(rewritten_yaml)

        with open(rewritten_yaml, "r") as params_yaml:
            params: Dict = yaml.safe_load(params_yaml)
            self.assertEqual(len(params), 6)
            self.assertTrue("name" in params)
            self.assertTrue("surname" in params)
            self.assertTrue("age" in params)
            self.assertTrue("married" in params)
            self.assertTrue("birthday" in params)
            self.assertTrue("weight" in params)
            self.assertEqual(params["name"], parametric_variables["name"])
            self.assertEqual(params["surname"], parametric_variables["surname"])
            self.assertEqual(params["age"], parametric_variables["age"])
            self.assertEqual(params["married"], parametric_variables["married"])
            self.assertEqual(params["birthday"], "01/01/1970")
            self.assertEqual(params["weight"], parametric_variables["weight"])
