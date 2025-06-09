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

from typing import Text

import uuid

from launch.condition import Condition
from launch.launch_context import LaunchContext

import rclpy
from rclpy.node import Node as RclpyNode
import rclpy.utilities


class UnlessNodeRunning(Condition):
    """
    Condition that evaluates to true if the node is running.

    This condition is useful to check if a node is running before executing
    an action. It uses the rclpy library to check the node names in the
    current context.
    """

    def __init__(self, node_name: Text) -> None:
        """
        Initialize the IfNodeRunning condition.

        :param node_name: The name of the node to check if it is running.
        """
        self.__node_name = node_name
        super().__init__(predicate=self._predicate_func)

    def _predicate_func(self, context: LaunchContext) -> bool:
        """
        Create a temporary node to check if the specified node is running.

        This method initializes rclpy if it is not already initialized and creates
        a temporary node to check the list of available nodes.
        Finally, the temporary node is destroyed, and rclpy is shut down if it was initialized.
        :param context: The launch context (not used in this method).
        :return: True if the specified node is NOT running, False otherwise.
        """
        if not rclpy.utilities.ok():
            rclpy.init()
        self.__node_checker = RclpyNode('node_checker_' + str(uuid.uuid4()).split('-')[0])
        rclpy.spin_once(self.__node_checker, timeout_sec=1.0)
        available_nodes = self.__node_checker.get_node_names()
        self.__node_checker.destroy_node()
        if rclpy.utilities.ok():
            rclpy.shutdown()
        return self.__node_name not in available_nodes

    def describe(self) -> Text:
        """Return a description of this Condition."""
        return self.__repr__()
