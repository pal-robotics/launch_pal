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
import uuid

import rclpy
from rclpy.node import Node as RclpyNode

from launch.launch_context import LaunchContext
from launch_pal.conditions import IfNodeRunning, UnlessNodeRunning


class TestNodeRunning(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        # Initialize rclpy once for all tests in this class
        if not rclpy.ok():
            rclpy.init()
        cls.context = LaunchContext()

    @classmethod
    def tearDownClass(cls):
        # Shutdown rclpy once after all tests
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        # This method is called before each test
        self.test_node_name = "my_test_node_" + str(uuid.uuid4()).split('-')[0]
        self.test_node = None

        # Re-initialize if it was shut down by a predicate_func
        if not rclpy.ok():
            rclpy.init()

    def tearDown(self):
        # This method is called after each test
        if self.test_node and rclpy.ok():
            self.test_node.destroy_node()
            self.test_node = None
        # Brief spin to allow node destruction to propagate
        if rclpy.ok():
            rclpy.spin_once(
                RclpyNode("cleanup_spinner_" + str(uuid.uuid4()).split('-')[0]),
                timeout_sec=0.05
            )

    def test_node_is_running(self):
        """Test that the condition is True when the node is running."""
        self.test_node = RclpyNode(self.test_node_name)

        # Allow time for the node to be discoverable
        # Spinning the test_node itself isn't strictly necessary for discovery by others,
        # but good practice. The main thing is that the ROS graph is updated.
        end_time = self.test_node.get_clock().now() + rclpy.duration.Duration(seconds=0.5)
        while rclpy.ok() and self.test_node.get_clock().now() < end_time:
            rclpy.spin_once(self.test_node, timeout_sec=0.05)

        condition = IfNodeRunning(node_name=self.test_node_name)
        self.assertTrue(condition.evaluate(self.context),
                        f"Condition should be True when node '{self.test_node_name}' is running.")

        unless_condition = UnlessNodeRunning(node_name=self.test_node_name)
        self.assertFalse(unless_condition.evaluate(self.context),
                         "Unless condition should be False"
                         f"when node '{self.test_node_name}' is running.")

        self.test_node.destroy_node()
        self.test_node = None

    def test_node_is_not_running(self):
        """Test that the condition is False when the node is not running."""
        non_existent_node_name = "non_existent_node_" + str(uuid.uuid4()).split('-')[0]
        condition = IfNodeRunning(node_name=non_existent_node_name)
        self.assertFalse(condition.evaluate(self.context),
                         "Condition should be False for non-existent node:"
                         f"{non_existent_node_name}.")

        unless_condition = UnlessNodeRunning(node_name=non_existent_node_name)
        self.assertTrue(unless_condition.evaluate(self.context),
                        f"Unless condition should be True for non-existent node:"
                        f"{non_existent_node_name}.")

    def test_node_is_running_with_namespace(self):
        """Test that the condition is True when the node with a namespace is running."""
        # Note: The original IfNodeRunning class checks for node_name only, not fqn.
        # This test assumes we are checking the base name.
        # If FQN check is desired, IfNodeRunning would need modification.
        node_name_with_ns = "my_namespaced_node_" + str(uuid.uuid4()).split('-')[0]
        namespace = "/test_ns"
        self.test_node = RclpyNode(node_name_with_ns, namespace=namespace,
                                   start_parameter_services=False,
                                   enable_rosout=False)

        end_time = self.test_node.get_clock().now() + rclpy.duration.Duration(seconds=0.5)
        while rclpy.ok() and self.test_node.get_clock().now() < end_time:
            rclpy.spin_once(self.test_node, timeout_sec=0.05)

        condition = IfNodeRunning(node_name=node_name_with_ns)
        self.assertTrue(condition.evaluate(self.context),
                        f"Condition should be True for node '{node_name_with_ns}'"
                        " even with namespace.")

        unless_condition = UnlessNodeRunning(node_name=node_name_with_ns)
        self.assertFalse(unless_condition.evaluate(self.context),
                         f"Unless condition should be False for node '{node_name_with_ns}'"
                         " even with namespace.")

        self.test_node.destroy_node()
        self.test_node = None

    def test_if_node_running_init_shutdown_management(self):
        """Test that rclpy is initialized and shutdown correctly by the condition if needed."""
        if rclpy.ok():
            rclpy.shutdown()

        self.assertFalse(rclpy.ok(), "rclpy should be shutdown before this test part.")

        condition = IfNodeRunning(node_name="some_node_for_init_test")
        # The evaluate call will initialize rclpy
        condition.evaluate(self.context)

        # Check if rclpy was initialized by the condition and then shut down
        # After evaluate, if it was initialized by the condition, it should be shut down.
        self.assertFalse(rclpy.ok(),
                         "rclpy should be shutdown by IfNodeRunning if it initialized it.")

    def test_unless_node_running_init_shutdown_management(self):
        """Test that rclpy is initialized and shutdown correctly by the condition if needed."""
        if rclpy.ok():
            rclpy.shutdown()

        self.assertFalse(rclpy.ok(), "rclpy should be shutdown before this test part.")

        condition = UnlessNodeRunning(node_name="some_node_for_init_test")
        # The evaluate call will initialize rclpy
        condition.evaluate(self.context)

        # Check if rclpy was initialized by the condition and then shut down
        # After evaluate, if it was initialized by the condition, it should be shut down.
        self.assertFalse(rclpy.ok(),
                         "rclpy should be shutdown by UnlessNodeRunning if it initialized it.")


if __name__ == '__main__':
    unittest.main()
