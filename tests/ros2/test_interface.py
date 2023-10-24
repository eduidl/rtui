from __future__ import annotations

import typing as t
import unittest
import warnings

from rtui_app.ros.interface.ros2 import Ros2

from .node.dummy_node1 import DummyNode1
from .node.dummy_node2 import DummyNode2


def ignore_warnings(test_func):
    def do_test(self, *args, **kwargs):
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", ResourceWarning)
            test_func(self, *args, **kwargs)

    return do_test


class TestRos2Interface(unittest.TestCase):
    NODE1: t.ClassVar[DummyNode1 | None] = None
    NODE2: t.ClassVar[DummyNode2 | None] = None

    @classmethod
    def setUpClass(cls) -> None:
        cls.ROS = Ros2()
        cls.NODE1 = DummyNode1()
        cls.NODE2 = DummyNode2()

        rate = cls.ROS.node.create_rate(3)
        rate.sleep()

    @classmethod
    def tearDownClass(cls) -> None:
        cls.ROS.terminate()

    def test_get_node_publisher(self):
        publishers = self.ROS.get_node_publishers("/dummy_node1")
        self.assertIn(("/topic", "std_msgs/msg/String"), publishers)
        self.assertIn(("/pub", "std_msgs/msg/Int32"), publishers)

    def test_get_node_subscribers(self):
        subscribers = self.ROS.get_node_subscribers("/dummy_node1")
        self.assertIn(("/sub", "sensor_msgs/msg/Image"), subscribers)

    def test_get_node_service_servers(self):
        servers = self.ROS.get_node_service_servers("/dummy_node1")
        self.assertIn(("/service", "std_srvs/srv/SetBool"), servers)

    def test_get_node_service_clients(self):
        self.assertEqual(
            self.ROS.get_node_service_clients("/dummy_node1"),
            [("/client", "std_srvs/srv/Empty")],
        )

    def test_get_node_action_servers(self):
        self.assertEqual(
            self.ROS.get_node_action_servers("/dummy_node1"),
            [("/action", "tf2_msgs/action/LookupTransform")],
        )

    def test_get_node_action_clients(self):
        clients = self.ROS.get_node_action_clients("/dummy_node1")
        self.assertIn(("/action_client", "tf2_msgs/action/LookupTransform"), clients)

    def test_get_topic_types(self):
        types = self.ROS.get_topic_types("/topic")
        self.assertEqual(types, ["std_msgs/msg/String"])

    def test_get_topic_publishers(self):
        self.assertEqual(
            self.ROS.get_topic_publishers("/topic"),
            [("/dummy_node1", "std_msgs/msg/String")],
        )

    def test_get_topic_subscribers(self):
        self.assertEqual(
            self.ROS.get_topic_subscribers("/topic"),
            [("/dummy_node2", "std_msgs/msg/String")],
        )

    def test_get_service_types(self):
        self.assertEqual(
            self.ROS.get_service_types("/service"), ["std_srvs/srv/SetBool"]
        )

    def test_get_service_servers(self):
        # Not supported
        self.assertIsNone(self.ROS.get_service_servers("/service"))

    def test_get_action_types(self):
        self.assertEqual(
            self.ROS.get_action_types("/action"), ["tf2_msgs/action/LookupTransform"]
        )

    def test_get_action_servers(self):
        self.assertEqual(
            self.ROS.get_action_servers("/action"),
            [("/dummy_node1", "tf2_msgs/action/LookupTransform")],
        )

    def test_get_action_clients(self):
        self.assertEqual(
            self.ROS.get_action_clients("/action"),
            [("/dummy_node2", "tf2_msgs/action/LookupTransform")],
        )

    def test_list_nodes(self):
        nodes = self.ROS.list_nodes()
        self.assertIn("/dummy_node1", nodes)
        self.assertIn("/dummy_node2", nodes)
        self.assertNotIn("/_rtui", nodes)

    def test_list_topics(self):
        topics = self.ROS.list_topics()
        self.assertIn("/topic", topics)
        self.assertIn("/pub", topics)
        self.assertIn("/sub", topics)

    def test_list_services(self):
        services = self.ROS.list_services()
        self.assertIn("/service", services)
        self.assertIn("/client", services)

    def test_list_actions(self):
        actions = self.ROS.list_actions()
        self.assertIn("/action", actions)
        self.assertIn("/action_client", actions)


if __name__ == "__main__":
    unittest.main()
