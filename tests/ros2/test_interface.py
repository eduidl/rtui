from __future__ import annotations

import typing as t
import unittest
import warnings

from rtui.ros import init_ros, is_ros2

from .node.dummy_node1 import DummyNode1
from .node.dummy_node2 import DummyNode2

TestCase: t.Type = unittest.TestCase if is_ros2() else object


def ignore_warnings(test_func):
    def do_test(self, *args, **kwargs):
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", ResourceWarning)
            test_func(self, *args, **kwargs)

    return do_test


class TestRos2Interface(TestCase):
    NODE1: t.ClassVar[DummyNode1 | None] = None
    NODE2: t.ClassVar[DummyNode2 | None] = None

    @classmethod
    def setUpClass(cls) -> None:
        cls.ROS = init_ros()
        cls.NODE1 = DummyNode1()
        cls.NODE2 = DummyNode2()

        rate = cls.ROS.node.create_rate(3)
        rate.sleep()

    @classmethod
    def tearDownClass(cls) -> None:
        cls.ROS.terminate()

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

    def test_get_node_info(self):
        node_name = "/dummy_node1"
        info = self.ROS.get_node_info(node_name)
        self.assertEqual(info.name, node_name)
        self.assertIn(("/topic", "std_msgs/msg/String"), info.publishers)
        self.assertIn(("/pub", "std_msgs/msg/Int32"), info.publishers)
        self.assertIn(("/sub", "sensor_msgs/msg/Image"), info.subscribers)
        self.assertIn(("/service", "std_srvs/srv/SetBool"), info.service_servers)
        assert info.service_clients
        self.assertIn(("/client", "std_srvs/srv/Empty"), info.service_clients)
        assert info.action_servers
        self.assertIn(
            ("/action", "tf2_msgs/action/LookupTransform"), info.action_servers
        )
        assert info.action_clients
        self.assertIn(
            ("/action_client", "tf2_msgs/action/LookupTransform"),
            info.action_clients,
        )

    def test_get_topic_info(self):
        topic_name = "/topic"
        info = self.ROS.get_topic_info(topic_name)
        self.assertEqual(info.name, topic_name)
        self.assertEqual(info.types, ["std_msgs/msg/String"])
        self.assertEqual(info.publishers, [("/dummy_node1", "std_msgs/msg/String")])
        self.assertEqual(info.subscribers, [("/dummy_node2", "std_msgs/msg/String")])

    def test_get_service_info(self):
        service_name = "/service"
        info = self.ROS.get_service_info(service_name)
        self.assertEqual(info.name, service_name)
        self.assertEqual(info.types, ["std_srvs/srv/SetBool"])
        self.assertIsNone(info.servers)

    def test_get_action_info(self):
        action_name = "/action"
        info = self.ROS.get_action_info(action_name)
        self.assertEqual(info.name, action_name)
        self.assertEqual(info.types, ["tf2_msgs/action/LookupTransform"])
        self.assertEqual(
            info.servers, [("/dummy_node1", "tf2_msgs/action/LookupTransform")]
        )
        self.assertEqual(
            info.clients, [("/dummy_node2", "tf2_msgs/action/LookupTransform")]
        )


if __name__ == "__main__":
    unittest.main()
