from __future__ import annotations

import subprocess as sp
import typing as t
import unittest
import warnings

from rtui.ros import init_ros, is_ros2

TestCase: t.Type = unittest.TestCase if is_ros2() else object


def ignore_warnings(test_func):
    def do_test(self, *args, **kwargs):
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", ResourceWarning)
            test_func(self, *args, **kwargs)

    return do_test


class TestRos2Interface(TestCase):
    NODE1: t.ClassVar[sp.Popen | None] = None
    NODE2: t.ClassVar[sp.Popen | None] = None

    @classmethod
    def setUpClass(cls) -> None:
        cls.NODE1 = sp.Popen("python3 tests/ros2/node/dummy_node1.py".split())
        cls.NODE2 = sp.Popen("python3 tests/ros2/node/dummy_node2.py".split())
        cls.ROS = init_ros()

        rate = cls.ROS.node.create_rate(3)
        rate.sleep()

    @classmethod
    def tearDownClass(cls) -> None:
        if cls.NODE1:
            cls.NODE1.kill()
        if cls.NODE2:
            cls.NODE2.kill()

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

    def test_get_topic_info_multi_types(self):
        topic_name = "/pub_dup"
        info = self.ROS.get_topic_info(topic_name)
        self.assertEqual(info.name, topic_name)
        self.assertSetEqual(
            set(info.types), set(("std_msgs/msg/String", "std_msgs/msg/Int32"))
        )
        self.assertSetEqual(
            set(info.publishers),
            set(
                (
                    ("/dummy_node1", "std_msgs/msg/String"),
                    ("/dummy_node2", "std_msgs/msg/Int32"),
                )
            ),
        )

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
