from __future__ import annotations

import subprocess as sp
import typing as t
import unittest
import warnings

import rospy

from rtui_app.ros.interface.ros1 import Ros1


def ignore_warnings(test_func):
    def do_test(self, *args, **kwargs):
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", ResourceWarning)
            test_func(self, *args, **kwargs)

    return do_test


class TestRos1Interface(unittest.TestCase):
    NODE1: t.ClassVar[sp.Popen | None] = None
    NODE2: t.ClassVar[sp.Popen | None] = None

    @classmethod
    def setUpClass(cls) -> None:
        cls.NODE1 = sp.Popen("python3 tests/ros1/node/dummy_node1.py".split())
        cls.NODE2 = sp.Popen("python3 tests/ros1/node/dummy_node2.py".split())
        cls.ROS = Ros1()
        rospy.sleep(3)

    @classmethod
    def tearDownClass(cls) -> None:
        if cls.NODE1:
            cls.NODE1.kill()
        if cls.NODE2:
            cls.NODE2.kill()

        cls.ROS.terminate()

    def test_get_node_publishers(self):
        publishers = self.ROS.get_node_publishers("/dummy_node1")
        self.assertIn(("/topic", "std_msgs/String"), publishers)
        self.assertIn(("/pub", "std_msgs/Int32"), publishers)

    def test_get_node_subscribers(self):
        subscribers = self.ROS.get_node_subscribers("/dummy_node1")
        self.assertIn(("/sub", "sensor_msgs/Image"), subscribers)

    def test_get_node_service_servers(self):
        servers = self.ROS.get_node_service_servers("/dummy_node1")
        self.assertIn(("/server", "std_srvs/SetBool"), servers)

    def test_get_node_service_clients(self):
        # Not supported
        self.assertIsNone(self.ROS.get_node_service_clients("/dummy_node1"))

    def test_get_node_action_servers(self):
        # Not supported
        self.assertIsNone(self.ROS.get_node_action_servers("/dummy_node1"))

    def test_get_node_action_clients(self):
        # Not supported
        self.assertIsNone(self.ROS.get_node_action_clients("/dummy_node1"))

    def test_get_topic_types(self):
        topic_types = self.ROS.get_topic_types("/topic")
        self.assertEqual(topic_types, ["std_msgs/String"])

    def test_get_topic_publishers(self):
        publishers = self.ROS.get_topic_publishers("/topic")
        self.assertEqual(publishers, [("/dummy_node1", None)])

    def test_get_topic_subscribers(self):
        subscribers = self.ROS.get_topic_subscribers("/topic")
        self.assertEqual(subscribers, [("/dummy_node2", None)])

    def test_get_service_types(self):
        self.assertEqual(self.ROS.get_service_types("/server"), ["std_srvs/SetBool"])

    def test_get_service_servers(self):
        self.assertEqual(
            self.ROS.get_service_servers("/server"), [("/dummy_node1", None)]
        )

    def test_list_nodes(self):
        nodes = self.ROS.list_nodes()
        self.assertIn("/dummy_node1", nodes)
        self.assertIn("/dummy_node2", nodes)

    def test_list_topics(self):
        topics = self.ROS.list_topics()
        self.assertIn("/topic", topics)
        self.assertIn("/pub", topics)
        self.assertIn("/sub", topics)

    def test_list_services(self):
        services = self.ROS.list_services()
        self.assertIn("/server", services)
        self.assertNotIn("/client", services)


if __name__ == "__main__":
    unittest.main()
