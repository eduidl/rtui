from __future__ import annotations

import subprocess as sp
import typing as t
import unittest
import warnings

try:
    import rospy
except ImportError:
    pass

from rtui.ros import init_ros, is_ros1

TestCase: t.Type = unittest.TestCase if is_ros1() else object


def ignore_warnings(test_func):
    def do_test(self, *args, **kwargs):
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", ResourceWarning)
            test_func(self, *args, **kwargs)

    return do_test


class TestRos1Interface(TestCase):
    NODE1: t.ClassVar[sp.Popen | None] = None
    NODE2: t.ClassVar[sp.Popen | None] = None

    @classmethod
    def setUpClass(cls) -> None:
        cls.NODE1 = sp.Popen("python3 tests/ros1/node/dummy_node1.py".split())
        cls.NODE2 = sp.Popen("python3 tests/ros1/node/dummy_node2.py".split())
        cls.ROS = init_ros()
        rospy.sleep(3)

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

    def test_list_topics(self):
        topics = self.ROS.list_topics()
        self.assertIn("/topic", topics)
        self.assertIn("/pub", topics)
        self.assertIn("/sub", topics)

    def test_list_services(self):
        services = self.ROS.list_services()
        self.assertIn("/server", services)
        self.assertNotIn("/client", services)

    def test_get_node_info(self):
        node_name = "/dummy_node1"
        info = self.ROS.get_node_info(node_name)
        self.assertEqual(info.name, node_name)
        self.assertIn(("/topic", "std_msgs/String"), info.publishers)
        self.assertIn(("/pub", "std_msgs/Int32"), info.publishers)
        self.assertIn(("/sub", "sensor_msgs/Image"), info.subscribers)
        self.assertIn(("/server", "std_srvs/SetBool"), info.service_servers)
        self.assertIsNone(info.service_clients)
        self.assertIsNone(info.action_servers)
        self.assertIsNone(info.action_clients)

    def test_get_topic_info(self):
        topic_name = "/topic"
        info = self.ROS.get_topic_info(topic_name)
        self.assertEqual(info.name, topic_name)
        self.assertEqual(info.types, ["std_msgs/String"])
        self.assertEqual(info.publishers, [("/dummy_node1", None)])
        self.assertEqual(info.subscribers, [("/dummy_node2", None)])

    def test_get_service_info(self):
        service_name = "/server"
        info = self.ROS.get_service_info(service_name)
        self.assertEqual(info.name, service_name)
        self.assertEqual(info.types, ["std_srvs/SetBool"])
        self.assertEqual(info.servers, ["/dummy_node1"])

    @ignore_warnings
    def test_subscribe_topic(self):
        class Count:
            count: int = 0

            def increment(self):
                self.count += 1

        count = Count()

        def callback(msg):
            self.assertEqual(msg.data, 42)
            count.increment()

        sub = self.ROS.subscribe_topic("/pub", callback)

        rospy.sleep(0.1)

        self.assertGreater(count.count, 5)
        self.ROS.unregister_subscriber(sub)

    @ignore_warnings
    def test_unregister_subscriber(self):
        topic_name = "/topic"
        sub = self.ROS.subscribe_topic(topic_name, lambda x: ())

        info = self.ROS.get_topic_info(topic_name)
        self.assertTrue(
            any(topic.startswith("/rtui_node") for topic, _ in info.subscribers)
        )

        self.ROS.unregister_subscriber(sub)

        info = self.ROS.get_topic_info(topic_name)
        self.assertTrue(
            all(not topic.startswith("/rtui_node") for topic, _ in info.subscribers)
        )


if __name__ == "__main__":
    unittest.main()
