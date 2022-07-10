import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, String
from std_srvs.srv import Empty, SetBool
from tf2_msgs.action import LookupTransform


def msg_callback(_msg):
    pass


class DummyNode1(Node):
    def __init__(self):
        super().__init__(
            "dummy_node1",
            enable_rosout=False,
            start_parameter_services=False,
            parameter_overrides=[],
        )
        self.pub1 = self.create_publisher(String, "topic", 1)
        self.pub2 = self.create_publisher(Int32, "pub", 1)
        self.pub3 = self.create_publisher(String, "pub_dup", 1)
        self.sub = self.create_subscription(Image, "sub", msg_callback, 1)
        self.server = self.create_service(SetBool, "service", lambda x, y: ())
        self.client = self.create_client(Empty, "client")
        self.action_server = ActionServer(self, LookupTransform, "action", lambda x: ())
        self.action_client = ActionClient(self, LookupTransform, "action_client")


def main():
    rclpy.init()
    rclpy.spin(DummyNode1())


if __name__ == "__main__":
    main()
