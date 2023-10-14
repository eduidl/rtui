import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_msgs.action import LookupTransform


def msg_callback(_msg):
    pass


class DummyNode2(Node):
    def __init__(self):
        super().__init__(
            "dummy_node2",
            enable_rosout=False,
            start_parameter_services=False,
            parameter_overrides=[],
        )

        self.sub = self.create_subscription(String, "topic", msg_callback, 1)
        self.client = self.create_client(SetBool, "service")
        self.action_client = ActionClient(self, LookupTransform, "action")


def main():
    rclpy.init()
    rclpy.spin(DummyNode2())


if __name__ == "__main__":
    main()
