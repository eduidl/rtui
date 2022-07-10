import unittest

from geometry_msgs.msg import AccelWithCovariance
from std_msgs.msg import Header, UInt8MultiArray

from rtui.ros import get_ros_cls

ROS = get_ros_cls()


class TestRos1Format(unittest.TestCase):
    def test_format_header(self):
        msg = Header()
        msg.seq = 10
        msg.frame_id = "base_link"

        answer = '''
seq: 10
stamp:
  secs: 0
  nsecs: 0
frame_id: "base_link"'''
        self.assertEqual(ROS.format_msg(msg), answer)

    def test_format_uint8_array(self):
        msg = UInt8MultiArray()
        msg.data = bytes.fromhex("FFFF")

        answer = """
layout:
  dim: []
  data_offset: 0
data: [255, 255]"""
        self.assertEqual(ROS.format_msg(msg), answer)

        msg = UInt8MultiArray()
        msg.data = bytes.fromhex("F" * 100)

        answer = '''
layout:
  dim: []
  data_offset: 0
data: "<uint8[], length: 50>"'''
        self.assertEqual(ROS.format_msg(msg), answer)

    def test_format_fixed_array(self):
        msg = AccelWithCovariance()

        answer = '''
accel:
  linear:
    x: 0.0
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.0
covariance: "<float64[36]>"'''
        self.assertEqual(ROS.format_msg(msg), answer)


if __name__ == "__main__":
    unittest.main()
