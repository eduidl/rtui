import unittest
from os import environ

from .test_history import TestHistory

if environ.get("ROS_VERSION") == "1":
    from .ros1 import *

if environ.get("ROS_VERSION") == "2":
    from .ros2 import *


if __name__ == "__main__":
    unittest.main()
