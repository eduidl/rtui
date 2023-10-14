import unittest

from rtui.ros import is_ros1, is_ros2

from .test_history import TestHistory

if is_ros1():
    from .ros1 import *

if is_ros2():
    from .ros2 import *


if __name__ == "__main__":
    unittest.main()
