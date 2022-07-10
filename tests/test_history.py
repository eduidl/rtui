import unittest

from rtui.utility.hisotry import History


def init_history() -> History:
    history = History(3)
    for i in range(1, 4):
        history.append(i)
    return history


class TestHistory(unittest.TestCase):
    def setUp(self):
        self.history = init_history()

    def test_history_current(self):
        self.assertEqual(self.history.current(), 3)

    def test_history_do_not_append_same_values_consecuitively(self):
        history = History(10)

        for _ in range(10):
            history.append(1)

        self.assertEqual(history.len(), 1)

    def test_history_back(self):
        self.assertEqual(self.history.back(), 2)
        self.assertEqual(self.history.back(), 1)

    def test_history_back_fail(self):
        self.history.back()
        self.history.back()
        self.assertIsNone(self.history.back())

    def test_history_back_fail_when_empty(self):
        history = History(3)

        self.assertIsNone(history.back())

    def test_history_forward(self):
        self.history.back()
        self.history.back()

        self.assertEqual(self.history.forward(), 2)
        self.assertEqual(self.history.forward(), 3)

    def test_history_forward_fail_when_latest(self):
        self.assertIsNone(self.history.forward())

    def test_history_forward_fail_when_empty(self):
        history = History(3)

        self.assertIsNone(history.forward())

    def test_history_append_clear_future_history(self):
        self.history.back()
        self.history.back()

        self.history.append(5)
        self.assertEqual(self.history.current(), 5)
        self.assertIsNone(self.history.forward())


if __name__ == "__main__":
    unittest.main()
