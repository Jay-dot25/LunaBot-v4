"""Startup arming requires a continuing zero-command stream."""
import sys
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

import rclpy
from geometry_msgs.msg import Twist

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from control_odometry import ControlNode


class StartupArmingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = ControlNode("/p8_test/input", "/p8_test/output",
                                .45, 1., .4, .8, .5, 30.,
                                auto_arm_startup=True)
        self.node.fault = "STARTUP_INHIBIT"
        self.node.health = SimpleNamespace(error=lambda **kwargs: "")
        self.node.count_publishers = lambda topic: 1

    def tearDown(self):
        self.node.destroy_node()

    def test_healthy_sensors_without_commands_do_not_arm(self):
        self.node._tick()
        self.assertEqual(self.node.fault, "STARTUP_INHIBIT")

    def test_stale_zero_does_not_arm(self):
        self.node._input_cb(Twist())
        self.node.last_input = time.monotonic() - 1.
        self.node._tick()
        self.assertEqual(self.node.fault, "STARTUP_INHIBIT")

    def test_fresh_zero_arms_with_zero_output(self):
        self.node._input_cb(Twist())
        self.node._tick()
        self.assertEqual(self.node.fault, "")
        self.assertEqual((self.node.output_v, self.node.output_w), (0., 0.))

    def test_fresh_zero_does_not_clear_runtime_latch(self):
        self.node.fault = "HEALTH_ACTUATOR_TIMEOUT_LATCHED"
        self.node._input_cb(Twist())
        self.node._tick()
        self.assertEqual(self.node.fault, "HEALTH_ACTUATOR_TIMEOUT_LATCHED")


if __name__ == "__main__":
    unittest.main()
