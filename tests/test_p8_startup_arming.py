"""Startup arming requires a continuing zero-command stream."""
import sys
import time
import unittest
from concurrent.futures import Future
from itertools import count
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import rclpy
from geometry_msgs.msg import Twist

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import arm_safety
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


class StartupArmingClientTests(unittest.TestCase):
    def test_retries_when_the_first_service_response_is_lost(self):
        lost = Future()
        acknowledged = Future()
        acknowledged.set_result(SimpleNamespace(success=True, message="Already armed"))
        with (
            patch.object(arm_safety, "Node") as node_factory,
            patch.object(arm_safety, "rclpy") as ros,
            patch.object(arm_safety.time, "monotonic", side_effect=count(step=0.25)),
            patch.object(sys, "argv", ["arm_safety.py", "--timeout", "10"]),
        ):
            client = node_factory.return_value.create_client.return_value
            client.service_is_ready.return_value = True
            client.call_async.side_effect = [lost, acknowledged]
            ros.ok.return_value = True

            with self.assertRaises(SystemExit) as exit_context:
                arm_safety.main()

            self.assertEqual(exit_context.exception.code, 0)
            self.assertEqual(client.call_async.call_count, 2)
            client.remove_pending_request.assert_called_once_with(lost)


if __name__ == "__main__":
    unittest.main()
