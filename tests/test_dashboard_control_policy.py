import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from dashboard_control_policy import decide_request, safety_health_created_monotonic


class DashboardControlPolicyTests(unittest.TestCase):
    def decide(self, request, mission=0.1, safety=0.1):
        return decide_request(request, mission_status_age=mission,
                              safety_health_age=safety)

    def test_regular_mission_command_routes_only_with_fresh_status(self):
        self.assertEqual(self.decide({"action": "PAUSE"})["route"], "mission")
        self.assertEqual(self.decide({"action": "PAUSE"}, mission=1.51)["reason"],
                         "MISSION_STATUS_STALE_OR_UNKNOWN")
        self.assertEqual(self.decide({"action": "PAUSE"}, mission=None)["reason"],
                         "MISSION_STATUS_STALE_OR_UNKNOWN")

    def test_cancel_requires_confirmation_and_fresh_mission(self):
        self.assertEqual(self.decide({"action": "CANCEL"})["reason"],
                         "EXPLICIT_CONFIRMATION_REQUIRED")
        accepted = self.decide({"action": "CANCEL", "confirmed": True})
        self.assertEqual((accepted["accepted"], accepted["command"]), (True, "CANCEL"))

    def test_estop_is_confirmed_but_not_blocked_by_stale_telemetry(self):
        response = self.decide({"action": "ESTOP", "confirmed": True},
                               mission=None, safety=None)
        self.assertEqual(response["route"], "emergency_stop")
        self.assertEqual(self.decide({"action": "ESTOP"})["reason"],
                         "EXPLICIT_CONFIRMATION_REQUIRED")

    def test_reset_requires_confirmation_and_fresh_mission_and_safety(self):
        request = {"action": "RESET_ESTOP", "confirmed": True}
        self.assertEqual(self.decide(request)["route"], "reset_then_mission")
        self.assertEqual(self.decide(request, mission=None)["reason"],
                         "MISSION_STATUS_STALE_OR_UNKNOWN")
        self.assertEqual(self.decide(request, safety=0.51)["reason"],
                         "SAFETY_HEALTH_STALE_OR_UNKNOWN")

    def test_arm_startup_requires_fresh_health(self):
        self.assertEqual(self.decide({"action": "ARM_STARTUP"})["route"],
                         "arm_startup")
        self.assertEqual(self.decide({"action": "ARM_STARTUP"}, safety=None)["reason"],
                         "SAFETY_HEALTH_STALE_OR_UNKNOWN")

    def test_delivery_id_and_action_syntax_are_restricted(self):
        self.assertEqual(self.decide({"action": "DELIVER", "delivery_id": "sample-01"})["command"],
                         "DELIVER sample-01")
        self.assertEqual(self.decide({"action": "DELIVER", "delivery_id": "../bad"})["reason"],
                         "INVALID_DELIVERY_ID")
        self.assertEqual(self.decide({"action": "FAIL"})["reason"],
                         "ACTION_NOT_ALLOWED")
        self.assertEqual(self.decide({"action": "pause"})["reason"],
                         "ACTION_MUST_BE_UPPERCASE_STRING")

    def test_safety_health_source_clock_requires_valid_schema_and_stamp(self):
        self.assertEqual(safety_health_created_monotonic(
            '{"schema":1,"created_monotonic":42.5}'), 42.5)
        self.assertIsNone(safety_health_created_monotonic("not json"))
        self.assertIsNone(safety_health_created_monotonic(
            '{"schema":2,"created_monotonic":42.5}'))


if __name__ == "__main__":
    unittest.main()
