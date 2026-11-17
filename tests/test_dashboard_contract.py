import copy
import json
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from validate_dashboard_contract import validate


class DashboardContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.options = json.loads((ROOT / "config/dashboard/options.json").read_text(encoding="utf-8"))
        cls.contract = json.loads((ROOT / "config/dashboard/contracts.json").read_text(encoding="utf-8"))
        cls.adapter = json.loads((ROOT / "config/dashboard/adapter.json").read_text(encoding="utf-8"))
        cls.controls = json.loads((ROOT / "config/dashboard/controls.json").read_text(encoding="utf-8"))

    def test_candidate_comparison_and_six_view_contract_are_complete(self):
        self.assertEqual(validate(self.options, self.contract, self.adapter, self.controls), [])

    def test_rejects_writable_velocity_interface(self):
        contract = copy.deepcopy(self.contract)
        contract["write_contract"]["velocity_publish_allowed"] = True
        self.assertTrue(any("must not publish velocity" in error
                            for error in validate(self.options, contract)))

    def test_rejects_unregistered_topic(self):
        contract = copy.deepcopy(self.contract)
        contract["views"][0]["topics"].append("/cmd_vel")
        self.assertTrue(any("unknown or writable" in error
                            for error in validate(self.options, contract, self.adapter, self.controls)))

    def test_rejects_missing_view_and_missing_freshness_indicator(self):
        contract = copy.deepcopy(self.contract)
        contract["views"].pop()
        contract["views"][0]["show_receive_age"] = False
        errors = validate(self.options, contract, self.adapter, self.controls)
        self.assertTrue(any("exactly the six" in error for error in errors))
        self.assertTrue(any("receive age" in error for error in errors))

    def test_rejects_unallowlisted_or_incomplete_adapter(self):
        adapter = copy.deepcopy(self.adapter)
        adapter["topics"][0]["topic"] = "/world/lunar_world/pose/info"
        errors = validate(self.options, self.contract, adapter, self.controls)
        self.assertTrue(any("views 1-5 topic" in error for error in errors))
        self.assertTrue(any("not an allowlisted" in error for error in errors))

    def test_rejects_direct_dashboard_writes_to_unprotected_interfaces(self):
        contract = copy.deepcopy(self.contract)
        contract["views"][5]["write_topics"] = ["/lunabot/mission/command"]
        errors = validate(self.options, contract, self.adapter, self.controls)
        self.assertTrue(any("guarded dashboard request topic" in error for error in errors))

    def test_rejects_control_confirmation_policy_drift(self):
        controls = copy.deepcopy(self.controls)
        controls["confirmed_actions"].remove("RESET_ESTOP")
        errors = validate(self.options, self.contract, self.adapter, controls)
        self.assertTrue(any("same confirmations" in error for error in errors))


if __name__ == "__main__":
    unittest.main()
