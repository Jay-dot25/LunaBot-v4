import json
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from validate_evaluation_contract import validate_contract


class EvaluationContractTests(unittest.TestCase):
    def setUp(self):
        self.contract = json.loads(
            (ROOT / "config/evaluation/metrics.json").read_text(encoding="utf-8")
        )

    def test_repository_contract_is_complete_and_truth_isolated(self):
        self.assertEqual(validate_contract(self.contract, ROOT), [])

    def test_rejects_truth_bridge(self):
        self.contract["truth_boundary"]["ros_bridge_enabled"] = True
        errors = validate_contract(self.contract, ROOT)
        self.assertTrue(any("must not be bridged" in error for error in errors))

    def test_rejects_missing_required_metric(self):
        self.contract["metrics"] = [
            item for item in self.contract["metrics"] if item["id"] != "collision_count"
        ]
        errors = validate_contract(self.contract, ROOT)
        self.assertTrue(any("collision_count" in error for error in errors))

    def test_rejects_truth_interface_in_operational_source(self):
        self.contract["truth_boundary"]["operational_paths"] = ["scripts"]
        self.contract["truth_boundary"]["forbidden_operational_markers"] = ["gazebo_truth"]
        with tempfile.TemporaryDirectory() as temporary:
            fake_root = Path(temporary)
            (fake_root / "scripts").mkdir()
            (fake_root / "scripts/controller.py").write_text("topic = 'gazebo_truth'\n", encoding="utf-8")
            errors = validate_contract(self.contract, fake_root)
        self.assertTrue(any("forbidden truth interface" in error for error in errors))


if __name__ == "__main__":
    unittest.main()
