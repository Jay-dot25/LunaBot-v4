import json
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from build_evaluation_report import build_report, render_markdown, render_plots, validate_report


class EvaluationReportTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.report = build_report(ROOT, "test-p10-report")
        cls.schema = json.loads((ROOT / "config/evaluation/report.schema.json").read_text(encoding="utf-8"))
        cls.catalog = json.loads((ROOT / "config/evaluation/metrics.json").read_text(encoding="utf-8"))

    def test_every_catalog_metric_has_value_or_unavailable_reason(self):
        expected = {item["id"] for item in self.catalog["metrics"]}
        self.assertEqual(validate_report(self.report, self.schema, expected), [])
        self.assertEqual(len(self.report["metrics"]), len(expected))

    def test_all_inputs_have_sha256_provenance(self):
        self.assertGreaterEqual(len(self.report["inputs"]), 8)
        self.assertTrue(all(len(item["sha256"]) == 64 for item in self.report["inputs"]))

    def test_report_discloses_limited_mission_and_fixture_scope(self):
        metrics = {item["id"]: item for item in self.report["metrics"]}
        self.assertEqual(metrics["mission_success"]["status"], "limited")
        self.assertEqual(metrics["collision_count"]["status"], "unavailable")
        self.assertTrue(metrics["collision_count"]["unavailable_reason"])
        self.assertEqual(self.report["trial_summary"]["p8_live_roundtrips"], 3)

    def test_markdown_and_plots_are_renderable_artifacts(self):
        markdown = render_markdown(self.report)
        plots = render_plots(self.report, ROOT)
        self.assertIn("Input provenance", markdown)
        self.assertEqual(len(plots), 4)
        self.assertTrue(all(value.startswith('<svg xmlns="http://www.w3.org/2000/svg"') for value in plots.values()))


if __name__ == "__main__":
    unittest.main()
