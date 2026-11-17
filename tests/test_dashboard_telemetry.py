import json
import sys
import types
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from dashboard_telemetry import evaluate_topic, message_provenance, stamp_seconds


class DashboardTelemetryTests(unittest.TestCase):
    def test_ros_header_stamp_and_frame_are_preserved(self):
        msg = types.SimpleNamespace(header=types.SimpleNamespace(
            stamp=types.SimpleNamespace(sec=42, nanosec=250_000_000), frame_id="map"))
        self.assertEqual(message_provenance(msg), {
            "source_stamp_seconds": 42.25,
            "source_clock": "ros",
            "frame_id": "map",
            "stamp_origin": "message_header",
        })

    def test_tf_provenance_lists_transform_frames_and_stamp(self):
        stamp = types.SimpleNamespace(sec=9, nanosec=0)
        transform = types.SimpleNamespace(
            header=types.SimpleNamespace(stamp=stamp, frame_id="map"),
            child_frame_id="odom")
        result = message_provenance(types.SimpleNamespace(transforms=[transform]))
        self.assertEqual(result["source_stamp_seconds"], 9.0)
        self.assertEqual(result["source_frame_ids"], ["map", "odom"])

    def test_json_status_source_clock_is_kept_distinct(self):
        msg = types.SimpleNamespace(data=json.dumps({"created_monotonic": 77.5}))
        result = message_provenance(msg)
        self.assertEqual(result["source_clock"], "host_monotonic")
        self.assertEqual(result["source_stamp_seconds"], 77.5)

    def test_missing_status_stamp_is_explicit_and_receive_age_is_used(self):
        provenance = message_provenance(types.SimpleNamespace(data="READY"))
        self.assertEqual(provenance["stamp_origin"], "not_provided_by_message")
        row = evaluate_topic(
            topic="/status", message_type="std_msgs/msg/String",
            last_receive_monotonic=10.0, now_monotonic=10.75,
            source_stamp_seconds=None, source_clock=None, frame_id=None,
            source_nodes=["/mission"], stale_after_seconds=0.5,
        )
        self.assertEqual(row["state"], "stale")
        self.assertEqual(row["receive_age_seconds"], 0.75)
        self.assertEqual(row["source_nodes"], ["/mission"])

    def test_missing_feed_is_unknown_and_latched_feed_stays_valid(self):
        common = dict(topic="/tf_static", message_type="tf2_msgs/msg/TFMessage",
                      now_monotonic=100.0, source_stamp_seconds=None,
                      source_clock=None, frame_id=None, source_nodes=[], stale_after_seconds=1.0)
        missing = evaluate_topic(last_receive_monotonic=None, **common)
        latched = evaluate_topic(last_receive_monotonic=1.0, freshness_mode="latched", **common)
        self.assertEqual(missing["state"], "unknown")
        self.assertEqual(latched["state"], "fresh")

    def test_ros_source_age_and_frame_gates_are_applied(self):
        common = dict(
            topic="/map", message_type="nav_msgs/msg/OccupancyGrid",
            last_receive_monotonic=10.0, now_monotonic=10.1,
            source_stamp_seconds=9.7, source_clock="ros", frame_id="odom",
            source_nodes=["/slam"], stale_after_seconds=2.0,
            source_age_limit_seconds=0.25, now_ros_seconds=10.0,
            expected_frame="map",
        )
        stale = evaluate_topic(**common)
        self.assertAlmostEqual(stale["source_age_seconds"], 0.3)
        self.assertEqual(stale["state"], "unknown")
        self.assertEqual(stale["state_reason"], "expected_frame_mismatch")

        common["frame_id"] = "map"
        stale = evaluate_topic(**common)
        self.assertEqual(stale["state"], "stale")
        self.assertEqual(stale["state_reason"], "source_stamp_timeout")

        common["source_stamp_seconds"] = 10.1
        future = evaluate_topic(**common)
        self.assertEqual(future["state"], "unknown")
        self.assertEqual(future["state_reason"], "source_clock_ahead_of_adapter")

    def test_invalid_ros_stamp_is_rejected(self):
        stamp = types.SimpleNamespace(sec=-1, nanosec=0)
        self.assertIsNone(stamp_seconds(stamp))


if __name__ == "__main__":
    unittest.main()
