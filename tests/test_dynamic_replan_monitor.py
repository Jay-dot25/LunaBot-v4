import sys
import unittest
from pathlib import Path as FilePath
from unittest.mock import patch

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

ROOT = FilePath(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from dynamic_replan_monitor import DynamicReplanMonitor


def make_path(points):
    path = Path()
    for x, y in points:
        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        path.poses.append(pose)
    return path


class DynamicReplanMonitorTests(unittest.TestCase):
    def test_obstacle_to_changed_plan_latency_is_recorded(self):
        monitor = DynamicReplanMonitor.__new__(DynamicReplanMonitor)
        monitor.passed = False
        monitor.last_signature = DynamicReplanMonitor._signature(
            make_path([(0.0, 0.0), (1.0, 0.0)]))
        monitor.revisions = 0
        monitor.total_motion = 0.2
        monitor.obstacle_detected_monotonic = 12.0
        monitor.obstacle_replan_latency_ms = None
        statuses = []
        monitor.publish_status = statuses.append
        monitor._evaluate = lambda: None
        with patch("dynamic_replan_monitor.time.monotonic", return_value=12.275):
            monitor._plan_callback(make_path([(0.0, 0.0), (1.0, 0.5)]))
        self.assertAlmostEqual(275.0, monitor.obstacle_replan_latency_ms)
        self.assertTrue(any("obstacle_replan_latency_ms=275.0" in status
                            for status in statuses))


if __name__ == "__main__":
    unittest.main()
