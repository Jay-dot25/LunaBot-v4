import json
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from waypoint_registry import RegistryError, load_registry, resolve_waypoint

REGISTRY = ROOT / "config/waypoints.json"
MAP_ID = json.loads(REGISTRY.read_text(encoding="utf-8"))["map"]["id"]
FRAME = "map"


def write_registry(directory: Path, data: dict) -> Path:
    data["map"]["metadata_path"] = str(
        (REGISTRY.parent / data["map"]["metadata_path"]).resolve())
    path = directory / "registry.json"
    path.write_text(json.dumps(data), encoding="utf-8")
    return path

class WaypointRegistryTests(unittest.TestCase):
    def test_habitat_and_science_are_registered(self):
        data = load_registry(REGISTRY, map_id=MAP_ID, frame_id=FRAME,
                             require_bound=True)
        self.assertEqual({"habitat", "science"}, set(data["waypoints_by_id"]))
        self.assertEqual("bound", data["waypoints_by_id"]["habitat"]["binding_status"])

    def test_wrong_map_id_is_rejected(self):
        with self.assertRaisesRegex(RegistryError, "map ID mismatch"):
            load_registry(REGISTRY, map_id="wrong_map", frame_id=FRAME)

    def test_wrong_map_frame_is_rejected(self):
        with self.assertRaisesRegex(RegistryError, "map frame mismatch"):
            load_registry(REGISTRY, map_id=MAP_ID, frame_id="odom")

    def test_waypoint_with_wrong_frame_is_rejected(self):
        data = json.loads(REGISTRY.read_text(encoding="utf-8"))
        data["waypoints"][0]["frame_id"] = "odom"
        with tempfile.TemporaryDirectory() as tmp:
            path = write_registry(Path(tmp), data)
            with self.assertRaisesRegex(RegistryError, "waypoint frame mismatch"):
                load_registry(path, map_id=MAP_ID, frame_id=FRAME)

    def test_unbound_goal_cannot_be_dispatched(self):
        source = json.loads(REGISTRY.read_text(encoding="utf-8"))
        source["waypoints"][0].update(
            pose=None, binding_status="unbound",
            binding_reason="No verified pose in this test fixture.")
        with tempfile.TemporaryDirectory() as tmp:
            path = write_registry(Path(tmp), source)
            data = load_registry(path, map_id=MAP_ID, frame_id=FRAME)
        with self.assertRaisesRegex(RegistryError, "refusing to dispatch an unbound goal"):
            resolve_waypoint(data, "habitat")

    def test_require_bound_rejects_unbound_site(self):
        source = json.loads(REGISTRY.read_text(encoding="utf-8"))
        source["waypoints"][0].update(
            pose=None, binding_status="unbound",
            binding_reason="No verified pose in this test fixture.")
        with tempfile.TemporaryDirectory() as tmp:
            path = write_registry(Path(tmp), source)
            with self.assertRaisesRegex(RegistryError, "no verified pose"):
                load_registry(path, map_id=MAP_ID, frame_id=FRAME, require_bound=True)

if __name__ == "__main__":
    unittest.main()
