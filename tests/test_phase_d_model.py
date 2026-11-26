import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
import sys
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
from prepare_phase_d_model import prepare

class NavigationSensorProfileTests(unittest.TestCase):
    def test_only_lidar_pose_changes_and_matches_launch_tf(self):
        source = ROOT / 'src/lunabot_gazebo/models/lunabot_v4/model.sdf'
        original_bytes = source.read_bytes()
        with tempfile.TemporaryDirectory() as directory:
            destination = Path(directory) / 'model.sdf'
            prepare(source, destination)
            original = ET.fromstring(original_bytes)
            derived = ET.parse(destination).getroot()
            pose = derived.find(".//sensor[@name='lidar']/pose")
            self.assertEqual('0.35 0 -0.60 0 0 0', pose.text)
            watchdog = derived.find(".//plugin[@name='lunabot::CommandWatchdog']/timeout_sec")
            self.assertEqual('0.25', watchdog.text)
            launch = (ROOT / 'scripts/launch-d.sh').read_text()
            self.assertIn('sensor_head|lunabot_v4/sensor_head/lidar|' + pose.text, launch)
            pose.text = original.find(".//sensor[@name='lidar']/pose").text
            self.assertEqual(ET.tostring(original), ET.tostring(derived))
        self.assertEqual(original_bytes, source.read_bytes())

    def test_phase_l_overrides_are_local_to_derived_model(self):
        source = ROOT / 'src/lunabot_gazebo/models/lunabot_v4/model.sdf'
        with tempfile.TemporaryDirectory() as directory:
            destination = Path(directory) / 'model.sdf'
            prepare(source, destination, watchdog_timeout_sec=1.0, lidar_height=-0.90)
            derived = ET.parse(destination).getroot()
            watchdog = derived.find(".//plugin[@name='lunabot::CommandWatchdog']/timeout_sec")
            self.assertEqual('1.0', watchdog.text)
            self.assertEqual('0.35 0 -0.90 0 0 0',
                             derived.find(".//sensor[@name='lidar']/pose").text)
            baseline = ET.parse(source).getroot()
            original = baseline.find(".//plugin[@name='lunabot::CommandWatchdog']/timeout_sec")
            self.assertEqual('0.25', original.text)
