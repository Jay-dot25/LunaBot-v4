#!/usr/bin/env python3
"""Derive a level navigation LiDAR profile; preserve the baseline model."""
import argparse
import xml.etree.ElementTree as ET
from pathlib import Path

def prepare(source, destination, watchdog_timeout_sec=None,
            wsl_gui_budget=False):
    tree = ET.parse(source)
    sensor = tree.find(".//sensor[@name='lidar']")
    if sensor is None or sensor.get('type') != 'gpu_lidar':
        raise ValueError('Expected the baseline GPU LiDAR')
    # Intersect the physical low obstacle while providing a planar SLAM scan.
    # The sensor head sits high on the mast; -0.60 m left the horizontal scan
    # grazing above the trained rock cluster. Lower it another 0.15 m so the
    # scan plane crosses the rock bodies without pitching into the ground.
    sensor.find('pose').text = '0.35 0 -0.75 0 0 0'
    if wsl_gui_budget:
        camera = tree.find(".//sensor[@name='rgb_camera']")
        depth = tree.find(".//sensor[@name='depth_camera']")
        horizontal = sensor.find('./ray/scan/horizontal')
        if camera is None or depth is None or horizontal is None:
            raise ValueError('Expected RGB, depth, and horizontal LiDAR sensors')
        # Bound WSL llvmpipe rendering and scan-matching work. At 5 Hz / 360
        # samples, odometry can advance faster than slam_toolbox consumes
        # scans, leaving map->odom several simulation seconds behind.
        camera.find('update_rate').text = '5'
        depth.find('update_rate').text = '5'
        sensor.find('update_rate').text = '2'
        horizontal.find('samples').text = '180'
    if watchdog_timeout_sec is not None:
        if watchdog_timeout_sec <= 0:
            raise ValueError('watchdog timeout must be positive')
        watchdog = tree.find(".//plugin[@name='lunabot::CommandWatchdog']/timeout_sec")
        if watchdog is None:
            raise ValueError('Expected the actuator command watchdog')
        watchdog.text = str(watchdog_timeout_sec)
    Path(destination).parent.mkdir(parents=True, exist_ok=True)
    tree.write(destination, encoding='unicode', xml_declaration=True)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('destination', type=Path)
    parser.add_argument('--watchdog-timeout-sec', type=float)
    parser.add_argument('--wsl-gui-budget', action='store_true')
    args = parser.parse_args()
    prepare(args.source, args.destination, args.watchdog_timeout_sec,
            args.wsl_gui_budget)
