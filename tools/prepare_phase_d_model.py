#!/usr/bin/env python3
"""Derive a level navigation LiDAR profile; preserve the baseline model."""
import argparse
import xml.etree.ElementTree as ET
from pathlib import Path

def prepare(source, destination, watchdog_timeout_sec=None,
            wsl_gui_budget=False, *, lidar_height=-0.60):
    tree = ET.parse(source)
    sensor = tree.find(".//sensor[@name='lidar']")
    if sensor is None or sensor.get('type') != 'gpu_lidar':
        raise ValueError('Expected the baseline GPU LiDAR')
    # Keep a planar SLAM scan. Phase L lowers the scan plane further to
    # intersect its low rock cluster.
    sensor.find('pose').text = f'0.35 0 {lidar_height:.2f} 0 0 0'
    if wsl_gui_budget:
        camera = tree.find(".//sensor[@name='rgb_camera']")
        depth = tree.find(".//sensor[@name='depth_camera']")
        horizontal = sensor.find('./ray/scan/horizontal')
        if camera is None or depth is None or horizontal is None:
            raise ValueError('Expected RGB, depth, and horizontal LiDAR sensors')
        # Bound WSL llvmpipe rendering and Python perception cost. The rover
        # keeps live RGB-D mapping at a modest rate; the independent LiDAR and
        # command watchdogs preserve local obstacle stops and safe motion.
        for sensor_camera in (camera, depth):
            image = sensor_camera.find('./camera/image')
            if image is None:
                raise ValueError('Expected camera image settings')
            image.find('width').text = '96'
            image.find('height').text = '72'
            sensor_camera.find('update_rate').text = '2'
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
    parser.add_argument('--lidar-height', type=float, default=-0.60)
    args = parser.parse_args()
    prepare(args.source, args.destination, args.watchdog_timeout_sec,
            args.wsl_gui_budget, lidar_height=args.lidar_height)
