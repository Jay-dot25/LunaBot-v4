#!/usr/bin/env python3
"""Generate an isolated Gazebo world containing one known flat surface."""
from __future__ import annotations

import argparse
from pathlib import Path


def make_world(height: float, extent: float) -> str:
    bottom = height - 0.05
    return f'''<?xml version="1.0"?>
<sdf version="1.9">
  <world name="lunar_world">
    <physics name="1ms" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>1.0</real_time_factor></physics>
    <gravity>0 0 -1.62</gravity>
    <scene><ambient>0.06 0.06 0.06 1</ambient><background>0 0 0 1</background><shadows>true</shadows><grid>false</grid></scene>
    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <plugin filename="ignition-gazebo-imu-system" name="ignition::gazebo::systems::Imu"/>
    <plugin filename="ignition-gazebo-contact-system" name="ignition::gazebo::systems::Contact"/>
    <light type="directional" name="sun"><cast_shadows>true</cast_shadows><pose>0 0 200 0 0 0</pose><diffuse>0.96 0.96 0.96 1</diffuse><specular>0.18 0.18 0.18 1</specular><direction>-0.6 0.45 -0.35</direction></light>
    <model name="p6_known_plane">
      <static>true</static><pose>0 0 {bottom:.9f} 0 0 0</pose>
      <link name="surface">
        <collision name="collision"><geometry><box><size>{extent:.3f} {extent:.3f} 0.1</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>{extent:.3f} {extent:.3f} 0.1</size></box></geometry>
          <material><ambient>0.52 0.52 0.54 1</ambient><diffuse>0.52 0.52 0.54 1</diffuse><specular>0.04 0.04 0.04 1</specular></material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
'''


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--height-m", type=float, default=-2.788)
    parser.add_argument("--extent-m", type=float, default=40.0)
    args = parser.parse_args()
    if args.extent_m < 12.0:
        parser.error("extent must be at least 12 m to cover the configured range bands")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(make_world(args.height_m, args.extent_m), encoding="utf-8")
    print(f"wrote {args.output}: plane_z={args.height_m:.3f} m extent={args.extent_m:.1f} m")


if __name__ == "__main__":
    main()
