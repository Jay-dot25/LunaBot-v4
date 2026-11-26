#!/usr/bin/env python3
"""Derive a Phase L world with an explicit sensor renderer."""
import argparse
from pathlib import Path
import xml.etree.ElementTree as ET


def prepare(source, destination, renderer):
    world = ET.parse(source)
    engine = world.find(".//plugin[@name='ignition::gazebo::systems::Sensors']/render_engine")
    if engine is None:
        raise ValueError("lunar world has no sensor renderer setting")
    engine.text = renderer
    for uri in world.findall(".//uri"):
        if uri.text and uri.text.startswith("file://meshes/"):
            uri.text = (source.resolve().parent / uri.text.removeprefix("file://")).as_uri()
    world.write(destination, encoding="utf-8", xml_declaration=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("destination", type=Path)
    parser.add_argument("renderer", choices=("ogre", "ogre2"))
    args = parser.parse_args()
    prepare(args.source, args.destination, args.renderer)


if __name__ == "__main__":
    main()
