#!/usr/bin/env python3
"""Create a lunar-themed visual rover variant while preserving its mechanics."""
from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "src/lunabot_gazebo/models/lunabot_v4/model.sdf"
MODEL_DIR = ROOT / "src/lunabot_gazebo/models/lunabot_lunar_design"
DESTINATION = MODEL_DIR / "model.sdf"

CERAMIC = (0.70, 0.53, 0.31, 1.0)      # dusty lunar ochre armor
TITANIUM = (0.76, 0.69, 0.54, 1.0)     # warm regolith-dusted metal
BRASS = (0.84, 0.65, 0.30, 1.0)       # muted lunar-gold accents
GRAPHITE = (0.055, 0.050, 0.043, 1.0)  # dark, low-glare track hardware
BLUE = (0.025, 0.10, 0.22, 1.0)        # deep blue optical glass / solar cells
BLUE_HIGHLIGHT = (0.30, 0.62, 0.78, 1.0)
CAMERA_X = 0.270
CAMERA_Y = 0.092
CAMERA_Z = 0.034


def set_material(visual: ET.Element, color: tuple[float, float, float, float]) -> None:
    material = visual.find("material")
    if material is None:
        material = ET.SubElement(visual, "material")
    for name in ("ambient", "diffuse"):
        child = material.find(name)
        if child is None:
            child = ET.SubElement(material, name)
        child.text = " ".join(f"{component:.3f}" for component in color)
    specular = material.find("specular")
    if specular is None:
        specular = ET.SubElement(material, "specular")
    specular.text = "0.16 0.17 0.17 1"


def recolor_link(root: ET.Element, link_name: str, colors: dict[str, tuple]) -> None:
    link = root.find(f".//link[@name='{link_name}']")
    if link is None:
        raise ValueError(f"missing baseline link {link_name}")
    for visual in link.findall("visual"):
        color = colors.get(visual.get("name", ""))
        if color:
            set_material(visual, color)


def add_visual(link: ET.Element, name: str, pose: tuple[float, ...],
               size: tuple[float, float, float],
               color: tuple[float, float, float, float]) -> ET.Element:
    visual = ET.SubElement(link, "visual", {"name": name})
    ET.SubElement(visual, "pose").text = " ".join(f"{v:.4f}" for v in pose)
    geometry = ET.SubElement(visual, "geometry")
    ET.SubElement(ET.SubElement(geometry, "box"), "size").text = (
        " ".join(f"{v:.4f}" for v in size))
    set_material(visual, color)
    return visual


def add_cylinder_visual(link: ET.Element, name: str, pose: tuple[float, ...],
                        radius: float, length: float,
                        color: tuple[float, float, float, float]) -> None:
    visual = ET.SubElement(link, "visual", {"name": name})
    ET.SubElement(visual, "pose").text = " ".join(f"{v:.4f}" for v in pose)
    geometry = ET.SubElement(visual, "geometry")
    cylinder = ET.SubElement(geometry, "cylinder")
    ET.SubElement(cylinder, "radius").text = f"{radius:.4f}"
    ET.SubElement(cylinder, "length").text = f"{length:.4f}"
    set_material(visual, color)


def main() -> None:
    tree = ET.parse(SOURCE)
    root = tree.getroot()
    model = root.find("model")
    if model is None or model.get("name") != "lunabot_v4":
        raise ValueError("unexpected baseline model identity")

    # Keep the entity and every physical link, joint, collision, sensor, and
    # plugin intact so the existing ROS/Gazebo interfaces continue to work.
    physical_signature = tuple(
        (tag, tuple(node.get("name", "") for node in model.findall(tag)))
        for tag in ("link", "joint", "plugin")
    )
    sensor_signature = tuple(
        (sensor.get("name"), sensor.get("type"))
        for sensor in model.findall(".//sensor")
    )

    recolors = {
        "chassis": {"body_visual": CERAMIC},
        "lower_armor": {"armor_visual": GRAPHITE},
        "upper_deck": {"deck_visual": TITANIUM},
        "front_armor": {"front_armor_visual": BRASS},
        "left_side_rail": {"rail_visual": GRAPHITE},
        "right_side_rail": {"rail_visual": GRAPHITE},
        "left_rocker": {"rocker_visual": GRAPHITE},
        "right_rocker": {"rocker_visual": GRAPHITE},
        "left_bogie": {"bogie_visual": TITANIUM},
        "right_bogie": {"bogie_visual": TITANIUM},
        "left_front_knuckle": {"knuckle_visual": BRASS},
        "right_front_knuckle": {"knuckle_visual": BRASS},
        "left_rear_knuckle": {"knuckle_visual": BRASS},
        "right_rear_knuckle": {"knuckle_visual": BRASS},
        "left_front_wheel": {"wheel_visual": GRAPHITE},
        "right_front_wheel": {"wheel_visual": GRAPHITE},
        "left_middle_wheel": {"wheel_visual": GRAPHITE},
        "right_middle_wheel": {"wheel_visual": GRAPHITE},
        "left_rear_wheel": {"wheel_visual": GRAPHITE},
        "right_rear_wheel": {"wheel_visual": GRAPHITE},
        "left_solar_panel": {"panel": BLUE},
        "right_solar_panel": {"panel": BLUE},
        "sensor_mast": {"mast": BRASS},
        "mast_pan_link": {"pan_visual": TITANIUM},
        "antenna": {"antenna_stem": TITANIUM},
    }
    for link_name, colors in recolors.items():
        recolor_link(root, link_name, colors)

    chassis = root.find(".//link[@name='chassis']")
    head = root.find(".//link[@name='sensor_head']")
    if chassis is None or head is None:
        raise ValueError("baseline chassis or sensor head is missing")

    # Build a WALL-E-like binocular head. Each camera is centered in its own
    # eye lens; RGB is the rover's left eye and depth is the right eye.
    head_visual = head.find("visual[@name='head']")
    if head_visual is None:
        raise ValueError("baseline sensor head visual is missing")
    head_visual.find("geometry/box/size").text = "0.36 0.32 0.20"
    set_material(head_visual, CERAMIC)
    for side, y in (("left", CAMERA_Y), ("right", -CAMERA_Y)):
        add_cylinder_visual(head, f"lunar_eye_bezel_{side}",
                            (0.207, y, CAMERA_Z, 0.0, 1.5708, 0.0),
                            0.084, 0.060, GRAPHITE)
        add_cylinder_visual(head, f"lunar_eye_lens_{side}",
                            (0.242, y, CAMERA_Z, 0.0, 1.5708, 0.0),
                            0.067, 0.026, BRASS)
        add_cylinder_visual(head, f"lunar_eye_glass_{side}",
                            (0.258, y, CAMERA_Z, 0.0, 1.5708, 0.0),
                            0.052, 0.008, BLUE)
        add_cylinder_visual(head, f"lunar_eye_glint_{side}",
                            (0.264, y - 0.014, CAMERA_Z + 0.016, 0.0, 1.5708, 0.0),
                            0.008, 0.003, BLUE_HIGHLIGHT)

    rgb_camera = head.find("sensor[@name='rgb_camera']")
    depth_camera = head.find("sensor[@name='depth_camera']")
    if rgb_camera is None or depth_camera is None:
        raise ValueError("baseline RGB-D sensors are missing")
    rgb_camera.find("pose").text = (
        f"{CAMERA_X:.3f} {CAMERA_Y:.3f} {CAMERA_Z:.3f} 0 0.25 0")
    depth_camera.find("pose").text = (
        f"{CAMERA_X:.3f} {-CAMERA_Y:.3f} {CAMERA_Z:.3f} 0 0.25 0")

    # Front service hatch and restrained gold maintenance markings.
    add_visual(chassis, "lunar_front_fascia", (0.485, 0.0, -0.005, 0, 0, 0),
               (0.020, 0.50, 0.17), CERAMIC)
    add_visual(chassis, "lunar_front_service_panel", (0.498, 0.0, -0.005, 0, 0, 0),
               (0.012, 0.26, 0.12), GRAPHITE)
    add_visual(chassis, "lunar_chest_display", (0.508, 0.0, -0.005, 0, 0, 0),
               (0.008, 0.145, 0.090), BLUE)
    add_visual(chassis, "lunar_chest_status_light", (0.514, 0.0, 0.027, 0, 0, 0),
               (0.004, 0.070, 0.008), BLUE_HIGHLIGHT)
    for side, y in (("left", 0.178), ("right", -0.178)):
        add_visual(chassis, f"lunar_front_marker_{side}",
                   (0.506, y, -0.005, 0, 0, 0), (0.010, 0.022, 0.13), BRASS)
    add_visual(chassis, "lunar_top_service_hatch", (-0.020, 0.0, 0.225, 0, 0, 0),
               (0.54, 0.38, 0.028), CERAMIC)
    for side, y in (("left", 0.105), ("right", -0.105)):
        add_visual(chassis, f"lunar_top_hatch_stripe_{side}",
                   (0.255, y, 0.225, 0, 0, 0), (0.020, 0.070, 0.030), BRASS)

    # Track outlines and individual raised cleats. These are visuals only:
    # the original six driven wheels and rocker/bogie physics remain unchanged.
    for side, y_sign in (("left", 1.0), ("right", -1.0)):
        # A broad dark side belt makes the wheel modules read as continuous
        # tracks from the front view and fills the visual gap below the body.
        # Its lower edge matches the measured tire contact plane (z ~= -0.428).
        belt_y = 0.45 * y_sign
        y = 0.526 * y_sign
        add_visual(chassis, f"lunar_track_belt_{side}",
                   (0.0, belt_y, -0.264, 0, 0, 0),
                   (0.90, 0.132, 0.328), GRAPHITE)
        add_visual(chassis, f"lunar_track_top_{side}",
                   (0.0, y, -0.085, 0, 0, 0), (0.90, 0.018, 0.030), GRAPHITE)
        add_visual(chassis, f"lunar_track_bottom_{side}",
                   (0.0, y, -0.403, 0, 0, 0), (0.90, 0.018, 0.050), GRAPHITE)
        for end, x in (("front", 0.425), ("rear", -0.425)):
            add_visual(chassis, f"lunar_track_{end}_{side}",
                       (x, y, -0.264, 0, 0, 0), (0.050, 0.018, 0.328), GRAPHITE)
        for number, x in enumerate((-0.32, -0.19, -0.06, 0.07, 0.20, 0.33)):
            add_visual(chassis, f"lunar_track_cleat_top_{side}_{number}",
                       (x, y + 0.029 * y_sign, -0.085, 0, 0, 0),
                       (0.045, 0.012, 0.030), TITANIUM)
            add_visual(chassis, f"lunar_track_cleat_bottom_{side}_{number}",
                       (x, y + 0.029 * y_sign, -0.413, 0, 0, 0),
                       (0.045, 0.012, 0.030), TITANIUM)
        for wheel_number, x in enumerate((-0.40, 0.0, 0.40)):
            add_cylinder_visual(chassis, f"lunar_wheel_hub_{side}_{wheel_number}",
                                (x, 0.531 * y_sign, -0.260, 1.5708, 0, 0),
                                0.075, 0.026, TITANIUM)
            add_cylinder_visual(chassis, f"lunar_wheel_cap_{side}_{wheel_number}",
                                (x, 0.550 * y_sign, -0.260, 1.5708, 0, 0),
                                0.032, 0.030, BRASS)

    # Short, articulated-looking front manipulators echo the reference while
    # staying visual-only so they cannot alter rover clearance or sensing.
    for side, sign in (("left", 1.0), ("right", -1.0)):
        add_cylinder_visual(chassis, f"lunar_arm_shoulder_{side}",
                            (0.285, 0.355 * sign, 0.040, 1.5708, 0, 0),
                            0.070, 0.085, BRASS)
        add_visual(chassis, f"lunar_arm_upper_{side}",
                   (0.420, 0.415 * sign, 0.025, 0, 0, -0.20 * sign),
                   (0.30, 0.070, 0.065), TITANIUM)
        add_visual(chassis, f"lunar_arm_palm_{side}",
                   (0.565, 0.468 * sign, 0.015, 0, 0, -0.20 * sign),
                   (0.095, 0.105, 0.075), CERAMIC)
        for claw, offset in (("inner", -0.034), ("outer", 0.034)):
            add_visual(chassis, f"lunar_arm_claw_{side}_{claw}",
                       (0.625, (0.468 + offset) * sign, 0.010, 0, 0, 0),
                       (0.105, 0.025, 0.035), BRASS)

    # Guard against accidental changes to the working drive/sensor structure.
    new_signature = tuple(
        (tag, tuple(node.get("name", "") for node in model.findall(tag)))
        for tag in ("link", "joint", "plugin")
    )
    new_sensors = tuple((sensor.get("name"), sensor.get("type"))
                        for sensor in model.findall(".//sensor"))
    if new_signature != physical_signature or new_sensors != sensor_signature:
        raise RuntimeError("design variant changed baseline mechanics or sensors")

    MODEL_DIR.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="  ")
    tree.write(DESTINATION, encoding="utf-8", xml_declaration=True)
    (MODEL_DIR / "model.config").write_text(
        "<?xml version=\"1.0\"?>\n"
        "<model>\n"
        "  <name>LunaBot Lunar Design</name>\n"
        "  <version>1.0</version>\n"
        "  <sdf version=\"1.9\">model.sdf</sdf>\n"
        "  <author><name>LunaBot V4</name></author>\n"
        "  <description>WALL-E-inspired lunar rover with dusty ochre armor, tracked drive, articulated arms, and stereo eye cameras.</description>\n"
        "</model>\n",
        encoding="utf-8")
    print(f"created {DESTINATION}")
    print(f"preserved {len(physical_signature[0][1])} links, "
          f"{len(physical_signature[1][1])} joints, "
          f"{len(sensor_signature)} sensors, and all Gazebo plugins")


if __name__ == "__main__":
    main()
