import yaml
import math

from xml.etree.ElementTree import Element, SubElement, ElementTree
import xml.etree.ElementTree as ET


def add_wall(model, wall,wall_thickness,wall_height):
    name = wall["name"]
    x1, y1 = wall["start"]
    x2, y2 = wall["end"]

    dx = x2 - x1
    dy = y2 - y1
    length = (dx ** 2 + dy ** 2) ** 0.5

    cx = (x1 + x2) / 2.0
    cy = (y1 + y2) / 2.0
    cz = wall_height / 2.0

    yaw = math.atan2(dy, dx)

    link = SubElement(model, "link", {"name": name})
    pose = SubElement(link, "pose")
    pose.text = f"{cx} {cy} {cz} 0 0 {yaw}"

    collision = SubElement(link, "collision")
    collision.set("name", "collision")
    geometry = SubElement(collision, "geometry")
    box = SubElement(geometry, "box")
    size = SubElement(box, "size")
    size.text = f"{length} {wall_thickness} {wall_height}"

    visual = SubElement(link, "visual")
    visual.set("name", "visual")
    geometry = SubElement(visual, "geometry")
    box = SubElement(geometry, "box")
    size = SubElement(box, "size")
    size.text = f"{length} {wall_thickness} {wall_height}"

def add_floor(model, floor, floor_thickness):
    name = floor["name"]
    vertices = floor["vertices"]
    elevation = floor.get("elevation", 0.0)

    xs = [p[0] for p in vertices]
    ys = [p[1] for p in vertices]
    min_x = min(xs)
    max_x = max(xs)
    min_y = min(ys)
    max_y = max(ys)
    length = max_x - min_x
    width = max_y - min_y
    cx = (min_x + max_x) / 2.0
    cy = (min_y + max_y) / 2.0
    # elevation 表示地面的上表面
    cz = elevation - floor_thickness / 2.0

    link = SubElement(model, "link")
    link.set("name", name)
    pose = SubElement(link, "pose")
    pose.text = f"{cx} {cy} {cz} 0 0 0"

    collision = SubElement(link, "collision")
    collision.set("name", "collision")
    geometry = SubElement(collision, "geometry")
    box = SubElement(geometry, "box")
    size = SubElement(box, "size")
    size.text = f"{length} {width} {floor_thickness}"

    visual = SubElement(link, "visual")
    visual.set("name", "visual")
    geometry = SubElement(visual, "geometry")
    box = SubElement(geometry, "box")
    size = SubElement(box, "size")
    size.text = f"{length} {width} {floor_thickness}"

def add_info(model):

    # spherical_coordinates = SubElement(model, "spherical_coordinates")
    # surface_model = SubElement(spherical_coordinates, "surface_model")
    # surface_model.text = "EARTH_WGS84"
    # latitude_deg = SubElement(spherical_coordinates, "latitude_deg")
    # latitude_deg.text = "47.3667"
    # longitude_deg = SubElement(spherical_coordinates, "longitude_deg")
    # longitude_deg.text = "8.5500"
    # elevation = SubElement(spherical_coordinates, "elevation")
    # elevation.text = "500.0"
    # heading_deg = SubElement(spherical_coordinates, "heading_deg")
    # heading_deg.text = "0"


    # ============================================================
    # physics
    # ============================================================

    physics = SubElement(model, "physics")
    physics.set("type", "ode")

    ode = SubElement(physics, "ode")


    # ----------------------------
    # solver
    # ----------------------------

    solver = SubElement(ode, "solver")

    solver_type = SubElement(solver, "type")
    solver_type.text = "quick"

    iters = SubElement(solver, "iters")
    iters.text = "1000"

    sor = SubElement(solver, "sor")
    sor.text = "1.3"


    # ----------------------------
    # constraints
    # ----------------------------

    constraints = SubElement(ode, "constraints")

    cfm = SubElement(constraints, "cfm")
    cfm.text = "0"

    erp = SubElement(constraints, "erp")
    erp.text = "0.2"

    contact_max_correcting_vel = SubElement(
        constraints,
        "contact_max_correcting_vel"
    )
    contact_max_correcting_vel.text = "100"

    contact_surface_layer = SubElement(
        constraints,
        "contact_surface_layer"
    )
    contact_surface_layer.text = "0.001"


    # ----------------------------
    # General physics parameters
    # ----------------------------

    max_step_size = SubElement(physics, "max_step_size")
    max_step_size.text = "0.001"

    real_time_factor = SubElement(physics, "real_time_factor")
    real_time_factor.text = "1"

    real_time_update_rate = SubElement(
        physics,
        "real_time_update_rate"
    )
    real_time_update_rate.text = "1000"

    gravity = SubElement(physics, "gravity")
    gravity.text = "0 0 -9.8"

def build_world(config):
    sdf = Element("sdf", {"version": "1.6"})
    world = SubElement(sdf, "world", {"name": "default"})

    # sun
    include = SubElement(world, "include")
    uri = SubElement(include, "uri")
    uri.text = "model://sun"

    # building model
    model = SubElement(world, "model", {"name": "building_1f"})
    static = SubElement(model, "static")
    static.text = "true"
    
    wall_height = config["wall_defaults"]["height"]
    wall_thickness = config["wall_defaults"]["thickness"]
    floor_thickness = config["floor_defaults"]["thickness"]

    for wall in config["walls"]:
        add_wall(model,wall,wall_thickness,wall_height)

    for floor in config["floors"]:
        add_floor(model, floor, floor_thickness)

    add_info(model)

    # min_x, min_y, max_x, max_y = compute_ground_bbox(segments)
    # ground_cx = (min_x + max_x) / 2.0
    # ground_cy = (min_y + max_y) / 2.0
    # ground_size_x = max_x - min_x
    # ground_size_y = max_y - min_y

    # create_box_link(
    #     model,
    #     "ground",
    #     [ground_size_x, ground_size_y, GROUND_THICKNESS],
    #     [ground_cx, ground_cy, -GROUND_THICKNESS / 2.0, 0, 0, 0],
    # )

    # wall_id = 0
    # for x1, y1, x2, y2 in segments:
    #     result = line_to_wall_segment(x1, y1, x2, y2)
    #     if result is None:
    #         continue
    #     size, pose = result
    #     create_box_link(model, f"wall_{wall_id}", size, pose)
    #     wall_id += 1

    return sdf
def main():
    # doc = ezdxf.readfile(DXF_FILE)
    # segments = extract_wall_segments(doc)
    # segments = shift_segments_to_local_frame(segments)

    with open("test_room.yaml", "r") as f:
        config = yaml.safe_load(f)

    OUT_SDF = "building.world"

    world_xml = build_world(config)
    ET.indent(world_xml, space="  ")
    ElementTree(world_xml).write(OUT_SDF, encoding="utf-8", xml_declaration=True)

    print(f"Saved to {OUT_SDF}")


if __name__ == "__main__":
    main()