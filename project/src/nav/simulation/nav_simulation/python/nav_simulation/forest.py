"""Pillar-only Gazebo adaptation of EGO-Planner's RandomMapGenerateCylinder.

The upstream sampling, minimum center spacing and endpoint exclusion scheme are
ported to Python. PCG64 replaces the upstream random_device-seeded C++ generator;
one height per solid cylinder replaces voxel-column heights. Layouts are frozen
JSON, never fed into an online planner. See third_party/ego_forest/SOURCE.json.
SPDX-License-Identifier: GPL-3.0-only
"""
import hashlib
from decimal import Decimal
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

MAP_ID = "ego_pillar_forest"
SLIM_MAP_ID = "ego_pillar_forest_slim"
FOREST_MAP_IDS = (MAP_ID, SLIM_MAP_ID)
GROUND_TOP = 0.20


def generate_layout(seed=20261022, count=120):
    import numpy as np
    rng = np.random.default_rng(seed)
    bounds = [0.0, 30.0, -1.0, 17.0]
    cylinders = []
    for attempt in range(100000):
        if len(cylinders) == count:
            break
        x, y = rng.uniform(bounds[0]+1, bounds[1]-1), rng.uniform(bounds[2]+1, bounds[3]-1)
        w, inflation = rng.uniform(.45, .75), rng.uniform(.8, 1.2)
        if any(math.hypot(x-px, y-py) < 2.0 for px, py in [(1.0, 8.0), (29.0, 8.0)]):
            continue
        if any(math.hypot(x-p["x"], y-p["y"]) < 1.15 for p in cylinders):
            continue
        x, y = math.floor(x/.1)*.1+.05, math.floor(y/.1)*.1+.05
        cylinders.append(dict(x=round(x, 6), y=round(y, 6),
                              radius=round(w*inflation/2, 6), height=round(float(rng.uniform(1.6, 2.2)), 6)))
    if len(cylinders) != count:
        raise ValueError("Could not place requested pillar count without changing spacing")
    return dict(schema_version=1, map_id=MAP_ID, seed=seed, bounds_xy=bounds,
                ground_top_m=GROUND_TOP, pillars=cylinders,
                upstream_repository="https://github.com/ZJU-FAST-Lab/ego-planner",
                upstream_commit="bfda51284c8c1b476043255a8145ef925a3778a5",
                upstream_function="RandomMapGenerateCylinder",
                adaptation="Pillar-only seeded PCG64 layout, solid cylinders, one height per pillar, no rings. Physical floor and boundary walls added.")


def generate_slim_layout(source, diameter_scale=.85):
    """Derive a separate map by scaling radii only; never sample new centers."""
    source = Path(source)
    original = source.read_bytes()
    layout = json.loads(original)
    if layout.get("map_id") != MAP_ID or not math.isfinite(diameter_scale) or not 0 < diameter_scale < 1:
        raise ValueError("Slim variant requires the original forest and a scale strictly between zero and one")
    scale = Decimal(str(diameter_scale))
    for pillar in layout["pillars"]:
        # Frozen parent radii have six decimals. Decimal multiplication avoids
        # introducing an extra geometry rounding or changing x/y/height.
        pillar["radius"] = float(Decimal(str(pillar["radius"])) * scale)
    layout["map_id"] = SLIM_MAP_ID
    layout["geometry_version"] = "slim-v1"
    layout["derivation"] = dict(
        parent_map_id=MAP_ID, parent_layout_filename=source.name,
        parent_layout_sha256=hashlib.sha256(original).hexdigest(),
        diameter_scale=diameter_scale, radius_scale=diameter_scale,
        resampled_centers=False,
        unchanged=["seed", "bounds_xy", "ground_top_m", "pillar count/order", "pillar x/y/height"],
        scope="Controlled smaller-diameter simulator scene; offline reference must be regenerated. No online map or traversability guarantee.")
    return layout


def load_layout(directory, map_id):
    if map_id not in FOREST_MAP_IDS:
        raise ValueError("Unknown pillar-forest map_id")
    layout = json.loads((Path(directory) / (map_id + ".json")).read_text())
    if layout.get("map_id") != map_id:
        raise ValueError("Pillar-forest layout identity does not match requested map_id")
    return layout


def add_solid(world, name, pose, kind, parameters, color):
    model = ET.SubElement(world, "model", name=name)
    ET.SubElement(model, "static").text = "true"
    ET.SubElement(model, "pose").text = " ".join(str(v) for v in pose)
    link = ET.SubElement(model, "link", name="body")
    for tag in ("collision", "visual"):
        target = ET.SubElement(link, tag, name=name+"_"+tag)
        shape = ET.SubElement(ET.SubElement(target, "geometry"), kind)
        for key, value in parameters.items():
            ET.SubElement(shape, key).text = str(value)
        if tag == "visual":
            material = ET.SubElement(target, "material")
            ET.SubElement(material, "ambient").text = color
            ET.SubElement(material, "diffuse").text = color
    return model


def populate(world, layout):
    """Only simulator calls this; planners never import or receive the layout."""
    for model in list(world.findall("model")):
        world.remove(model)
    xmin, xmax, ymin, ymax = layout["bounds_xy"]
    floor = layout["ground_top_m"]
    add_solid(world, "forest_ground", [(xmin+xmax)/2, (ymin+ymax)/2, floor-.1, 0, 0, 0],
              "box", dict(size=f"{xmax-xmin+2} {ymax-ymin+2} .2"), ".42 .45 .40 1")
    for i, pillar in enumerate(layout["pillars"]):
        height = pillar["height"]
        add_solid(world, f"pillar_{i:03d}", [pillar["x"], pillar["y"], floor+height/2, 0, 0, 0],
                  "cylinder", dict(radius=pillar["radius"], length=height), ".38 .25 .12 1")
    for name, x, y, sx, sy in [
        ("south", (xmin+xmax)/2, ymin-.1, xmax-xmin+.4, .2),
        ("north", (xmin+xmax)/2, ymax+.1, xmax-xmin+.4, .2),
        ("west", xmin-.1, (ymin+ymax)/2, .2, ymax-ymin),
        ("east", xmax+.1, (ymin+ymax)/2, .2, ymax-ymin),
    ]:
        add_solid(world, "boundary_"+name, [x, y, floor+.6, 0, 0, 0],
                  "box", dict(size=f"{sx} {sy} 1.2"), ".55 .57 .59 1")


def write_reference(layout_path, output, resolution=.05, robot_radius=.32):
    """Offline endpoint/metric reference only; no ROS map is generated here."""
    import numpy as np
    from scipy import ndimage
    layout_path = Path(layout_path)
    layout = json.loads(layout_path.read_text())
    xmin, xmax, ymin, ymax = layout["bounds_xy"]
    nx, ny = round((xmax-xmin)/resolution), round((ymax-ymin)/resolution)
    x = xmin+(np.arange(nx)+.5)*resolution
    y = ymin+(np.arange(ny)+.5)*resolution
    xx, yy = np.meshgrid(x, y)
    clearance = np.minimum.reduce([xx-xmin, xmax-xx, yy-ymin, ymax-yy])
    for p in layout["pillars"]:
        clearance = np.minimum(clearance, np.hypot(xx-p["x"], yy-p["y"])-p["radius"])
    free = clearance > 0
    safe = clearance >= robot_radius+math.sqrt(2)*resolution/2
    labels, _ = ndimage.label(safe)
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(output, free_mask=free, free_centers=safe,
        safe_centers_unrestricted=safe, component_labels=labels.astype(np.int32),
        clearance=clearance.astype(np.float32), origin_xy=np.array([xmin, ymin]),
        resolution=np.array(resolution), source_sha256=np.array(hashlib.sha256(layout_path.read_bytes()).hexdigest()),
        map_id=np.array(layout["map_id"]), reference_scope=np.array("offline endpoints and metrics only"))
    return output


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--layout", type=Path, required=True)
    parser.add_argument("--reference", type=Path)
    parser.add_argument("--seed", type=int, default=20261022)
    parser.add_argument("--pillars", type=int, default=120)
    parser.add_argument("--slim-from", type=Path, help="Derive a separate slim map from this frozen parent; do not resample centers")
    parser.add_argument("--diameter-scale", type=float, default=.85, help="Uniform radius/diameter multiplier for --slim-from")
    args = parser.parse_args()
    args.layout.parent.mkdir(parents=True, exist_ok=True)
    if args.layout.exists():
        raise FileExistsError("Frozen layout exists; choose a new filename")
    layout = (generate_slim_layout(args.slim_from, args.diameter_scale) if args.slim_from
              else generate_layout(args.seed, args.pillars))
    args.layout.write_text(json.dumps(layout, indent=2)+"\n")
    if args.reference:
        write_reference(args.layout, args.reference)


if __name__ == "__main__":
    main()
