"""Versioned B-only correction after URDF conversion, preserving other bytes.

Restore coaxial cylinders after the converter's reduced-precision output.
No friction-frame variant, motor change or runtime profile switch is provided.
"""
import hashlib
import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

CONTACT_MODEL_VERSION = "omni-sentry-coaxial-v1"
WHEELS = ("front_left", "front_right", "rear_left", "rear_right")


def digest(data):
    return hashlib.sha256(data).hexdigest()


def _vector(text, count):
    values = [float(v) for v in text.split()]
    if len(values) != count or not all(math.isfinite(v) for v in values):
        raise ValueError("Invalid finite SDF vector")
    return values


def _block(data, tag, name, bounds=None):
    lo, hi = bounds or (0, len(data))
    pattern = rb"<" + tag.encode() + rb"\s+name=([\"'])" + re.escape(name.encode()) + rb"\1[^>]*>"
    hits = list(re.finditer(pattern, data[lo:hi]))
    if len(hits) != 1:
        raise ValueError("Non-unique SDF element: " + name)
    start = lo + hits[0].start()
    end = data.find(b"</" + tag.encode() + b">", start, hi)
    if end < 0:
        raise ValueError("Unclosed SDF element")
    return start, end + len(tag) + 3


def _patch(data, bounds, tag, values, field):
    lo, hi = bounds
    hits = list(re.finditer(rb"(<" + tag.encode() + rb"\b[^>]*>)([^<]*)(</" + tag.encode() + rb">)", data[lo:hi]))
    if len(hits) != 1:
        raise ValueError("Non-unique SDF field: " + field)
    hit = hits[0]
    return dict(field=field, start=lo+hit.start(2), end=lo+hit.end(2),
                old=hit.group(2).decode(), new=" ".join(values))


def _coaxial(axis, rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    cylinder = [cy*sp*cr+sy*sr, sy*sp*cr-cy*sr, cp*cr]
    length = math.sqrt(sum(v*v for v in axis))
    a = [v/length for v in axis]
    cross = [a[1]*cylinder[2]-a[2]*cylinder[1], a[2]*cylinder[0]-a[0]*cylinder[2], a[0]*cylinder[1]-a[1]*cylinder[0]]
    error = math.sqrt(sum(v*v for v in cross))
    if error > 5e-15 or abs(cylinder[2]) > 5e-15:
        raise ValueError("Corrected cylinder is not coaxial with its horizontal joint")
    return dict(axis_unit=a, cylinder_axis_joint=cylinder,
                signed_dot=sum(x*y for x, y in zip(a, cylinder)), cross_norm=error)


def correct_sdf(data):
    """Return final bytes and a certificate for four axes/four collision RPYs."""
    if b"<!DOCTYPE" in data or b"<!ENTITY" in data:
        raise ValueError("Entity declarations are unsupported")
    models = ET.fromstring(data).findall(".//model[@name='sentry']")
    if len(models) != 1 or models[0].findall(".//model"):
        raise ValueError("One flat sentry model is required")
    model = models[0]
    if model.find("link[@name='chassis']") is None:
        raise ValueError("Chassis is missing")
    mb = _block(data, "model", "sentry")
    patches, certificates = [], []
    for wheel in WHEELS:
        name, joint_name = wheel+"_wheel", wheel+"_wheel_joint"
        joint = model.find("joint[@name='"+joint_name+"']")
        if joint is None or joint.findtext("parent") != "chassis" or joint.findtext("child") != name:
            raise ValueError("Unexpected wheel joint topology")
        jp = joint.find("pose")
        if jp is None or jp.get("relative_to") != "chassis" or _vector(jp.text, 6)[3:] != [0., 0., 0.]:
            raise ValueError("Wheel joint must use an unrotated chassis frame")
        axis_node = joint.find("axis/xyz")
        if axis_node is None or axis_node.attrib:
            raise ValueError("Wheel axis must use its local joint frame")
        axis = _vector(axis_node.text, 3)
        sx = 1 if wheel.startswith("front") else -1
        sy = 1 if wheel.endswith("left") else -1
        norm = math.sqrt(sum(v*v for v in axis))
        if norm == 0 or any(abs(v/norm-e) > 1e-13 for v, e in zip(axis, [sx*math.sqrt(.5), sy*math.sqrt(.5), 0.])):
            raise ValueError("Unexpected signed diagonal wheel axis")
        link = model.find("link[@name='"+name+"']")
        lp = link.find("pose") if link is not None else None
        if lp is None or lp.get("relative_to") != joint_name or any(_vector(lp.text, 6)):
            raise ValueError("Wheel link must coincide with its joint frame")
        if len(link.findall("collision")) != 1:
            raise ValueError("One wheel cylinder collision is required")
        collision = link.find("collision")
        cp = collision.find("pose")
        if collision.find("geometry/cylinder") is None or cp is None or cp.attrib:
            raise ValueError("Wheel collision must be a local cylinder")
        pose = _vector(cp.text, 6)
        yaw_sign = -1 if wheel in ("front_left", "rear_right") else 1
        target_rpy = [math.pi/2, -math.pi/4, yaw_sign*math.pi/4]
        rounded_rpy = [1.5708, -.7854, yaw_sign*.7854]
        if not any(all(abs(x-y) < 1e-14 for x, y in zip(pose[3:], expected)) for expected in (rounded_rpy, target_rpy)):
            raise ValueError("Unexpected source cylinder orientation")
        target_axis = [sx*math.sqrt(.5), sy*math.sqrt(.5), 0.]
        lb = _block(data, "link", name, mb)
        cb = _block(data, "collision", collision.get("name"), lb)
        patches.append(_patch(data, _block(data, "joint", joint_name, mb), "xyz", [format(x, ".17g") for x in target_axis], joint_name+"/axis/xyz"))
        patches.append(_patch(data, cb, "pose", cp.text.split()[:3]+[format(x, ".17g") for x in target_rpy], name+"/collision/pose"))
    patches = sorted((p for p in patches if p["old"] != p["new"]), key=lambda p: p["start"])
    for a, b in zip(patches, patches[1:]):
        if a["end"] > b["start"]:
            raise ValueError("Overlapping correction fields")
    final = data
    for patch in reversed(patches):
        if final[patch["start"]:patch["end"]].decode() != patch["old"]:
            raise ValueError("Correction input bytes changed")
        final = final[:patch["start"]]+patch["new"].encode()+final[patch["end"]:]
    # Certify the actual formatted/parsed final SDF, not only target constants.
    parsed = ET.fromstring(final).find(".//model[@name='sentry']")
    for wheel in WHEELS:
        name = wheel+"_wheel"
        a = _vector(parsed.findtext("joint[@name='"+name+"_joint']/axis/xyz"), 3)
        rpy = _vector(parsed.findtext("link[@name='"+name+"']/collision/pose"), 6)[3:]
        certificates.append(dict(wheel=name, **_coaxial(a, rpy)))
    return final, dict(schema_version=1, model_version=CONTACT_MODEL_VERSION,
        intervention="B_exact_axes_collision_rpy", input_sdf_sha256=digest(data), final_sdf_sha256=digest(final),
        generator_module_sha256=digest(Path(__file__).read_bytes()),
        patches=patches, coaxial_certificates=certificates, arithmetic="double-precision geometry check, not interval arithmetic",
        all_bytes_outside_listed_fields_preserved=True, friction_frame_changed=False,
        source_urdf_role="Nominal upstream conversion input; final physical SDF has the explicitly listed model correction")


def finalize_world(path):
    """Apply after the final XML serialization and save its version proof."""
    path = Path(path)
    final, certificate = correct_sdf(path.read_bytes())
    path.write_bytes(final)
    certificate["final_sdf_file"] = path.name
    path.with_suffix(".contact-model.json").write_text(json.dumps(certificate, indent=2)+"\n")
    return certificate
