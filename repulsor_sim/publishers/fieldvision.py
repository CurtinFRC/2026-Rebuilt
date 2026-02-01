# repulsor_sim/publishers/fieldvision.py
from __future__ import annotations
from typing import Sequence
from repulsor_sim.types import CameraInfo, WorldObject, class_id_to_type

# keep track of the last actually published object states
last_published_objects: dict[str, WorldObject] = {}
last_published_cameras: dict[str, CameraInfo] = {}

def publish_fieldvision(
    table,
    objects: Sequence[WorldObject],
    cameras: Sequence[CameraInfo],
    extrinsics_xyzrpy=None,
    max_per_tick: int = 3,
):
    """
    Publish at most `max_per_tick` changed/new objects per call.
    `last_published_objects` tracks the last state that was actually written to `table`.
    """
    global last_published_objects, last_published_cameras

    # if extrinsics_xyzrpy is None:
    extrinsics_xyzrpy = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    ex, ey, ez, er, ep, eyaw = extrinsics_xyzrpy
    table.getEntry("extrinsics/x").setDouble(ex)
    table.getEntry("extrinsics/y").setDouble(ey)
    table.getEntry("extrinsics/z").setDouble(ez)
    table.getEntry("extrinsics/roll").setDouble(er)
    table.getEntry("extrinsics/pitch").setDouble(ep)
    table.getEntry("extrinsics/yaw").setDouble(eyaw)

    candidates: list[WorldObject] = []
    for o in objects:
        lo = last_published_objects.get(o.oid)
        if lo is None:
            candidates.append(o)
            continue
        if (
            o.class_id != lo.class_id
            or o.x != lo.x
            or o.y != lo.y
            or o.z != lo.z
            or o.roll != lo.roll
            or o.pitch != lo.pitch
            or o.yaw != lo.yaw
        ):
            candidates.append(o)
    
    # now if there is an object that was published last time but is not in the current objects, we need to
    for oid in list(last_published_objects.keys()):
        if not any(o.oid == oid for o in objects):
            base = f"object_{oid}"
            table.getEntry(base).setBoolean(False)
            del last_published_objects[oid]

    for o in candidates[:max_per_tick]:
        base = f"object_{o.oid}"
        table.getEntry(base).setBoolean(True)
        table.getEntry(base + "/frame").setString("field")
        table.getEntry(base + "/type").setString(class_id_to_type(o.class_id))
        table.getEntry(base + "/x").setDouble(float(o.x))
        table.getEntry(base + "/y").setDouble(float(o.y))
        table.getEntry(base + "/z").setDouble(float(o.z))
        table.getEntry(base + "/roll").setDouble(float(o.roll))
        table.getEntry(base + "/pitch").setDouble(float(o.pitch))
        table.getEntry(base + "/yaw").setDouble(float(o.yaw))
        last_published_objects[o.oid] = o

    # cameras (robot-relative)
    cam_candidates: list[CameraInfo] = []
    for c in cameras:
        lc = last_published_cameras.get(c.name)
        if lc is None:
            cam_candidates.append(c)
            continue
        if (
            c.x != lc.x
            or c.y != lc.y
            or c.z != lc.z
            or c.yaw_deg != lc.yaw_deg
            or c.pitch_deg != lc.pitch_deg
            or c.roll_deg != lc.roll_deg
            or c.hfov_deg != lc.hfov_deg
            or c.vfov_deg != lc.vfov_deg
            or c.max_range != lc.max_range
        ):
            cam_candidates.append(c)

    for name in list(last_published_cameras.keys()):
        if not any(c.name == name for c in cameras):
            base = f"camera_{name}"
            table.getEntry(base).setBoolean(False)
            del last_published_cameras[name]

    for c in cam_candidates:
        base = f"camera_{c.name}"
        table.getEntry(base).setBoolean(True)
        table.getEntry(base + "/x").setDouble(float(c.x))
        table.getEntry(base + "/y").setDouble(float(c.y))
        table.getEntry(base + "/z").setDouble(float(c.z))
        table.getEntry(base + "/yaw_deg").setDouble(float(c.yaw_deg))
        table.getEntry(base + "/pitch_deg").setDouble(float(c.pitch_deg))
        table.getEntry(base + "/roll_deg").setDouble(float(c.roll_deg))
        table.getEntry(base + "/hfov_deg").setDouble(float(c.hfov_deg))
        table.getEntry(base + "/vfov_deg").setDouble(float(c.vfov_deg))
        table.getEntry(base + "/max_range").setDouble(float(c.max_range))
        last_published_cameras[c.name] = c
