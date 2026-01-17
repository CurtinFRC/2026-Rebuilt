# repulsor_sim/publishers/fieldvision.py
from __future__ import annotations
from typing import Sequence
from repulsor_sim.types import WorldObject, class_id_to_type

# keep track of the last actually published object states
last_published_objects: dict[str, WorldObject] = {}

def publish_fieldvision(table, objects: Sequence[WorldObject], extrinsics_xyzrpy=None, max_per_tick: int = 3):
    """
    Publish at most `max_per_tick` changed/new objects per call.
    `last_published_objects` tracks the last state that was actually written to `table`.
    """
    global last_published_objects

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
