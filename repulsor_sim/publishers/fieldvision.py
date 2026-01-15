# repulsor_sim/publishers/fieldvision.py
from __future__ import annotations
from typing import Sequence
from repulsor_sim.types import WorldObject, class_id_to_type

def publish_fieldvision(table, objects: Sequence[WorldObject], extrinsics_xyzrpy=None):
    if extrinsics_xyzrpy is None:
        extrinsics_xyzrpy = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    ex, ey, ez, er, ep, eyaw = extrinsics_xyzrpy
    table.getEntry("extrinsics/x").setDouble(ex)
    table.getEntry("extrinsics/y").setDouble(ey)
    table.getEntry("extrinsics/z").setDouble(ez)
    table.getEntry("extrinsics/roll").setDouble(er)
    table.getEntry("extrinsics/pitch").setDouble(ep)
    table.getEntry("extrinsics/yaw").setDouble(eyaw)

    for o in objects:
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
