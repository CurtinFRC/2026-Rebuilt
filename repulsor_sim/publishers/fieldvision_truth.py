# repulsor_sim/publishers/fieldvision_truth.py
from __future__ import annotations
from typing import Sequence
from repulsor_sim.types import WorldObject, class_id_to_type

# track the last published ground-truth object states
last_published_truth: dict[str, WorldObject] = {}

def publish_fieldvision_truth(
    table,
    objects: Sequence[WorldObject],
    max_per_tick: int = 12,
):
    """
    Publish at most `max_per_tick` changed/new ground-truth objects per call.
    `last_published_truth` tracks the last state that was actually written to `table`.
    """
    global last_published_truth

    candidates: list[WorldObject] = []
    for o in objects:
        lo = last_published_truth.get(o.oid)
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

    current_ids = {o.oid for o in objects}
    for oid in list(last_published_truth.keys()):
        if oid not in current_ids:
            base = f"object_{oid}"
            table.getEntry(base).setBoolean(False)
            del last_published_truth[oid]

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
        last_published_truth[o.oid] = o
