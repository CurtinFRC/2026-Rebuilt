# repulsor_sim/publishers/repulsorvision.py
from __future__ import annotations
from typing import Sequence
from repulsor_sim.types import VisionObstacle

def publish_repulsorvision(table, obstacles: Sequence[VisionObstacle]):
    for o in obstacles:
        base = f"obs_{o.oid}"
        table.getEntry(base).setBoolean(True)
        table.getEntry(base + "/kind").setString(o.kind)
        table.getEntry(base + "/x").setDouble(float(o.x))
        table.getEntry(base + "/y").setDouble(float(o.y))
        table.getEntry(base + "/size_x").setDouble(float(o.sx))
        table.getEntry(base + "/size_y").setDouble(float(o.sy))
