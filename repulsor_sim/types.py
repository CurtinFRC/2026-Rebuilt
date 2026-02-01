# repulsor_sim/types.py
from __future__ import annotations
from dataclasses import dataclass
from typing import Optional, Sequence

CLASS_FUEL = 0
CLASS_ROBOT_BLUE = 1
CLASS_ROBOT_RED = 2

def class_id_to_type(class_id: int) -> str:
    if class_id == CLASS_FUEL:
        return "fuel"
    if class_id == CLASS_ROBOT_BLUE:
        return "robot_blue"
    if class_id == CLASS_ROBOT_RED:
        return "robot_red"
    return "unknown"

@dataclass(frozen=True)
class WorldObject:
    oid: str
    class_id: int
    x: float
    y: float
    z: float = 0.10
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

@dataclass(frozen=True)
class VisionObstacle:
    oid: str
    kind: str
    x: float
    y: float
    sx: float
    sy: float

@dataclass(frozen=True)
class CameraInfo:
    name: str
    x: float
    y: float
    z: float
    yaw_deg: float
    pitch_deg: float
    roll_deg: float
    hfov_deg: float
    vfov_deg: float
    max_range: float

@dataclass(frozen=True)
class ProviderFrame:
    objects: Sequence[WorldObject]
    obstacles: Sequence[VisionObstacle]
    cameras: Sequence[CameraInfo] = ()
    extrinsics_xyzrpy: Optional[tuple[float, float, float, float, float, float]] = None
