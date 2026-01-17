from __future__ import annotations
from dataclasses import dataclass
from typing import List, Optional, Tuple
from wpimath.geometry._geometry import Pose2d

@dataclass(frozen=True)
class FieldVisionObject:
    oid: str
    typ: str
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float

@dataclass(frozen=True)
class RepulsorVisionObstacle:
    oid: str
    kind: str
    x: float
    y: float
    size_x: float
    size_y: float

@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    theta: float

@dataclass(frozen=False)
class WorldSnapshot:
    fieldvision: List[FieldVisionObject]
    repulsorvision: List[RepulsorVisionObstacle]
    pose: Optional[Pose2d]
    extrinsics: Tuple[float, float, float, float, float, float]
