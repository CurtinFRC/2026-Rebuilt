from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from lib.fuel_estimator import YoloBBox


@dataclass(frozen=True)
class Detection2D:
    class_id: int
    confidence: float
    x1: float
    y1: float
    x2: float
    y2: float
    pipeline: str = ""
    metadata: dict[str, Any] = field(default_factory=dict)

    def as_bbox(self) -> YoloBBox:
        return YoloBBox(
            x1=float(self.x1),
            y1=float(self.y1),
            x2=float(self.x2),
            y2=float(self.y2),
        )


@dataclass(frozen=True)
class TrackedDetection:
    oid: str
    detection: Detection2D


@dataclass(frozen=True)
class FieldObject:
    oid: str
    type_name: str
    x: float
    y: float
    z: float
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    frame: str = "field"


@dataclass(frozen=True)
class CameraInfoOut:
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
