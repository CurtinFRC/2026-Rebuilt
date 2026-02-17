from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class ClassDefinition:
    type_name: str
    repulsor_class_id: int
    oid_prefix: str
    estimator: str
    plane_z_m: float
    ball_radius_m: float
    frac_from_top: float
    min_abs_dz: float
    max_range_m: float | None
    roll: float
    pitch: float
    yaw: float
    enabled: bool


class ClassMap:
    def __init__(self, classes: dict[int, ClassDefinition], default: ClassDefinition):
        self._classes = dict(classes)
        self._default = default

    @staticmethod
    def _parse_entry(raw: dict, fallback_class_id: int) -> ClassDefinition:
        type_name = str(raw.get("type", "unknown"))
        repulsor_class_id = int(raw.get("repulsor_class_id", fallback_class_id))
        oid_prefix = str(raw.get("oid_prefix", type_name if type_name else "obj"))
        estimator = str(raw.get("estimator", "plane"))
        plane_z_m = float(raw.get("plane_z_m", 0.0))
        ball_radius_m = float(raw.get("ball_radius_m", 0.15))
        frac_from_top = float(raw.get("frac_from_top", 0.0))
        min_abs_dz = float(raw.get("min_abs_dz", 0.0))
        max_range_raw = raw.get("max_range_m", None)
        max_range_m = float(max_range_raw) if max_range_raw is not None else None
        roll = float(raw.get("roll", 0.0))
        pitch = float(raw.get("pitch", 0.0))
        yaw = float(raw.get("yaw", 0.0))
        enabled = bool(raw.get("enabled", True))
        return ClassDefinition(
            type_name=type_name,
            repulsor_class_id=repulsor_class_id,
            oid_prefix=oid_prefix,
            estimator=estimator,
            plane_z_m=plane_z_m,
            ball_radius_m=ball_radius_m,
            frac_from_top=frac_from_top,
            min_abs_dz=min_abs_dz,
            max_range_m=max_range_m,
            roll=roll,
            pitch=pitch,
            yaw=yaw,
            enabled=enabled,
        )

    @classmethod
    def from_file(cls, path: str | Path) -> ClassMap:
        p = Path(path).resolve()
        raw = json.loads(p.read_text(encoding="utf-8"))
        if not isinstance(raw, dict):
            raise ValueError(f"class map at {p} must be an object")
        default_raw = raw.get("default", {})
        classes_raw = raw.get("classes", {})
        if not isinstance(default_raw, dict):
            raise ValueError(f"default class mapping at {p} must be an object")
        if not isinstance(classes_raw, dict):
            raise ValueError(f"classes mapping at {p} must be an object")
        default = cls._parse_entry(default_raw, fallback_class_id=-1)
        parsed: dict[int, ClassDefinition] = {}
        for key, entry in classes_raw.items():
            if not isinstance(entry, dict):
                raise ValueError(f"class entry {key} at {p} must be an object")
            cid = int(key)
            parsed[cid] = cls._parse_entry(entry, fallback_class_id=cid)
        return cls(parsed, default)

    def get(self, class_id: int) -> ClassDefinition:
        return self._classes.get(int(class_id), self._default)

    def prefix_for(self, class_id: int) -> str:
        return self.get(class_id).oid_prefix
