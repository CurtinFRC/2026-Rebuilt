from __future__ import annotations

import json
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

import numpy as np


@dataclass(frozen=True)
class NTSettings:
    server: str
    port: int
    client_name: str
    table_path: str
    pose_base_path: str
    pose_struct_key: str
    connect_timeout_s: float


@dataclass(frozen=True)
class PoseConfig:
    x: float
    y: float
    z: float
    yaw_deg: float
    pitch_deg: float
    roll_deg: float


@dataclass(frozen=True)
class CameraRuntimeConfig:
    name: str
    usb_index: int
    width: int
    height: int
    fps: float
    hfov_deg: float
    vfov_deg: float
    max_range_m: float
    pose_mode: str
    robot_pose: PoseConfig
    field_pose: PoseConfig
    K: np.ndarray
    dist: np.ndarray
    axis_cam_from_cv: np.ndarray | None
    pipeline: dict[str, Any]


@dataclass(frozen=True)
class RuntimeConfig:
    nt: NTSettings
    loop_hz: float
    max_objects_per_tick: int
    class_map_path: str
    cameras: list[CameraRuntimeConfig]


def _as_pose(raw: Any, defaults: PoseConfig) -> PoseConfig:
    if not isinstance(raw, dict):
        return defaults
    return PoseConfig(
        x=float(raw.get("x", defaults.x)),
        y=float(raw.get("y", defaults.y)),
        z=float(raw.get("z", defaults.z)),
        yaw_deg=float(raw.get("yaw_deg", defaults.yaw_deg)),
        pitch_deg=float(raw.get("pitch_deg", defaults.pitch_deg)),
        roll_deg=float(raw.get("roll_deg", defaults.roll_deg)),
    )


def _resolve_path(config_dir: Path, p: str) -> str:
    path = Path(p)
    if path.is_absolute():
        return str(path)
    return str((config_dir / path).resolve())


def _resolve_pipeline_paths(config_dir: Path, spec: Any) -> Any:
    if isinstance(spec, dict):
        out: dict[str, Any] = {}
        for k, v in spec.items():
            if isinstance(v, str) and k.endswith("_path"):
                out[k] = _resolve_path(config_dir, v)
            else:
                out[k] = _resolve_pipeline_paths(config_dir, v)
        return out
    if isinstance(spec, list):
        return [_resolve_pipeline_paths(config_dir, x) for x in spec]
    return spec


def _default_K(width: int, height: int, hfov_deg: float, vfov_deg: float) -> np.ndarray:
    w = float(max(1, width))
    h = float(max(1, height))
    hf = float(max(1.0, min(179.0, hfov_deg)))
    vf = float(max(1.0, min(179.0, vfov_deg)))
    fx = w * 0.5 / np.tan(np.deg2rad(hf) * 0.5)
    fy = h * 0.5 / np.tan(np.deg2rad(vf) * 0.5)
    cx = w * 0.5
    cy = h * 0.5
    return np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)


def _read_axis(raw: Any) -> np.ndarray | None:
    if raw is None:
        return None
    arr = np.asarray(raw, dtype=np.float64).reshape(3, 3)
    return arr


def _read_calibration(raw: Any, width: int, height: int, hfov_deg: float, vfov_deg: float) -> tuple[np.ndarray, np.ndarray]:
    if not isinstance(raw, dict):
        return _default_K(width, height, hfov_deg, vfov_deg), np.zeros((5,), dtype=np.float64)
    K_raw = raw.get("K")
    dist_raw = raw.get("dist")
    K = np.asarray(K_raw, dtype=np.float64).reshape(3, 3) if K_raw is not None else _default_K(width, height, hfov_deg, vfov_deg)
    dist = np.asarray(dist_raw, dtype=np.float64).reshape(-1) if dist_raw is not None else np.zeros((5,), dtype=np.float64)
    return K, dist


def _default_runtime_config_path() -> str:
    env_path = os.getenv("VISION_CONFIG")
    if env_path:
        return env_path
    for candidate in ("vision/config/runtime.default.json", "vision/config/runtime.json"):
        if Path(candidate).exists():
            return candidate
    return "vision/config/runtime.default.json"


def resolve_runtime_config_path(path: str | None = None) -> Path:
    return Path(path or _default_runtime_config_path()).resolve()


def load_runtime_config_raw(path: str | None = None) -> dict[str, Any]:
    p = resolve_runtime_config_path(path)
    raw = json.loads(p.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ValueError(f"runtime config at {p} must be an object")
    return raw


def save_runtime_camera_usb_indexes(
    usb_index_by_camera_name: Mapping[str, int],
    path: str | None = None,
) -> Path:
    p = resolve_runtime_config_path(path)
    raw = load_runtime_config_raw(str(p))
    cameras_raw = raw.get("cameras", [])
    if not isinstance(cameras_raw, list) or not cameras_raw:
        raise ValueError("cameras must be a non-empty list")

    known_names: set[str] = set()
    for c in cameras_raw:
        if not isinstance(c, dict):
            raise ValueError("camera entry must be an object")
        name = str(c.get("name", "")).strip()
        if not name:
            raise ValueError("camera name is required")
        known_names.add(name)
        if name in usb_index_by_camera_name:
            c["usb_index"] = int(usb_index_by_camera_name[name])

    unknown = sorted(set(usb_index_by_camera_name.keys()) - known_names)
    if unknown:
        raise ValueError(f"unknown camera names in mapping: {', '.join(unknown)}")

    p.write_text(json.dumps(raw, indent=2) + "\n", encoding="utf-8")
    return p


def load_runtime_config(path: str | None = None) -> RuntimeConfig:
    p = resolve_runtime_config_path(path)
    raw = load_runtime_config_raw(str(p))
    config_dir = p.parent

    nt_raw = raw.get("nt", {})
    if not isinstance(nt_raw, dict):
        raise ValueError("nt config must be an object")
    nt = NTSettings(
        server=str(nt_raw.get("server", "localhost")),
        port=int(nt_raw.get("port", 5810)),
        client_name=str(nt_raw.get("client_name", "vision")),
        table_path=str(nt_raw.get("table_path", "FieldVision/main")),
        pose_base_path=str(nt_raw.get("pose_base_path", "AdvantageKit/RealOutputs/Odometry")),
        pose_struct_key=str(nt_raw.get("pose_struct_key", "Robot")),
        connect_timeout_s=float(nt_raw.get("connect_timeout_s", 2.0)),
    )

    loop_hz = float(raw.get("loop_hz", 20.0))
    max_objects_per_tick = int(raw.get("max_objects_per_tick", 256))
    class_map_path = _resolve_path(config_dir, str(raw.get("class_map_path", "yolo_classes.json")))

    cameras_raw = raw.get("cameras", [])
    if not isinstance(cameras_raw, list) or not cameras_raw:
        raise ValueError("cameras must be a non-empty list")
    cams: list[CameraRuntimeConfig] = []
    for c in cameras_raw:
        if not isinstance(c, dict):
            raise ValueError("camera entry must be an object")
        name = str(c.get("name", "")).strip()
        if not name:
            raise ValueError("camera name is required")
        usb_index = int(c.get("usb_index", 0))
        width = int(c.get("width", 1280))
        height = int(c.get("height", 720))
        fps = float(c.get("fps", 20.0))
        hfov_deg = float(c.get("hfov_deg", 90.0))
        vfov_deg = float(c.get("vfov_deg", 60.0))
        max_range_m = float(c.get("max_range_m", 10.0))
        pose_mode = str(c.get("pose_mode", "robot")).strip().lower()
        if pose_mode not in ("robot", "field"):
            raise ValueError(f"camera {name} pose_mode must be robot or field")
        robot_defaults = PoseConfig(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        field_defaults = PoseConfig(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        robot_pose = _as_pose(c.get("robot_pose", {}), robot_defaults)
        field_pose = _as_pose(c.get("field_pose", {}), field_defaults)
        K, dist = _read_calibration(c.get("calibration", {}), width, height, hfov_deg, vfov_deg)
        axis_raw = c.get(
            "axis_cam_from_cv",
            [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]],
        )
        axis = _read_axis(axis_raw)
        pipeline_raw = c.get("pipeline", {"type": "yolo"})
        if not isinstance(pipeline_raw, dict):
            raise ValueError(f"camera {name} pipeline must be an object")
        pipeline = _resolve_pipeline_paths(config_dir, pipeline_raw)
        cams.append(
            CameraRuntimeConfig(
                name=name,
                usb_index=usb_index,
                width=width,
                height=height,
                fps=fps,
                hfov_deg=hfov_deg,
                vfov_deg=vfov_deg,
                max_range_m=max_range_m,
                pose_mode=pose_mode,
                robot_pose=robot_pose,
                field_pose=field_pose,
                K=K,
                dist=dist,
                axis_cam_from_cv=axis,
                pipeline=pipeline,
            )
        )

    return RuntimeConfig(
        nt=nt,
        loop_hz=loop_hz,
        max_objects_per_tick=max_objects_per_tick,
        class_map_path=class_map_path,
        cameras=cams,
    )
