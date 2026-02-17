from __future__ import annotations

import argparse
import time
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np
from wpimath.geometry import Pose2d

from lib.class_map import ClassMap, ClassDefinition
from lib.config import CameraRuntimeConfig, RuntimeConfig, load_runtime_config
from lib.fuel_estimator import (
    CameraCalibration,
    CameraPoseField,
    R_field_from_cam_ypr,
    apply_axis_conversion,
    bbox_top_center_pixel,
    estimate_fuel_center_field,
    intersect_ray_with_plane_z,
    pixel_to_unit_ray_camera_opencv,
    ray_camera_to_field,
)
from lib.nt_client import VisionNTClient
from lib.pipelines import VisionPipeline, build_pipeline
from lib.publisher import FieldVisionPublisher
from lib.runtime_types import CameraInfoOut, FieldObject, TrackedDetection
from lib.tracker import DetectionTracker


@dataclass
class _CameraRuntime:
    cfg: CameraRuntimeConfig
    cap: cv2.VideoCapture
    pipeline: VisionPipeline
    calib: CameraCalibration


def _open_capture(cfg: CameraRuntimeConfig) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(int(cfg.usb_index), cv2.CAP_DSHOW)
    if not cap.isOpened():
        cap = cv2.VideoCapture(int(cfg.usb_index))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(cfg.width))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(cfg.height))
    cap.set(cv2.CAP_PROP_FPS, float(cfg.fps))
    return cap


def _camera_pose_field(cfg: CameraRuntimeConfig, robot_pose: Optional[Pose2d]) -> CameraPoseField:
    if cfg.pose_mode == "field":
        p = cfg.field_pose
        R_f_c = R_field_from_cam_ypr(
            yaw_deg=float(p.yaw_deg),
            pitch_deg=float(p.pitch_deg),
            roll_deg=float(p.roll_deg),
        )
        p_f = np.array([float(p.x), float(p.y), float(p.z)], dtype=np.float64)
        return CameraPoseField(R_f_c=R_f_c, p_f=p_f)
    if robot_pose is None:
        robot_x = 0.0
        robot_y = 0.0
        robot_yaw_deg = 0.0
    else:
        robot_x = float(robot_pose.x)
        robot_y = float(robot_pose.y)
        robot_yaw_deg = float(robot_pose.rotation().degrees())
    m = cfg.robot_pose
    R_f_r = R_field_from_cam_ypr(yaw_deg=robot_yaw_deg, pitch_deg=0.0, roll_deg=0.0)
    R_r_c = R_field_from_cam_ypr(
        yaw_deg=float(m.yaw_deg),
        pitch_deg=float(m.pitch_deg),
        roll_deg=float(m.roll_deg),
    )
    R_f_c = R_f_r @ R_r_c
    offset_r = np.array([float(m.x), float(m.y), float(m.z)], dtype=np.float64)
    p_f = np.array([robot_x, robot_y, 0.0], dtype=np.float64) + (R_f_r @ offset_r)
    return CameraPoseField(R_f_c=R_f_c, p_f=p_f)


def _estimate_on_plane(
    tracked: TrackedDetection,
    calib: CameraCalibration,
    pose: CameraPoseField,
    axis_cam_from_cv: np.ndarray | None,
    z_plane_m: float,
    frac_from_top: float,
    min_abs_dz: float,
    max_range_m: float | None,
) -> np.ndarray | None:
    bbox = tracked.detection.as_bbox()
    uv = bbox_top_center_pixel(bbox, frac_from_top=frac_from_top)
    ray_cv = pixel_to_unit_ray_camera_opencv(calib, uv)
    ray_c = apply_axis_conversion(ray_cv, axis_cam_from_cv)
    o_f, d_f = ray_camera_to_field(pose, ray_c)
    if float(min_abs_dz) > 0.0 and abs(float(d_f[2])) < float(min_abs_dz):
        return None
    pt = intersect_ray_with_plane_z(o_f, d_f, float(z_plane_m))
    if pt is None:
        return None
    if max_range_m is not None and float(np.linalg.norm(pt - o_f)) > float(max_range_m):
        return None
    return pt


def _camera_info(cfg: CameraRuntimeConfig) -> CameraInfoOut:
    if cfg.pose_mode == "robot":
        p = cfg.robot_pose
    else:
        p = cfg.field_pose
    return CameraInfoOut(
        name=cfg.name,
        x=float(p.x),
        y=float(p.y),
        z=float(p.z),
        yaw_deg=float(p.yaw_deg),
        pitch_deg=float(p.pitch_deg),
        roll_deg=float(p.roll_deg),
        hfov_deg=float(cfg.hfov_deg),
        vfov_deg=float(cfg.vfov_deg),
        max_range=float(cfg.max_range_m),
    )


def _to_field_object(
    tracked: TrackedDetection,
    cls: ClassDefinition,
    cam: _CameraRuntime,
    pose: CameraPoseField,
) -> FieldObject | None:
    if not cls.enabled:
        return None
    max_range = cls.max_range_m if cls.max_range_m is not None else float(cam.cfg.max_range_m)
    estimator = cls.estimator.strip().lower()
    if estimator == "fuel":
        pt = estimate_fuel_center_field(
            bbox=tracked.detection.as_bbox(),
            calib=cam.calib,
            pose=pose,
            ball_radius_m=float(cls.ball_radius_m),
            A_cam_from_cv=cam.cfg.axis_cam_from_cv,
            frac_from_top=float(cls.frac_from_top),
            min_abs_dz=float(cls.min_abs_dz),
            max_range_m=max_range,
        )
    elif estimator == "plane":
        pt = _estimate_on_plane(
            tracked=tracked,
            calib=cam.calib,
            pose=pose,
            axis_cam_from_cv=cam.cfg.axis_cam_from_cv,
            z_plane_m=float(cls.plane_z_m),
            frac_from_top=float(cls.frac_from_top),
            min_abs_dz=float(cls.min_abs_dz),
            max_range_m=max_range,
        )
    else:
        return None
    if pt is None:
        return None
    return FieldObject(
        oid=tracked.oid,
        type_name=cls.type_name,
        x=float(pt[0]),
        y=float(pt[1]),
        z=float(pt[2]),
        roll=float(cls.roll),
        pitch=float(cls.pitch),
        yaw=float(cls.yaw),
        frame="field",
    )


def _read_frame(cam: _CameraRuntime) -> np.ndarray | None:
    ok, frame = cam.cap.read()
    if ok and frame is not None and frame.size > 0:
        return frame
    cam.cap.release()
    cam.cap = _open_capture(cam.cfg)
    ok2, frame2 = cam.cap.read()
    if ok2 and frame2 is not None and frame2.size > 0:
        return frame2
    return None


def _build_cameras(cfg: RuntimeConfig) -> list[_CameraRuntime]:
    out: list[_CameraRuntime] = []
    for c in cfg.cameras:
        cap = _open_capture(c)
        if not cap.isOpened():
            raise RuntimeError(f"failed to open camera {c.name} at usb index {c.usb_index}")
        pipe = build_pipeline(c.pipeline)
        calib = CameraCalibration(K=np.asarray(c.K, dtype=np.float64), dist=np.asarray(c.dist, dtype=np.float64))
        out.append(_CameraRuntime(cfg=c, cap=cap, pipeline=pipe, calib=calib))
    return out


def run(config_path: str | None = None) -> None:
    cfg = load_runtime_config(config_path)
    class_map = ClassMap.from_file(cfg.class_map_path)
    nt = VisionNTClient(
        server=cfg.nt.server,
        port=cfg.nt.port,
        client_name=cfg.nt.client_name,
        pose_base_path=cfg.nt.pose_base_path,
        pose_struct_key=cfg.nt.pose_struct_key,
        connect_timeout_s=cfg.nt.connect_timeout_s,
    )
    table = nt.table(cfg.nt.table_path)
    publisher = FieldVisionPublisher()
    tracker = DetectionTracker(iou_threshold=0.3, max_age_s=0.5)
    cameras = _build_cameras(cfg)
    camera_infos = [_camera_info(c.cfg) for c in cameras]
    dt = 1.0 / max(1e-6, float(cfg.loop_hz))
    try:
        while True:
            t0 = time.time()
            robot_pose = nt.pose()
            objects: list[FieldObject] = []
            for cam in cameras:
                frame = _read_frame(cam)
                if frame is None:
                    continue
                detections = cam.pipeline.run(frame, t0)
                tracked = tracker.update(
                    camera_name=cam.cfg.name,
                    detections=detections,
                    prefix_for_class=class_map.prefix_for,
                    now_s=t0,
                )
                cam_pose = _camera_pose_field(cam.cfg, robot_pose)
                for td in tracked:
                    cls = class_map.get(td.detection.class_id)
                    fo = _to_field_object(td, cls, cam, cam_pose)
                    if fo is not None:
                        objects.append(fo)
            publisher.publish(
                table=table,
                objects=objects,
                cameras=camera_infos,
                extrinsics_xyzrpy=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                max_per_tick=cfg.max_objects_per_tick,
            )
            nt.flush()
            VisionNTClient.sleep_dt(dt - (time.time() - t0))
    finally:
        for cam in cameras:
            try:
                cam.cap.release()
            except Exception:
                pass


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", dest="config", default=None)
    args = parser.parse_args()
    run(args.config)
