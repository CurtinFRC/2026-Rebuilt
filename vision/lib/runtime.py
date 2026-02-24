from __future__ import annotations

import argparse
import math
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
from lib.runtime_types import CameraInfoOut, Detection2D, FieldObject, TrackedDetection
from lib.tracker import DetectionTracker


@dataclass
class _CameraRuntime:
    cfg: CameraRuntimeConfig
    cap: cv2.VideoCapture
    pipeline: VisionPipeline
    calib: CameraCalibration


@dataclass(frozen=True)
class _DisplayDetection:
    tracked: TrackedDetection
    type_name: str
    distance_m: float | None


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


def _color_for_class(class_id: int) -> tuple[int, int, int]:
    palette: list[tuple[int, int, int]] = [
        (54, 67, 244),
        (99, 30, 233),
        (176, 39, 156),
        (183, 58, 103),
        (181, 81, 63),
        (243, 150, 33),
        (212, 188, 0),
        (136, 150, 0),
        (80, 175, 76),
        (74, 195, 139),
        (57, 220, 205),
        (59, 235, 255),
    ]
    return palette[int(class_id) % len(palette)]


def _draw_text_with_bg(
    frame: np.ndarray,
    text: str,
    x: int,
    y: int,
    fg: tuple[int, int, int] = (255, 255, 255),
    bg: tuple[int, int, int] = (10, 10, 10),
    scale: float = 0.5,
    thickness: int = 1,
) -> None:
    (tw, th), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, thickness)
    x0 = max(0, int(x))
    y0 = max(th + baseline + 2, int(y))
    pad = 3
    cv2.rectangle(
        frame,
        (x0 - pad, y0 - th - baseline - pad),
        (x0 + tw + pad, y0 + baseline + pad),
        bg,
        thickness=-1,
    )
    cv2.putText(
        frame,
        text,
        (x0, y0),
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        fg,
        thickness,
        lineType=cv2.LINE_AA,
    )


def _render_camera_frame(frame: np.ndarray, camera_name: str, detections: list[_DisplayDetection]) -> np.ndarray:
    out = frame.copy()
    _draw_text_with_bg(out, f"{camera_name} ({len(detections)} dets)", 8, 20, scale=0.6, thickness=2)
    for item in detections:
        d = item.tracked.detection
        x1 = int(round(float(d.x1)))
        y1 = int(round(float(d.y1)))
        x2 = int(round(float(d.x2)))
        y2 = int(round(float(d.y2)))
        color = _color_for_class(d.class_id)
        cv2.rectangle(out, (x1, y1), (x2, y2), color, 2, lineType=cv2.LINE_AA)
        label = f"{item.type_name} {d.confidence:.2f}"
        if item.distance_m is not None:
            label += f" {item.distance_m:.2f}m"
        _draw_text_with_bg(out, label, x1 + 2, y1 - 4, bg=color)
        _draw_text_with_bg(out, item.tracked.oid, x1 + 2, y2 + 14, bg=color)
    return out


def _compose_display_window(frames: list[np.ndarray]) -> np.ndarray:
    if not frames:
        return np.zeros((240, 320, 3), dtype=np.uint8)
    n = len(frames)
    cols = 1 if n == 1 else 2
    rows = int(math.ceil(float(n) / float(cols)))
    target_h = 720 if n == 1 else 360
    resized: list[np.ndarray] = []
    tile_w = 0
    for f in frames:
        h, w = f.shape[:2]
        if h <= 0 or w <= 0:
            continue
        new_w = max(1, int(round((float(w) / float(h)) * float(target_h))))
        tile_w = max(tile_w, new_w)
        resized.append(cv2.resize(f, (new_w, target_h), interpolation=cv2.INTER_AREA))
    if not resized:
        return np.zeros((240, 320, 3), dtype=np.uint8)
    padded: list[np.ndarray] = []
    for f in resized:
        canvas = np.zeros((target_h, tile_w, 3), dtype=np.uint8)
        canvas[:, : f.shape[1]] = f
        padded.append(canvas)
    while len(padded) < rows * cols:
        padded.append(np.zeros((target_h, tile_w, 3), dtype=np.uint8))
    row_imgs: list[np.ndarray] = []
    for r in range(rows):
        chunk = padded[r * cols : (r + 1) * cols]
        row_imgs.append(cv2.hconcat(chunk))
    out = row_imgs[0] if len(row_imgs) == 1 else cv2.vconcat(row_imgs)
    max_w = 1800
    if out.shape[1] > max_w:
        new_h = max(1, int(round(float(out.shape[0]) * (float(max_w) / float(out.shape[1])))))
        out = cv2.resize(out, (max_w, new_h), interpolation=cv2.INTER_AREA)
    return out


def _iou_xyxy(
    a_x1: float,
    a_y1: float,
    a_x2: float,
    a_y2: float,
    b_x1: float,
    b_y1: float,
    b_x2: float,
    b_y2: float,
) -> float:
    ix1 = max(float(a_x1), float(b_x1))
    iy1 = max(float(a_y1), float(b_y1))
    ix2 = min(float(a_x2), float(b_x2))
    iy2 = min(float(a_y2), float(b_y2))
    iw = max(0.0, ix2 - ix1)
    ih = max(0.0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0.0:
        return 0.0
    area_a = max(0.0, float(a_x2) - float(a_x1)) * max(0.0, float(a_y2) - float(a_y1))
    area_b = max(0.0, float(b_x2) - float(b_x1)) * max(0.0, float(b_y2) - float(b_y1))
    denom = area_a + area_b - inter
    if denom <= 0.0:
        return 0.0
    return inter / denom


def _refine_fuel_detection_box(hsv_frame: np.ndarray, det: Detection2D) -> Detection2D:
    h, w = hsv_frame.shape[:2]
    x1 = int(max(0, min(w - 1, int(math.floor(float(det.x1))))))
    y1 = int(max(0, min(h - 1, int(math.floor(float(det.y1))))))
    x2 = int(max(0, min(w, int(math.ceil(float(det.x2))))))
    y2 = int(max(0, min(h, int(math.ceil(float(det.y2))))))
    roi_w = x2 - x1
    roi_h = y2 - y1
    if roi_w < 4 or roi_h < 4:
        return det

    hsv_roi = hsv_frame[y1:y2, x1:x2]
    if hsv_roi.size == 0:
        return det

    # Mask yellow-ish fuel tones. Thresholds are intentionally broad to tolerate lighting shifts.
    yellow = cv2.inRange(hsv_roi, (12, 55, 45), (45, 255, 255))
    if cv2.countNonZero(yellow) <= 0:
        return det
    k = np.ones((3, 3), dtype=np.uint8)
    yellow = cv2.morphologyEx(yellow, cv2.MORPH_OPEN, k)
    yellow = cv2.morphologyEx(yellow, cv2.MORPH_CLOSE, k)

    contours, _ = cv2.findContours(yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return det

    roi_area = float(roi_w * roi_h)
    min_area = max(12.0, 0.01 * roi_area)
    roi_cx = 0.5 * float(roi_w)
    roi_cy = 0.5 * float(roi_h)

    best: tuple[np.ndarray, int, int, int, int, float, float] | None = None
    best_score = float("-inf")
    for c in contours:
        area = float(cv2.contourArea(c))
        if area < min_area:
            continue
        peri = float(cv2.arcLength(c, True))
        if peri <= 1e-6:
            continue
        bx, by, bw, bh = cv2.boundingRect(c)
        if bw < 2 or bh < 2:
            continue
        circularity = float(4.0 * math.pi * area / max(1e-9, peri * peri))
        aspect = float(min(bw, bh)) / float(max(bw, bh))
        fill = float(area) / float(max(1, bw * bh))
        ccx = float(bx) + 0.5 * float(bw)
        ccy = float(by) + 0.5 * float(bh)
        center_norm = math.hypot(
            (ccx - roi_cx) / max(1.0, float(roi_w)),
            (ccy - roi_cy) / max(1.0, float(roi_h)),
        )
        score = area * (1.0 + 0.50 * max(0.0, min(1.0, circularity)) + 0.35 * aspect + 0.20 * fill) - 0.25 * area * center_norm
        if score > best_score:
            best_score = score
            best = (c, bx, by, bw, bh, aspect, circularity)

    if best is None:
        return det

    best_contour, bx, by, bw, bh, aspect, circularity = best
    (cx_roi, cy_roi), radius = cv2.minEnclosingCircle(best_contour)
    if float(radius) <= 1.0:
        return det
    side = max(float(max(bw, bh)), float(2.0 * radius))
    side *= 1.08 if (circularity >= 0.45 and aspect >= 0.45) else 1.15
    side = min(side, 1.10 * float(max(roi_w, roi_h)))

    cx = float(x1) + float(cx_roi)
    cy = float(y1) + float(cy_roi)
    rx1 = max(0.0, min(float(w - 1), cx - 0.5 * side))
    ry1 = max(0.0, min(float(h - 1), cy - 0.5 * side))
    rx2 = max(0.0, min(float(w), cx + 0.5 * side))
    ry2 = max(0.0, min(float(h), cy + 0.5 * side))
    if rx2 - rx1 < 4.0 or ry2 - ry1 < 4.0:
        return det

    orig_area = max(1e-6, (float(det.x2) - float(det.x1)) * (float(det.y2) - float(det.y1)))
    new_area = max(1e-6, (rx2 - rx1) * (ry2 - ry1))
    area_ratio = new_area / orig_area
    if area_ratio < 0.08 or area_ratio > 1.50:
        return det
    if _iou_xyxy(float(det.x1), float(det.y1), float(det.x2), float(det.y2), rx1, ry1, rx2, ry2) < 0.15:
        return det

    meta = dict(det.metadata) if det.metadata else {}
    meta["bbox_refined"] = True
    meta["bbox_refiner"] = "yellow_ball"
    return Detection2D(
        class_id=int(det.class_id),
        confidence=float(det.confidence),
        x1=float(rx1),
        y1=float(ry1),
        x2=float(rx2),
        y2=float(ry2),
        pipeline=str(det.pipeline),
        metadata=meta,
    )


def _refine_fuel_detections(
    frame_bgr: np.ndarray,
    detections: list[Detection2D],
    class_map: ClassMap,
) -> list[Detection2D]:
    if frame_bgr is None or frame_bgr.size == 0 or not detections:
        return detections
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    out: list[Detection2D] = []
    for det in detections:
        cls = class_map.get(det.class_id)
        if not cls.enabled or cls.estimator.strip().lower() != "fuel":
            out.append(det)
            continue
        out.append(_refine_fuel_detection_box(hsv, det))
    return out


def run(config_path: str | None = None, display: bool = False) -> None:
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
        if display:
            cv2.namedWindow("Vision Runtime", cv2.WINDOW_NORMAL)
        while True:
            t0 = time.time()
            robot_pose = nt.pose()
            objects: list[FieldObject] = []
            display_frames: list[np.ndarray] = []
            for cam in cameras:
                frame = _read_frame(cam)
                if frame is None:
                    if display:
                        empty = np.zeros((240, 320, 3), dtype=np.uint8)
                        _draw_text_with_bg(empty, f"{cam.cfg.name}: no frame", 8, 22, scale=0.6, thickness=2)
                        display_frames.append(empty)
                    continue
                detections = cam.pipeline.run(frame, t0)
                detections = _refine_fuel_detections(frame, detections, class_map)
                tracked = tracker.update(
                    camera_name=cam.cfg.name,
                    detections=detections,
                    prefix_for_class=class_map.prefix_for,
                    now_s=t0,
                )
                cam_pose = _camera_pose_field(cam.cfg, robot_pose)
                display_detections: list[_DisplayDetection] = []
                for td in tracked:
                    cls = class_map.get(td.detection.class_id)
                    fo = _to_field_object(td, cls, cam, cam_pose)
                    if fo is not None:
                        objects.append(fo)
                    distance_m: float | None = None
                    if fo is not None:
                        obj = np.array([fo.x, fo.y, fo.z], dtype=np.float64)
                        distance_m = float(np.linalg.norm(obj - cam_pose.p_f))
                    display_detections.append(
                        _DisplayDetection(
                            tracked=td,
                            type_name=cls.type_name,
                            distance_m=distance_m,
                        )
                    )
                if display:
                    display_frames.append(_render_camera_frame(frame, cam.cfg.name, display_detections))
            publisher.publish(
                table=table,
                objects=objects,
                cameras=camera_infos,
                extrinsics_xyzrpy=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                max_per_tick=cfg.max_objects_per_tick,
            )
            nt.flush()
            if display:
                cv2.imshow("Vision Runtime", _compose_display_window(display_frames))
                key = int(cv2.waitKey(1) & 0xFF)
                if key in (27, ord("q")):
                    break
            VisionNTClient.sleep_dt(dt - (time.time() - t0))
    finally:
        for cam in cameras:
            try:
                cam.cap.release()
            except Exception:
                pass
        if display:
            try:
                cv2.destroyWindow("Vision Runtime")
            except Exception:
                cv2.destroyAllWindows()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", dest="config", default=None)
    parser.add_argument("--display", dest="display", action="store_true")
    args = parser.parse_args()
    run(args.config, display=bool(args.display))
