from __future__ import annotations

import argparse
import math
import time
from pathlib import Path
from typing import Any, cast

import cv2
import numpy as np


def _next_path(out_dir: Path, prefix: str, ext: str, idx: int) -> Path:
    return out_dir / f"{prefix}_{idx:04d}.{ext}"


def _starting_index(out_dir: Path, prefix: str, ext: str) -> int:
    mx = -1
    for p in out_dir.glob(f"{prefix}_*.{ext}"):
        stem = p.stem
        base = f"{prefix}_"
        if not stem.startswith(base):
            continue
        tail = stem[len(base) :]
        if tail.isdigit():
            mx = max(mx, int(tail))
    return mx + 1


def _dict_by_name(name: str):
    if not hasattr(cv2, "aruco"):
        raise RuntimeError("cv2.aruco is required for charuco")
    aruco = cast(Any, cv2.aruco)
    code = getattr(aruco, str(name).strip().upper(), None)
    if code is None:
        raise ValueError(f"unsupported aruco dictionary: {name}")
    return aruco.getPredefinedDictionary(int(code))


def _make_charuco_board(cols: int, rows: int, marker_ratio: float, dict_name: str):
    aruco = cast(Any, cv2.aruco)
    dictionary = _dict_by_name(dict_name)
    sx = int(cols) + 1
    sy = int(rows) + 1
    mr = float(marker_ratio)
    if mr <= 0.0 or mr >= 1.0:
        raise ValueError("marker-ratio must be in (0, 1)")
    if hasattr(aruco, "CharucoBoard"):
        board = aruco.CharucoBoard((sx, sy), 1.0, mr, dictionary)
    else:
        board = aruco.CharucoBoard_create(sx, sy, 1.0, mr, dictionary)
    return board, dictionary


def _make_aruco_detector(dictionary):
    aruco = cast(Any, cv2.aruco)
    params = None
    if hasattr(aruco, "DetectorParameters"):
        params = aruco.DetectorParameters()
    elif hasattr(aruco, "DetectorParameters_create"):
        params = aruco.DetectorParameters_create()
    if hasattr(aruco, "ArucoDetector") and params is not None:
        return aruco.ArucoDetector(dictionary, params), params
    return None, params


def _detect_markers(gray: np.ndarray, dictionary, detector, detector_params):
    aruco = cast(Any, cv2.aruco)
    if detector is not None:
        return detector.detectMarkers(gray)
    if detector_params is None:
        return aruco.detectMarkers(gray, dictionary)
    return aruco.detectMarkers(gray, dictionary, parameters=detector_params)


def _detect_checkerboard(gray: np.ndarray, pattern_size: tuple[int, int]) -> tuple[bool, np.ndarray | None]:
    found, corners = cv2.findChessboardCornersSB(gray, pattern_size, None)
    if found and corners is not None:
        return True, corners
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK
    found2, corners2 = cv2.findChessboardCorners(gray, pattern_size, None, flags)
    if not found2 or corners2 is None:
        return False, None
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001)
    cv2.cornerSubPix(gray, corners2, (11, 11), (-1, -1), criteria)
    return True, corners2


def _detect_charuco(
    gray: np.ndarray,
    board,
    dictionary,
    detector,
    detector_params,
    min_corners: int,
) -> tuple[bool, np.ndarray | None, np.ndarray | None, tuple]:
    aruco = cast(Any, cv2.aruco)
    corners, ids, rejected = _detect_markers(gray, dictionary, detector, detector_params)
    if ids is None or len(ids) <= 0:
        return False, None, None, (corners, ids, rejected)
    if hasattr(aruco, "refineDetectedMarkers"):
        try:
            refined = aruco.refineDetectedMarkers(gray, board, corners, ids, rejected)
            if isinstance(refined, tuple) and len(refined) >= 3:
                corners, ids, rejected = refined[0], refined[1], refined[2]
        except Exception:
            pass
    retval, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(corners, ids, gray, board)
    if retval is None or float(retval) < float(min_corners) or charuco_corners is None or charuco_ids is None:
        return False, None, None, (corners, ids, rejected)
    return True, charuco_corners, charuco_ids, (corners, ids, rejected)


def _feature_from_points(points: np.ndarray | None, frame_w: int, frame_h: int) -> tuple[float, float, float, float, float]:
    if points is None:
        return 0.0, 0.0, 0.0, 0.0, 0.0
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    if pts.shape[0] <= 0:
        return 0.0, 0.0, 0.0, 0.0, 0.0
    w = max(1.0, float(frame_w))
    h = max(1.0, float(frame_h))
    cx = float(np.mean(pts[:, 0])) / w
    cy = float(np.mean(pts[:, 1])) / h
    xspan = (float(np.max(pts[:, 0])) - float(np.min(pts[:, 0]))) / w
    yspan = (float(np.max(pts[:, 1])) - float(np.min(pts[:, 1]))) / h
    coverage = max(0.0, xspan * yspan)
    scale = math.sqrt(max(1e-12, coverage))
    ang = 0.0
    if pts.shape[0] >= 5:
        try:
            (_, _), (_, _), ellipse_ang = cv2.fitEllipse(pts.astype(np.float32))
            ang = float(ellipse_ang) / 180.0
        except Exception:
            ang = 0.0
    return cx, cy, scale, ang, coverage


def _diversity_score(cur: tuple[float, float, float, float, float], prev: tuple[float, float, float, float, float] | None) -> float:
    if prev is None:
        return 1e9
    dc = math.hypot(cur[0] - prev[0], cur[1] - prev[1])
    ds = abs(cur[2] - prev[2])
    da = abs(cur[3] - prev[3])
    da = min(da, abs(1.0 - da))
    return dc + 0.6 * ds + 0.3 * da


def _sharpness(gray: np.ndarray) -> float:
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--pattern", type=str, default="charuco", choices=["charuco", "checkerboard"])
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=float, default=20.0)
    parser.add_argument("--cols", type=int, default=9)
    parser.add_argument("--rows", type=int, default=6)
    parser.add_argument("--marker-ratio", type=float, default=0.70)
    parser.add_argument("--aruco-dict", type=str, default="DICT_5X5_1000")
    parser.add_argument("--min-charuco-corners", type=int, default=14)
    parser.add_argument("--out-dir", type=str, default="calib_images")
    parser.add_argument("--prefix", type=str, default="calib")
    parser.add_argument("--ext", type=str, default="png")
    parser.add_argument("--auto", action="store_true")
    parser.add_argument("--interval-s", type=float, default=0.8)
    parser.add_argument("--target-count", type=int, default=0)
    parser.add_argument("--min-sharpness", type=float, default=80.0)
    parser.add_argument("--min-coverage", type=float, default=0.03)
    parser.add_argument("--min-diversity", type=float, default=0.08)
    parser.add_argument("--manual-ignore-quality", action="store_true")
    args = parser.parse_args()

    cols = int(args.cols)
    rows = int(args.rows)
    if cols < 2 or rows < 2:
        raise ValueError("rows and cols must be >= 2")

    out_dir = Path(args.out_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    pattern = (cols, rows)
    mode = str(args.pattern).strip().lower()

    charuco_board = None
    aruco_dictionary = None
    aruco_detector = None
    aruco_params = None
    if mode == "charuco":
        charuco_board, aruco_dictionary = _make_charuco_board(
            cols=cols,
            rows=rows,
            marker_ratio=float(args.marker_ratio),
            dict_name=str(args.aruco_dict),
        )
        aruco_detector, aruco_params = _make_aruco_detector(aruco_dictionary)

    cap = cv2.VideoCapture(int(args.camera), cv2.CAP_DSHOW)
    if not cap.isOpened():
        cap = cv2.VideoCapture(int(args.camera))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(args.width))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(args.height))
    cap.set(cv2.CAP_PROP_FPS, float(args.fps))
    if not cap.isOpened():
        raise RuntimeError(f"failed to open camera index {int(args.camera)}")

    print("keys: q/esc quit, space save, a toggle auto")
    auto_mode = bool(args.auto)
    saved = 0
    idx = _starting_index(out_dir, str(args.prefix), str(args.ext))
    last_save = 0.0
    prev_saved_feature: tuple[float, float, float, float, float] | None = None

    try:
        cv2.namedWindow("Calibration Capture", cv2.WINDOW_NORMAL)
        while True:
            ok, frame = cap.read()
            if not ok or frame is None or frame.size == 0:
                continue
            h, w = frame.shape[:2]
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            vis = frame.copy()

            found = False
            points: np.ndarray | None = None
            if mode == "charuco":
                found, cc, ci, marker_pack = _detect_charuco(
                    gray,
                    charuco_board,
                    aruco_dictionary,
                    aruco_detector,
                    aruco_params,
                    min_corners=max(4, int(args.min_charuco_corners)),
                )
                marker_corners, marker_ids, _ = marker_pack
                if marker_ids is not None and len(marker_ids) > 0:
                    cast(Any, cv2.aruco).drawDetectedMarkers(vis, marker_corners, marker_ids)
                if found and cc is not None and ci is not None:
                    cast(Any, cv2.aruco).drawDetectedCornersCharuco(vis, cc, ci, (0, 255, 0))
                    points = cc
            else:
                found, corners = _detect_checkerboard(gray, pattern)
                if found and corners is not None:
                    cv2.drawChessboardCorners(vis, pattern, corners, found)
                    points = corners

            sharp = _sharpness(gray)
            feat = _feature_from_points(points, w, h) if found else (0.0, 0.0, 0.0, 0.0, 0.0)
            coverage = feat[4]
            div = _diversity_score(feat, prev_saved_feature) if found else 0.0
            quality_ok = found and sharp >= float(args.min_sharpness) and coverage >= float(args.min_coverage)
            diverse_ok = (not found) or div >= float(args.min_diversity) or prev_saved_feature is None

            state = "FOUND" if found else "MISSING"
            cap_mode = "AUTO" if auto_mode else "MANUAL"
            q_state = "OK" if quality_ok else "LOW"
            d_state = "OK" if diverse_ok else "SIM"
            txt = f"{str(args.pattern).upper()} {state} {cap_mode} q:{q_state} d:{d_state} saved:{saved} sharp:{sharp:.0f} cov:{coverage:.3f} div:{div:.3f}"
            cv2.putText(
                vis,
                txt,
                (10, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 0) if quality_ok and diverse_ok else (0, 165, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.imshow("Calibration Capture", vis)

            now = time.time()
            if auto_mode and quality_ok and diverse_ok and (now - last_save) >= float(args.interval_s):
                out_path = _next_path(out_dir, str(args.prefix), str(args.ext), idx)
                cv2.imwrite(str(out_path), frame)
                saved += 1
                idx += 1
                last_save = now
                prev_saved_feature = feat
                print(str(out_path))
                if int(args.target_count) > 0 and saved >= int(args.target_count):
                    break

            key = int(cv2.waitKey(1) & 0xFF)
            if key in (27, ord("q")):
                break
            if key == ord("a"):
                auto_mode = not auto_mode
            if key == ord(" ") and found:
                if bool(args.manual_ignore_quality) or quality_ok:
                    out_path = _next_path(out_dir, str(args.prefix), str(args.ext), idx)
                    cv2.imwrite(str(out_path), frame)
                    saved += 1
                    idx += 1
                    last_save = now
                    prev_saved_feature = feat
                    print(str(out_path))
                    if int(args.target_count) > 0 and saved >= int(args.target_count):
                        break
    finally:
        cap.release()
        try:
            cv2.destroyWindow("Calibration Capture")
        except Exception:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
