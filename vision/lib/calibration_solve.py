from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, cast

import cv2
import numpy as np


def _collect_images(images_dir: Path) -> list[Path]:
    exts = {".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff"}
    files = [p for p in images_dir.iterdir() if p.is_file() and p.suffix.lower() in exts]
    files.sort()
    return files


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


def _detect_charuco(
    gray: np.ndarray,
    board,
    dictionary,
    detector,
    detector_params,
    min_corners: int,
) -> tuple[bool, np.ndarray | None, np.ndarray | None]:
    aruco = cast(Any, cv2.aruco)
    corners, ids, rejected = _detect_markers(gray, dictionary, detector, detector_params)
    if ids is None or len(ids) <= 0:
        return False, None, None
    if hasattr(aruco, "refineDetectedMarkers"):
        try:
            refined = aruco.refineDetectedMarkers(gray, board, corners, ids, rejected)
            if isinstance(refined, tuple) and len(refined) >= 3:
                corners, ids, rejected = refined[0], refined[1], refined[2]
        except Exception:
            pass
    retval, cc, ci = aruco.interpolateCornersCharuco(corners, ids, gray, board)
    if retval is None or float(retval) < float(min_corners) or cc is None or ci is None:
        return False, None, None
    return True, cc, ci


def _coverage(points: np.ndarray | None, image_size: tuple[int, int]) -> float:
    if points is None:
        return 0.0
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    if pts.shape[0] <= 0:
        return 0.0
    w = max(1.0, float(image_size[0]))
    h = max(1.0, float(image_size[1]))
    xspan = float(np.max(pts[:, 0]) - np.min(pts[:, 0])) / w
    yspan = float(np.max(pts[:, 1]) - np.min(pts[:, 1])) / h
    return max(0.0, xspan * yspan)


def _sharpness(gray: np.ndarray) -> float:
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


def _calib_flags(dist_model: str, zero_tangent: bool, n_views: int) -> int:
    model = str(dist_model).strip().lower()
    flags = 0
    use_rational = model == "rational" or (model == "auto" and int(n_views) >= 20)
    if use_rational:
        flags |= int(cv2.CALIB_RATIONAL_MODEL)
    if bool(zero_tangent):
        flags |= int(cv2.CALIB_ZERO_TANGENT_DIST)
    return flags


def _per_view_errors_checkerboard(
    object_points: list[np.ndarray],
    image_points: list[np.ndarray],
    rvecs: list[np.ndarray],
    tvecs: list[np.ndarray],
    K: np.ndarray,
    dist: np.ndarray,
) -> list[float]:
    out: list[float] = []
    for obj, img, rvec, tvec in zip(object_points, image_points, rvecs, tvecs):
        proj, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
        proj2 = np.asarray(proj, dtype=np.float64).reshape(-1, 2)
        img2 = np.asarray(img, dtype=np.float64).reshape(-1, 2)
        if proj2.shape != img2.shape or proj2.size == 0:
            out.append(float("inf"))
            continue
        err = np.linalg.norm(proj2 - img2, axis=1)
        out.append(float(np.mean(err)))
    return out


def _per_view_errors_charuco(
    board,
    corners_list: list[np.ndarray],
    ids_list: list[np.ndarray],
    rvecs: list[np.ndarray],
    tvecs: list[np.ndarray],
    K: np.ndarray,
    dist: np.ndarray,
) -> list[float]:
    all_pts = np.asarray(board.getChessboardCorners(), dtype=np.float64).reshape(-1, 3)
    out: list[float] = []
    for corners, ids, rvec, tvec in zip(corners_list, ids_list, rvecs, tvecs):
        idx = np.asarray(ids, dtype=np.int32).reshape(-1)
        if idx.size <= 0:
            out.append(float("inf"))
            continue
        obj = all_pts[idx]
        proj, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
        proj2 = np.asarray(proj, dtype=np.float64).reshape(-1, 2)
        img2 = np.asarray(corners, dtype=np.float64).reshape(-1, 2)
        if proj2.shape != img2.shape or proj2.size == 0:
            out.append(float("inf"))
            continue
        err = np.linalg.norm(proj2 - img2, axis=1)
        out.append(float(np.mean(err)))
    return out


def _global_error_from_views(per_view: list[float]) -> float:
    vals = [float(v) for v in per_view if np.isfinite(v)]
    if not vals:
        return 0.0
    return float(np.mean(vals))


def _pruned_indices(
    per_view_errors: list[float],
    max_per_view_error: float,
    mad_mult: float,
    min_keep: int,
) -> tuple[list[int], dict[str, float]]:
    n = len(per_view_errors)
    if n <= 0:
        return [], {"threshold_px": 0.0, "median_px": 0.0, "mad_px": 0.0}
    errs = np.asarray(per_view_errors, dtype=np.float64)
    finite = np.isfinite(errs)
    if not np.any(finite):
        keep = list(range(max(0, min_keep)))
        return keep, {"threshold_px": float("inf"), "median_px": float("inf"), "mad_px": float("inf")}
    med = float(np.median(errs[finite]))
    mad = float(np.median(np.abs(errs[finite] - med)))
    robust = med + max(0.0, float(mad_mult)) * 1.4826 * mad
    hard = float(max_per_view_error) if float(max_per_view_error) > 0.0 else float("inf")
    thr = min(hard, robust if np.isfinite(robust) and robust > 0.0 else hard)
    keep = [i for i, e in enumerate(per_view_errors) if np.isfinite(float(e)) and float(e) <= thr]
    if len(keep) < int(min_keep):
        ranked = np.argsort(np.where(np.isfinite(errs), errs, np.inf)).astype(np.int64).reshape(-1)
        keep = [int(x) for x in ranked[: int(min_keep)]]
    keep = sorted(set(int(i) for i in keep if 0 <= int(i) < n))
    return keep, {"threshold_px": float(thr), "median_px": med, "mad_px": mad}


def _calibrate_charuco(
    corners: list[np.ndarray],
    ids: list[np.ndarray],
    board: Any,
    image_size: tuple[int, int],
    flags: int,
    criteria: tuple[int, int, float],
) -> tuple[float, np.ndarray, np.ndarray, list[np.ndarray], list[np.ndarray]]:
    aruco = cast(Any, cv2.aruco)
    return aruco.calibrateCameraCharuco(
        corners,
        ids,
        board,
        image_size,
        None,
        None,
        flags=flags,
        criteria=criteria,
    )


def _calibrate_checkerboard(
    object_points: list[np.ndarray],
    image_points: list[np.ndarray],
    image_size: tuple[int, int],
    flags: int,
    criteria: tuple[int, int, float],
) -> tuple[float, np.ndarray, np.ndarray, list[np.ndarray], list[np.ndarray]]:
    cv2_any = cast(Any, cv2)
    return cv2_any.calibrateCamera(
        object_points,
        image_points,
        image_size,
        None,
        None,
        flags=flags,
        criteria=criteria,
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--pattern", type=str, default="charuco", choices=["charuco", "checkerboard"])
    parser.add_argument("--images-dir", type=str, default="calib_images")
    parser.add_argument("--cols", type=int, default=9)
    parser.add_argument("--rows", type=int, default=6)
    parser.add_argument("--square-mm", type=float, default=25.0)
    parser.add_argument("--marker-ratio", type=float, default=0.70)
    parser.add_argument("--aruco-dict", type=str, default="DICT_5X5_1000")
    parser.add_argument("--min-charuco-corners", type=int, default=14)
    parser.add_argument("--min-coverage", type=float, default=0.03)
    parser.add_argument("--min-sharpness", type=float, default=80.0)
    parser.add_argument("--max-per-view-error", type=float, default=1.20)
    parser.add_argument("--mad-mult", type=float, default=2.5)
    parser.add_argument("--min-views-after-prune", type=int, default=12)
    parser.add_argument("--dist-model", type=str, default="auto", choices=["auto", "basic", "rational"])
    parser.add_argument("--zero-tangent", action="store_true")
    parser.add_argument("--out-json", type=str, default="calibration.json")
    parser.add_argument("--show-used", action="store_true")
    args = parser.parse_args()

    cols = int(args.cols)
    rows = int(args.rows)
    if cols < 2 or rows < 2:
        raise ValueError("rows and cols must be >= 2")
    square_m = float(args.square_mm) / 1000.0
    if square_m <= 0.0:
        raise ValueError("square-mm must be > 0")

    images_dir = Path(args.images_dir).resolve()
    if not images_dir.exists():
        raise FileNotFoundError(f"images dir not found: {images_dir}")
    files = _collect_images(images_dir)
    if not files:
        raise FileNotFoundError(f"no images found in: {images_dir}")

    mode = str(args.pattern).strip().lower()
    pattern = (cols, rows)
    obj_template = np.zeros((rows * cols, 3), dtype=np.float32)
    grid = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2).astype(np.float32)
    obj_template[:, :2] = grid * np.float32(square_m)

    board = None
    dictionary = None
    aruco_detector = None
    aruco_params = None
    if mode == "charuco":
        board, dictionary = _make_charuco_board(
            cols=cols,
            rows=rows,
            marker_ratio=float(args.marker_ratio),
            dict_name=str(args.aruco_dict),
        )
        aruco_detector, aruco_params = _make_aruco_detector(dictionary)

    image_size: tuple[int, int] | None = None
    used_files: list[Path] = []
    used_coverage: list[float] = []
    used_sharpness: list[float] = []
    object_points: list[np.ndarray] = []
    image_points: list[np.ndarray] = []
    charuco_corners: list[np.ndarray] = []
    charuco_ids: list[np.ndarray] = []

    for p in files:
        img = cv2.imread(str(p), cv2.IMREAD_COLOR)
        if img is None or img.size == 0:
            continue
        h, w = img.shape[:2]
        if image_size is None:
            image_size = (w, h)
        elif image_size != (w, h):
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        sharp = _sharpness(gray)
        if sharp < float(args.min_sharpness):
            continue
        if mode == "charuco":
            ok, cc, ci = _detect_charuco(
                gray,
                board,
                dictionary,
                aruco_detector,
                aruco_params,
                min_corners=max(4, int(args.min_charuco_corners)),
            )
            if not ok or cc is None or ci is None:
                continue
            cov = _coverage(cc, image_size)
            if cov < float(args.min_coverage):
                continue
            charuco_corners.append(np.asarray(cc, dtype=np.float32))
            charuco_ids.append(np.asarray(ci, dtype=np.int32))
            used_files.append(p)
            used_coverage.append(float(cov))
            used_sharpness.append(float(sharp))
        else:
            found, corners = _detect_checkerboard(gray, pattern)
            if not found or corners is None:
                continue
            cov = _coverage(corners, image_size)
            if cov < float(args.min_coverage):
                continue
            object_points.append(obj_template.copy())
            image_points.append(np.asarray(corners, dtype=np.float32))
            used_files.append(p)
            used_coverage.append(float(cov))
            used_sharpness.append(float(sharp))

    if image_size is None:
        raise RuntimeError("no readable images found")

    n_initial = len(used_files)
    if n_initial < 8:
        raise RuntimeError(f"need at least 8 good images, got {n_initial}")

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 120, 1e-9)
    min_keep = max(8, int(args.min_views_after_prune))
    flags = _calib_flags(str(args.dist_model), bool(args.zero_tangent), n_initial)

    if mode == "charuco":
        rms0, K0, dist0, rvecs0, tvecs0 = _calibrate_charuco(charuco_corners, charuco_ids, board, image_size, flags, criteria)
        per_view0 = _per_view_errors_charuco(board, charuco_corners, charuco_ids, rvecs0, tvecs0, K0, dist0)
    else:
        rms0, K0, dist0, rvecs0, tvecs0 = _calibrate_checkerboard(object_points, image_points, image_size, flags, criteria)
        per_view0 = _per_view_errors_checkerboard(object_points, image_points, rvecs0, tvecs0, K0, dist0)

    keep_idx, prune_meta = _pruned_indices(
        per_view_errors=per_view0,
        max_per_view_error=float(args.max_per_view_error),
        mad_mult=float(args.mad_mult),
        min_keep=min_keep,
    )
    rejected_idx = sorted(set(range(n_initial)) - set(keep_idx))

    if len(keep_idx) < 8:
        keep_idx = list(range(n_initial))
        rejected_idx = []

    if len(keep_idx) == n_initial:
        rms = rms0
        K = K0
        dist = dist0
        per_view = per_view0
        final_files = used_files
        final_cov = used_coverage
        final_sharp = used_sharpness
    else:
        if mode == "charuco":
            cc1 = [charuco_corners[i] for i in keep_idx]
            ci1 = [charuco_ids[i] for i in keep_idx]
            rms, K, dist, rvecs, tvecs = _calibrate_charuco(cc1, ci1, board, image_size, flags, criteria)
            per_view = _per_view_errors_charuco(board, cc1, ci1, rvecs, tvecs, K, dist)
        else:
            op1 = [object_points[i] for i in keep_idx]
            ip1 = [image_points[i] for i in keep_idx]
            rms, K, dist, rvecs, tvecs = _calibrate_checkerboard(op1, ip1, image_size, flags, criteria)
            per_view = _per_view_errors_checkerboard(op1, ip1, rvecs, tvecs, K, dist)
        final_files = [used_files[i] for i in keep_idx]
        final_cov = [used_coverage[i] for i in keep_idx]
        final_sharp = [used_sharpness[i] for i in keep_idx]

    mean_err = _global_error_from_views(per_view)
    result = {
        "pattern": mode,
        "image_width": int(image_size[0]),
        "image_height": int(image_size[1]),
        "images_total": int(len(files)),
        "images_used_initial": int(n_initial),
        "images_used_final": int(len(final_files)),
        "images_rejected_outlier": int(len(rejected_idx)),
        "rms_reprojection_error": float(rms),
        "mean_reprojection_error_px": float(mean_err),
        "max_reprojection_error_px": float(max(per_view) if per_view else 0.0),
        "min_coverage": float(args.min_coverage),
        "mean_coverage": float(np.mean(final_cov) if final_cov else 0.0),
        "mean_sharpness": float(np.mean(final_sharp) if final_sharp else 0.0),
        "prune_threshold_px": float(prune_meta["threshold_px"]),
        "prune_median_px": float(prune_meta["median_px"]),
        "prune_mad_px": float(prune_meta["mad_px"]),
        "dist_model": str(args.dist_model).lower(),
        "zero_tangent": bool(args.zero_tangent),
        "K": np.asarray(K, dtype=np.float64).reshape(3, 3).tolist(),
        "dist": np.asarray(dist, dtype=np.float64).reshape(-1).tolist(),
        "runtime_camera_calibration": {
            "calibration": {
                "K": np.asarray(K, dtype=np.float64).reshape(3, 3).tolist(),
                "dist": np.asarray(dist, dtype=np.float64).reshape(-1).tolist(),
            }
        },
    }
    if bool(args.show_used):
        result["used_files"] = [str(p) for p in final_files]
        result["rejected_files"] = [str(used_files[i]) for i in rejected_idx]

    out_path = Path(args.out_json).resolve()
    out_path.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(str(out_path))
    print(json.dumps(result["runtime_camera_calibration"], indent=2))


if __name__ == "__main__":
    main()
