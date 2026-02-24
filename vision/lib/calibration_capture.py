from __future__ import annotations

import argparse
import time
from pathlib import Path

import cv2
import numpy as np


def _next_path(out_dir: Path, prefix: str, ext: str, idx: int) -> Path:
    return out_dir / f"{prefix}_{idx:04d}.{ext}"


def _dict_by_name(name: str):
    if not hasattr(cv2, "aruco"):
        raise RuntimeError("cv2.aruco is required for charuco")
    code = getattr(cv2.aruco, str(name).strip().upper(), None)
    if code is None:
        raise ValueError(f"unsupported aruco dictionary: {name}")
    return cv2.aruco.getPredefinedDictionary(int(code))


def _make_charuco_board(cols: int, rows: int, marker_ratio: float, dict_name: str):
    dictionary = _dict_by_name(dict_name)
    sx = int(cols) + 1
    sy = int(rows) + 1
    mr = float(marker_ratio)
    if mr <= 0.0 or mr >= 1.0:
        raise ValueError("marker-ratio must be in (0, 1)")
    if hasattr(cv2.aruco, "CharucoBoard"):
        board = cv2.aruco.CharucoBoard((sx, sy), 1.0, mr, dictionary)
    else:
        board = cv2.aruco.CharucoBoard_create(sx, sy, 1.0, mr, dictionary)
    return board, dictionary


def _detect_checkerboard(gray: np.ndarray, pattern_size: tuple[int, int]) -> tuple[bool, np.ndarray | None]:
    found, corners = cv2.findChessboardCornersSB(gray, pattern_size, None)
    if found and corners is not None:
        return True, corners
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK
    found2, corners2 = cv2.findChessboardCorners(gray, pattern_size, flags)
    if not found2 or corners2 is None:
        return False, None
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001)
    cv2.cornerSubPix(gray, corners2, (11, 11), (-1, -1), criteria)
    return True, corners2


def _detect_charuco(gray: np.ndarray, board, dictionary, min_corners: int) -> tuple[bool, np.ndarray | None, np.ndarray | None, tuple]:
    corners, ids, rejected = cv2.aruco.detectMarkers(gray, dictionary)
    if ids is None or len(ids) <= 0:
        return False, None, None, (corners, ids, rejected)
    retval, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, board)
    if retval is None or float(retval) < float(min_corners) or charuco_corners is None or charuco_ids is None:
        return False, None, None, (corners, ids, rejected)
    return True, charuco_corners, charuco_ids, (corners, ids, rejected)


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
    if mode == "charuco":
        charuco_board, aruco_dictionary = _make_charuco_board(
            cols=cols,
            rows=rows,
            marker_ratio=float(args.marker_ratio),
            dict_name=str(args.aruco_dict),
        )

    cap = cv2.VideoCapture(int(args.camera), cv2.CAP_DSHOW)
    if not cap.isOpened():
        cap = cv2.VideoCapture(int(args.camera))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(args.width))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(args.height))
    cap.set(cv2.CAP_PROP_FPS, float(args.fps))
    if not cap.isOpened():
        raise RuntimeError(f"failed to open camera index {int(args.camera)}")

    print("keys: q/esc quit, space save when board found, a toggle auto")
    auto_mode = bool(args.auto)
    saved = 0
    idx = 0
    last_save = 0.0
    try:
        cv2.namedWindow("Calibration Capture", cv2.WINDOW_NORMAL)
        while True:
            ok, frame = cap.read()
            if not ok or frame is None or frame.size == 0:
                continue
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            vis = frame.copy()
            if mode == "charuco":
                found, cc, ci, marker_pack = _detect_charuco(
                    gray,
                    charuco_board,
                    aruco_dictionary,
                    min_corners=max(4, int(args.min_charuco_corners)),
                )
                marker_corners, marker_ids, _ = marker_pack
                if marker_ids is not None and len(marker_ids) > 0:
                    cv2.aruco.drawDetectedMarkers(vis, marker_corners, marker_ids)
                if found and cc is not None and ci is not None:
                    cv2.aruco.drawDetectedCornersCharuco(vis, cc, ci, (0, 255, 0))
            else:
                found, corners = _detect_checkerboard(gray, pattern)
                if found and corners is not None:
                    cv2.drawChessboardCorners(vis, pattern, corners, found)
            state = "FOUND" if found else "MISSING"
            cap_mode = "AUTO" if auto_mode else "MANUAL"
            cv2.putText(
                vis,
                f"{str(args.pattern).upper()} {state}  {cap_mode}  saved:{saved}",
                (14, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                (0, 255, 0) if found else (0, 120, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.imshow("Calibration Capture", vis)

            now = time.time()
            if auto_mode and found and (now - last_save) >= float(args.interval_s):
                out_path = _next_path(out_dir, str(args.prefix), str(args.ext), idx)
                cv2.imwrite(str(out_path), frame)
                saved += 1
                idx += 1
                last_save = now
                print(str(out_path))
                if int(args.target_count) > 0 and saved >= int(args.target_count):
                    break

            key = int(cv2.waitKey(1) & 0xFF)
            if key in (27, ord("q")):
                break
            if key == ord("a"):
                auto_mode = not auto_mode
            if key == ord(" ") and found:
                out_path = _next_path(out_dir, str(args.prefix), str(args.ext), idx)
                cv2.imwrite(str(out_path), frame)
                saved += 1
                idx += 1
                last_save = now
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
