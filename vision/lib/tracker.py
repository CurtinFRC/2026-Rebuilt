from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

from lib.runtime_types import Detection2D, TrackedDetection


@dataclass
class _Track:
    oid: str
    class_id: int
    x1: float
    y1: float
    x2: float
    y2: float
    last_seen: float


def _iou(
    ax1: float,
    ay1: float,
    ax2: float,
    ay2: float,
    bx1: float,
    by1: float,
    bx2: float,
    by2: float,
) -> float:
    ix1 = max(ax1, bx1)
    iy1 = max(ay1, by1)
    ix2 = min(ax2, bx2)
    iy2 = min(ay2, by2)
    iw = max(0.0, ix2 - ix1)
    ih = max(0.0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0.0:
        return 0.0
    a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    denom = a + b - inter
    if denom <= 0.0:
        return 0.0
    return inter / denom


class DetectionTracker:
    def __init__(self, iou_threshold: float = 0.3, max_age_s: float = 0.5):
        self._iou_threshold = float(iou_threshold)
        self._max_age_s = float(max_age_s)
        self._tracks: dict[str, list[_Track]] = {}
        self._next_ids: dict[tuple[str, str], int] = {}

    def _new_id(self, camera_name: str, prefix: str) -> str:
        key = (camera_name, prefix)
        n = self._next_ids.get(key, 0)
        self._next_ids[key] = n + 1
        return f"{camera_name}_{prefix}_{n}"

    def update(
        self,
        camera_name: str,
        detections: list[Detection2D],
        prefix_for_class: Callable[[int], str],
        now_s: float,
    ) -> list[TrackedDetection]:
        now = float(now_s)
        tracks = self._tracks.get(camera_name, [])
        tracks = [t for t in tracks if now - t.last_seen <= self._max_age_s]
        matched_tracks: set[int] = set()
        matched_dets: set[int] = set()
        pairs: list[tuple[float, int, int]] = []
        for ti, t in enumerate(tracks):
            for di, d in enumerate(detections):
                if t.class_id != d.class_id:
                    continue
                s = _iou(t.x1, t.y1, t.x2, t.y2, d.x1, d.y1, d.x2, d.y2)
                if s >= self._iou_threshold:
                    pairs.append((s, ti, di))
        pairs.sort(key=lambda x: x[0], reverse=True)
        assigned: dict[int, int] = {}
        for _, ti, di in pairs:
            if ti in matched_tracks or di in matched_dets:
                continue
            matched_tracks.add(ti)
            matched_dets.add(di)
            assigned[di] = ti
        out: list[TrackedDetection] = []
        for di, d in enumerate(detections):
            if di in assigned:
                t = tracks[assigned[di]]
                t.x1 = float(d.x1)
                t.y1 = float(d.y1)
                t.x2 = float(d.x2)
                t.y2 = float(d.y2)
                t.last_seen = now
                out.append(TrackedDetection(oid=t.oid, detection=d))
            else:
                prefix = prefix_for_class(d.class_id)
                oid = self._new_id(camera_name, prefix)
                t = _Track(
                    oid=oid,
                    class_id=int(d.class_id),
                    x1=float(d.x1),
                    y1=float(d.y1),
                    x2=float(d.x2),
                    y2=float(d.y2),
                    last_seen=now,
                )
                tracks.append(t)
                out.append(TrackedDetection(oid=oid, detection=d))
        self._tracks[camera_name] = tracks
        return out
