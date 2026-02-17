from __future__ import annotations

import time
from typing import Optional

from ntcore import MultiSubscriber, NetworkTableInstance
from wpimath.geometry import Pose2d, Rotation2d


def _norm_table_path(path: str) -> str:
    pp = (path or "").strip()
    while pp.startswith("/"):
        pp = pp[1:]
    while pp.endswith("/"):
        pp = pp[:-1]
    return pp


def _topic_prefix(table_path: str) -> str:
    tp = _norm_table_path(table_path)
    return "/" + tp if tp else "/"


def _join_topic(prefix: str, leaf: str) -> str:
    p = prefix.rstrip("/")
    l = leaf.lstrip("/")
    if not p:
        return "/" + l
    return p + "/" + l


class VisionNTClient:
    def __init__(
        self,
        server: str,
        port: int,
        client_name: str,
        pose_base_path: str,
        pose_struct_key: str,
        connect_timeout_s: float = 2.0,
    ):
        self.inst = NetworkTableInstance.getDefault()
        try:
            self.inst.stopServer()
            self.inst.stopClient()
        except Exception:
            pass
        self.inst.startClient4(client_name)
        self.inst.setServer(server, int(port))
        self._pose_sub = None
        self._ms = MultiSubscriber(self.inst, [pose_base_path])
        pose_prefix = _topic_prefix(pose_base_path)
        topic = self.inst.getStructTopic(_join_topic(pose_prefix, pose_struct_key), Pose2d)
        self._pose_sub = topic.subscribe(Pose2d(0.0, 0.0, Rotation2d()))
        t0 = time.time()
        while not self.inst.isConnected() and (time.time() - t0) < max(0.0, float(connect_timeout_s)):
            time.sleep(0.02)

    def table(self, path: str):
        return self.inst.getTable(path)

    def flush(self) -> None:
        self.inst.flush()

    @staticmethod
    def sleep_dt(dt: float) -> None:
        if dt > 0.0:
            time.sleep(dt)

    def pose(self) -> Optional[Pose2d]:
        if self._pose_sub is None:
            return None
        p: Pose2d = self._pose_sub.get()
        x = float(p.x)
        y = float(p.y)
        th_deg = float(p.rotation().degrees())
        if x != x or y != y or th_deg != th_deg:
            return None
        return Pose2d(x, y, Rotation2d.fromDegrees(th_deg))
