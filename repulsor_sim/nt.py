# repulsor_sim/nt.py
import time
from typing import Optional

from ntcore import MultiSubscriber, NetworkTableInstance
from wpimath.geometry import Pose2d, Rotation2d

from repulsor_sim.config import Config

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

class NTClient:
    def __init__(self,cfg: Config):
        self.inst = NetworkTableInstance.getDefault()
        try:
            self.inst.stopServer()
            self.inst.stopClient()
        except Exception:
            pass

        self.inst.startClient4("repulsor_sim")
        self.inst.setServer("localhost", NetworkTableInstance.kDefaultPort4)

        self._ms = MultiSubscriber(self.inst, [cfg.pose_base_path])

        self._pose_prefix = _topic_prefix(cfg.pose_base_path)
        pose_topic = self.inst.getStructTopic(_join_topic(self._pose_prefix, cfg.pose_struct_key), Pose2d)
        self._pose_struct = pose_topic.subscribe(Pose2d(0.0, 0.0, Rotation2d()))

        # self._piece_pub = self.inst.getIntegerTopic("/pieceCount").publish()
        # self._piece_sub = self.inst.getIntegerTopic("/pieceCount").subscribe(0)

        t0 = time.time()
        while not self.inst.isConnected() and time.time() - t0 < 2.0:
            time.sleep(0.02)

    def table(self, path: str):
        return self.inst.getTable(path)

    def flush(self):
        self.inst.flush()

    @staticmethod
    def sleep_dt(dt: float):
        if dt > 0:
            time.sleep(dt)

    def pose(self) -> Optional[Pose2d]:
        p: Pose2d = self._pose_struct.get()
        x = float(p.x)
        y = float(p.y)
        th_deg = float(p.rotation().degrees())
        if x != x or y != y or th_deg != th_deg:
            return None
        return Pose2d(x, y, Rotation2d.fromDegrees(th_deg))

    def set_piece_count(self, v: int) -> None:
        # self._piece_pub.set(int(v))
        pass

    def get_piece_count(self) -> int:
        # return int(self._piece_sub.get())
        return 1