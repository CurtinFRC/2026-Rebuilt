# repulsor_3d_sim/nt4.py
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Set, Tuple

from ntcore import MultiSubscriber, NetworkTableInstance
from wpimath.geometry import Pose2d, Rotation2d

from repulsor_3d_sim.config import ViewerConfig
from repulsor_3d_sim.model import FieldVisionObject, Pose2D, RepulsorVisionObstacle, WorldSnapshot


def _norm_table_path(p: str) -> str:
    pp = (p or "").strip()
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


def _extract_id_from_topic(topic_name: str, table_prefix: str, prefix: str) -> Optional[str]:
    base = table_prefix.rstrip("/")
    if not topic_name.startswith(base + "/"):
        return None
    rel = topic_name[len(base) + 1 :]
    head = rel.split("/", 1)[0]
    if not head.startswith(prefix):
        return None
    return head[len(prefix) :]


@dataclass
class _FVSubs:
    alive: any
    typ: any
    x: any
    y: any
    z: any
    roll: any
    pitch: any
    yaw: any


@dataclass
class _RVSubs:
    alive: any
    kind: any
    x: any
    y: any
    sx: any
    sy: any


class NT4Reader:
    def __init__(self, cfg: ViewerConfig):
        self.cfg = cfg
        self.inst = NetworkTableInstance.getDefault()
        self.inst.stopClient()
        self.inst.startClient4(cfg.nt_client_name)
        self.inst.setServerTeam(4788)
        self.inst.setServer(cfg.nt_server, NetworkTableInstance.kDefaultPort4)
        self._ms = MultiSubscriber(self.inst, ["/"])

        self._fv_prefix = _topic_prefix(cfg.fieldvision_path)
        self._rv_prefix = _topic_prefix(cfg.repulsorvision_path)

        self._pose_prefix = _topic_prefix(cfg.pose_base_path)
        pose_topic = self.inst.getStructTopic(_join_topic(self._pose_prefix, cfg.pose_struct_key), Pose2d)
        self._pose_struct = pose_topic.subscribe(Pose2d(0.0, 0.0, Rotation2d()))

        self._ex = self.inst.getDoubleTopic(_join_topic(self._fv_prefix, "extrinsics/x")).subscribe(0.0)
        self._ey = self.inst.getDoubleTopic(_join_topic(self._fv_prefix, "extrinsics/y")).subscribe(0.0)
        self._ez = self.inst.getDoubleTopic(_join_topic(self._fv_prefix, "extrinsics/z")).subscribe(0.0)
        self._er = self.inst.getDoubleTopic(_join_topic(self._fv_prefix, "extrinsics/roll")).subscribe(0.0)
        self._ep = self.inst.getDoubleTopic(_join_topic(self._fv_prefix, "extrinsics/pitch")).subscribe(0.0)
        self._eyaw = self.inst.getDoubleTopic(_join_topic(self._fv_prefix, "extrinsics/yaw")).subscribe(0.0)

        self._fv_subs: Dict[str, _FVSubs] = {}
        self._rv_subs: Dict[str, _RVSubs] = {}

        self._connected_at: Optional[float] = None
        self._last_connect_check = 0.0

        self._last_discovery = 0.0
        self._discovery_period_s = 0.25

    def is_connected(self) -> bool:
        now = time.time()
        if now - self._last_connect_check > 0.25:
            self._last_connect_check = now
            if self.inst.isConnected():
                if self._connected_at is None:
                    self._connected_at = now
            else:
                self._connected_at = None
        return self.inst.isConnected()

    def _discover_dynamic_topics(self):
        now = time.time()
        if now - self._last_discovery < self._discovery_period_s:
            return
        self._last_discovery = now

        fv_topics = self.inst.getTopics(self._fv_prefix)
        rv_topics = self.inst.getTopics(self._rv_prefix)

        fv_ids: Set[str] = set()
        for t in fv_topics:
            tid = _extract_id_from_topic(t.getName(), self._fv_prefix, "object_")
            if tid is not None:
                fv_ids.add(tid)

        rv_ids: Set[str] = set()
        for t in rv_topics:
            tid = _extract_id_from_topic(t.getName(), self._rv_prefix, "obs_")
            if tid is not None:
                rv_ids.add(tid)

        for oid in fv_ids:
            if oid in self._fv_subs:
                continue
            base = f"object_{oid}"
            base_topic = _join_topic(self._fv_prefix, base)
            self._fv_subs[oid] = _FVSubs(
                alive=self.inst.getBooleanTopic(base_topic).subscribe(False),
                typ=self.inst.getStringTopic(_join_topic(self._fv_prefix, base + "/type")).subscribe(""),
                x=self.inst.getDoubleTopic(_join_topic(self._fv_prefix, base + "/x")).subscribe(0.0),
                y=self.inst.getDoubleTopic(_join_topic(self._fv_prefix, base + "/y")).subscribe(0.0),
                z=self.inst.getDoubleTopic(_join_topic(self._fv_prefix, base + "/z")).subscribe(0.0),
                roll=self.inst.getDoubleTopic(_join_topic(self._fv_prefix, base + "/roll")).subscribe(0.0),
                pitch=self.inst.getDoubleTopic(_join_topic(self._fv_prefix, base + "/pitch")).subscribe(0.0),
                yaw=self.inst.getDoubleTopic(_join_topic(self._fv_prefix, base + "/yaw")).subscribe(0.0),
            )

        for oid in rv_ids:
            if oid in self._rv_subs:
                continue
            base = f"obs_{oid}"
            base_topic = _join_topic(self._rv_prefix, base)
            self._rv_subs[oid] = _RVSubs(
                alive=self.inst.getBooleanTopic(base_topic).subscribe(False),
                kind=self.inst.getStringTopic(_join_topic(self._rv_prefix, base + "/kind")).subscribe(""),
                x=self.inst.getDoubleTopic(_join_topic(self._rv_prefix, base + "/x")).subscribe(0.0),
                y=self.inst.getDoubleTopic(_join_topic(self._rv_prefix, base + "/y")).subscribe(0.0),
                sx=self.inst.getDoubleTopic(_join_topic(self._rv_prefix, base + "/size_x")).subscribe(0.0),
                sy=self.inst.getDoubleTopic(_join_topic(self._rv_prefix, base + "/size_y")).subscribe(0.0),
            )

    def read_pose2d(self) -> Optional[Pose2d]:
        p: Pose2d = self._pose_struct.get()
        x = float(p.x)
        y = float(p.y)
        th_deg = float(p.rotation().degrees())
        if x != x or y != y or th_deg != th_deg:
            return None
        return Pose2d(x, y, Rotation2d.fromDegrees(th_deg))

    def read_extrinsics(self) -> Tuple[float, float, float, float, float, float]:
        return (
            float(self._ex.get()),
            float(self._ey.get()),
            float(self._ez.get()),
            float(self._er.get()),
            float(self._ep.get()),
            float(self._eyaw.get()),
        )

    def read_fieldvision(self) -> List[FieldVisionObject]:
        self._discover_dynamic_topics()
        out: List[FieldVisionObject] = []
        for oid, s in sorted(self._fv_subs.items(), key=lambda kv: kv[0]):
            if not bool(s.alive.get()):
                continue
            out.append(
                FieldVisionObject(
                    str(oid),
                    str(s.typ.get()),
                    float(s.x.get()),
                    float(s.y.get()),
                    float(s.z.get()),
                    float(s.roll.get()),
                    float(s.pitch.get()),
                    float(s.yaw.get()),
                )
            )
        return out

    def read_repulsorvision(self) -> List[RepulsorVisionObstacle]:
        self._discover_dynamic_topics()
        out: List[RepulsorVisionObstacle] = []
        for oid, s in sorted(self._rv_subs.items(), key=lambda kv: kv[0]):
            if not bool(s.alive.get()):
                continue
            out.append(
                RepulsorVisionObstacle(
                    str(oid),
                    str(s.kind.get()),
                    float(s.x.get()),
                    float(s.y.get()),
                    float(s.sx.get()),
                    float(s.sy.get()),
                )
            )
        return out

    def snapshot(self) -> WorldSnapshot:
        fv = self.read_fieldvision()
        rv = self.read_repulsorvision()
        pose = self.read_pose2d()
        ex = self.read_extrinsics()
        return WorldSnapshot(fv, rv, pose, ex)
