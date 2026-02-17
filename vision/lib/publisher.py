from __future__ import annotations

from typing import Sequence

from lib.runtime_types import CameraInfoOut, FieldObject


class FieldVisionPublisher:
    def __init__(self) -> None:
        self._last_objects: dict[str, FieldObject] = {}
        self._last_cameras: dict[str, CameraInfoOut] = {}

    @staticmethod
    def _obj_changed(a: FieldObject, b: FieldObject) -> bool:
        return (
            a.type_name != b.type_name
            or a.x != b.x
            or a.y != b.y
            or a.z != b.z
            or a.roll != b.roll
            or a.pitch != b.pitch
            or a.yaw != b.yaw
            or a.frame != b.frame
        )

    @staticmethod
    def _cam_changed(a: CameraInfoOut, b: CameraInfoOut) -> bool:
        return (
            a.x != b.x
            or a.y != b.y
            or a.z != b.z
            or a.yaw_deg != b.yaw_deg
            or a.pitch_deg != b.pitch_deg
            or a.roll_deg != b.roll_deg
            or a.hfov_deg != b.hfov_deg
            or a.vfov_deg != b.vfov_deg
            or a.max_range != b.max_range
        )

    def publish(
        self,
        table,
        objects: Sequence[FieldObject],
        cameras: Sequence[CameraInfoOut],
        extrinsics_xyzrpy: tuple[float, float, float, float, float, float] = (
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ),
        max_per_tick: int = 256,
    ) -> None:
        ex, ey, ez, er, ep, eyaw = extrinsics_xyzrpy
        table.getEntry("extrinsics/x").setDouble(float(ex))
        table.getEntry("extrinsics/y").setDouble(float(ey))
        table.getEntry("extrinsics/z").setDouble(float(ez))
        table.getEntry("extrinsics/roll").setDouble(float(er))
        table.getEntry("extrinsics/pitch").setDouble(float(ep))
        table.getEntry("extrinsics/yaw").setDouble(float(eyaw))

        candidates: list[FieldObject] = []
        for o in objects:
            prev = self._last_objects.get(o.oid)
            if prev is None or self._obj_changed(o, prev):
                candidates.append(o)

        current_ids = {o.oid for o in objects}
        for oid in list(self._last_objects.keys()):
            if oid not in current_ids:
                table.getEntry(f"object_{oid}").setBoolean(False)
                del self._last_objects[oid]

        for o in candidates[: max(0, int(max_per_tick))]:
            base = f"object_{o.oid}"
            table.getEntry(base).setBoolean(True)
            table.getEntry(base + "/frame").setString(o.frame)
            table.getEntry(base + "/type").setString(o.type_name)
            table.getEntry(base + "/x").setDouble(float(o.x))
            table.getEntry(base + "/y").setDouble(float(o.y))
            table.getEntry(base + "/z").setDouble(float(o.z))
            table.getEntry(base + "/roll").setDouble(float(o.roll))
            table.getEntry(base + "/pitch").setDouble(float(o.pitch))
            table.getEntry(base + "/yaw").setDouble(float(o.yaw))
            self._last_objects[o.oid] = o

        cam_candidates: list[CameraInfoOut] = []
        for c in cameras:
            prev = self._last_cameras.get(c.name)
            if prev is None or self._cam_changed(c, prev):
                cam_candidates.append(c)

        current_cam_names = {c.name for c in cameras}
        for name in list(self._last_cameras.keys()):
            if name not in current_cam_names:
                table.getEntry(f"camera_{name}").setBoolean(False)
                del self._last_cameras[name]

        for c in cam_candidates:
            base = f"camera_{c.name}"
            table.getEntry(base).setBoolean(True)
            table.getEntry(base + "/x").setDouble(float(c.x))
            table.getEntry(base + "/y").setDouble(float(c.y))
            table.getEntry(base + "/z").setDouble(float(c.z))
            table.getEntry(base + "/yaw_deg").setDouble(float(c.yaw_deg))
            table.getEntry(base + "/pitch_deg").setDouble(float(c.pitch_deg))
            table.getEntry(base + "/roll_deg").setDouble(float(c.roll_deg))
            table.getEntry(base + "/hfov_deg").setDouble(float(c.hfov_deg))
            table.getEntry(base + "/vfov_deg").setDouble(float(c.vfov_deg))
            table.getEntry(base + "/max_range").setDouble(float(c.max_range))
            self._last_cameras[c.name] = c
