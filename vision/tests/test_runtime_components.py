from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from lib.class_map import ClassMap
from lib.publisher import FieldVisionPublisher
from lib.runtime_types import CameraInfoOut, Detection2D, FieldObject
from lib.tracker import DetectionTracker


class _FakeEntry:
    def __init__(self, store: dict[str, object], key: str):
        self._store = store
        self._key = key

    def setDouble(self, value: float):
        self._store[self._key] = float(value)

    def setString(self, value: str):
        self._store[self._key] = str(value)

    def setBoolean(self, value: bool):
        self._store[self._key] = bool(value)


class _FakeTable:
    def __init__(self):
        self.store: dict[str, object] = {}

    def getEntry(self, key: str) -> _FakeEntry:
        return _FakeEntry(self.store, key)


class TestRuntimeComponents(unittest.TestCase):
    def test_class_map_loads_dynamic_ids(self):
        raw = {
            "default": {"type": "unknown", "enabled": False, "oid_prefix": "obj"},
            "classes": {
                "7": {
                    "type": "fuel",
                    "oid_prefix": "fuel",
                    "estimator": "fuel",
                    "ball_radius_m": 0.15,
                    "enabled": True,
                }
            },
        }
        with tempfile.TemporaryDirectory() as td:
            p = Path(td) / "classes.json"
            p.write_text(json.dumps(raw), encoding="utf-8")
            cm = ClassMap.from_file(p)
            c7 = cm.get(7)
            c9 = cm.get(9)
            self.assertEqual(c7.type_name, "fuel")
            self.assertEqual(c7.oid_prefix, "fuel")
            self.assertEqual(c7.estimator, "fuel")
            self.assertTrue(c7.enabled)
            self.assertEqual(c9.type_name, "unknown")
            self.assertFalse(c9.enabled)

    def test_publisher_writes_and_removes_objects(self):
        table = _FakeTable()
        pub = FieldVisionPublisher()
        cam = CameraInfoOut(
            name="cam0",
            x=0.1,
            y=0.0,
            z=0.2,
            yaw_deg=0.0,
            pitch_deg=0.0,
            roll_deg=0.0,
            hfov_deg=95.0,
            vfov_deg=60.0,
            max_range=10.0,
        )
        a = FieldObject(oid="fuel_0", type_name="fuel", x=1.0, y=2.0, z=0.15)
        pub.publish(table, [a], [cam], max_per_tick=10)
        self.assertTrue(table.store["object_fuel_0"])
        self.assertEqual(table.store["object_fuel_0/type"], "fuel")
        self.assertTrue(table.store["camera_cam0"])
        pub.publish(table, [], [cam], max_per_tick=10)
        self.assertFalse(table.store["object_fuel_0"])

    def test_tracker_keeps_id_with_iou(self):
        tracker = DetectionTracker(iou_threshold=0.2, max_age_s=1.0)
        d0 = Detection2D(class_id=0, confidence=0.9, x1=100.0, y1=100.0, x2=200.0, y2=200.0)
        d1 = Detection2D(class_id=0, confidence=0.9, x1=105.0, y1=102.0, x2=204.0, y2=201.0)
        t0 = tracker.update("cam0", [d0], lambda _: "fuel", 1.0)
        t1 = tracker.update("cam0", [d1], lambda _: "fuel", 1.1)
        self.assertEqual(len(t0), 1)
        self.assertEqual(len(t1), 1)
        self.assertEqual(t0[0].oid, t1[0].oid)


if __name__ == "__main__":
    unittest.main()
