from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from lib.config import load_runtime_config_raw, save_runtime_camera_usb_indexes


class TestConfigUsbMapping(unittest.TestCase):
    def test_save_runtime_camera_usb_indexes_updates_named_cameras(self):
        raw = {
            "nt": {"server": "localhost"},
            "cameras": [
                {"name": "front", "usb_index": 0},
                {"name": "rear", "usb_index": 1},
            ],
        }
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "runtime.json"
            path.write_text(json.dumps(raw), encoding="utf-8")
            out_path = save_runtime_camera_usb_indexes({"front": 3, "rear": 7}, str(path))
            self.assertEqual(path.resolve(), out_path.resolve())
            updated = load_runtime_config_raw(str(path))
            self.assertEqual(updated["cameras"][0]["usb_index"], 3)
            self.assertEqual(updated["cameras"][1]["usb_index"], 7)

    def test_save_runtime_camera_usb_indexes_rejects_unknown_camera_names(self):
        raw = {
            "nt": {"server": "localhost"},
            "cameras": [
                {"name": "front", "usb_index": 0},
            ],
        }
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "runtime.json"
            path.write_text(json.dumps(raw), encoding="utf-8")
            with self.assertRaises(ValueError):
                save_runtime_camera_usb_indexes({"rear": 2}, str(path))


if __name__ == "__main__":
    unittest.main()

