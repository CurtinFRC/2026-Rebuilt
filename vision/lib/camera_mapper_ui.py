from __future__ import annotations

import json
import threading
from dataclasses import dataclass
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np

from lib.config import (
    load_runtime_config_raw,
    resolve_runtime_config_path,
    save_runtime_camera_usb_indexes,
)


@dataclass
class CameraRow:
    name: str
    usb_index: int
    width: int
    height: int


@dataclass
class MapperState:
    config_path: Path
    scan_max_index: int
    cameras: list[CameraRow]
    available_indexes: list[int]
    lock: threading.Lock


def _read_camera_rows(config_path: Path) -> list[CameraRow]:
    raw = load_runtime_config_raw(str(config_path))
    cameras_raw = raw.get("cameras", [])
    if not isinstance(cameras_raw, list) or not cameras_raw:
        raise ValueError("cameras must be a non-empty list")
    out: list[CameraRow] = []
    for cam in cameras_raw:
        if not isinstance(cam, dict):
            raise ValueError("camera entry must be an object")
        name = str(cam.get("name", "")).strip()
        if not name:
            raise ValueError("camera name is required")
        out.append(
            CameraRow(
                name=name,
                usb_index=int(cam.get("usb_index", 0)),
                width=int(cam.get("width", 1280)),
                height=int(cam.get("height", 720)),
            )
        )
    return out


def _open_capture(index: int, width: int, height: int) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(int(index), cv2.CAP_DSHOW)
    if not cap.isOpened():
        cap = cv2.VideoCapture(int(index))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(max(1, int(width))))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(max(1, int(height))))
    return cap


def _read_frame(index: int, width: int, height: int) -> np.ndarray | None:
    cap = _open_capture(index, width, height)
    if not cap.isOpened():
        return None
    try:
        frame: np.ndarray | None = None
        for _ in range(3):
            ok, candidate = cap.read()
            if ok and candidate is not None and candidate.size > 0:
                frame = candidate
        return frame
    finally:
        cap.release()


def _encode_frame_jpeg(frame: np.ndarray) -> bytes:
    ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
    if not ok:
        raise RuntimeError("failed to encode JPEG frame")
    return bytes(encoded.tobytes())


def _error_frame(index: int, width: int, height: int) -> np.ndarray:
    out = np.zeros((max(80, height), max(120, width), 3), dtype=np.uint8)
    msg = f"USB {index}: no frame"
    cv2.putText(
        out,
        msg,
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2,
        lineType=cv2.LINE_AA,
    )
    return out


def _scan_available_indexes(max_index: int) -> list[int]:
    available: list[int] = []
    for idx in range(0, int(max_index) + 1):
        frame = _read_frame(index=idx, width=640, height=360)
        if frame is not None:
            available.append(idx)
    return available


def _state_payload(state: MapperState) -> dict[str, Any]:
    with state.lock:
        return {
            "config_path": str(state.config_path),
            "scan_max_index": int(state.scan_max_index),
            "available_indexes": list(state.available_indexes),
            "cameras": [
                {
                    "name": c.name,
                    "usb_index": int(c.usb_index),
                    "width": int(c.width),
                    "height": int(c.height),
                }
                for c in state.cameras
            ],
        }


def _render_html() -> str:
    return """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>Vision Camera Mapper</title>
  <style>
    :root {
      --bg: #0a1920;
      --panel: #102c37;
      --muted: #8fb5c4;
      --text: #e8f6ff;
      --accent: #ffd166;
      --ok: #06d6a0;
      --bad: #ef476f;
      --line: #1f4a5a;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      padding: 18px;
      font-family: "Segoe UI", Tahoma, sans-serif;
      color: var(--text);
      background:
        radial-gradient(circle at 20% 0%, rgba(255, 209, 102, 0.09), transparent 36%),
        linear-gradient(165deg, #05131a, var(--bg) 48%, #041116);
    }
    .wrap { max-width: 1300px; margin: 0 auto; }
    h1 { margin: 0 0 8px 0; font-size: 26px; }
    p, small { color: var(--muted); margin: 0; }
    .toolbar {
      margin-top: 14px;
      display: flex;
      gap: 8px;
      flex-wrap: wrap;
      align-items: center;
    }
    button {
      border: 1px solid var(--line);
      background: #0d3541;
      color: var(--text);
      border-radius: 10px;
      padding: 8px 12px;
      cursor: pointer;
      font-weight: 600;
    }
    button:hover { filter: brightness(1.12); }
    .primary { background: #11586b; border-color: #267a90; }
    .ok { color: var(--ok); }
    .bad { color: var(--bad); }
    .panel {
      margin-top: 14px;
      border: 1px solid var(--line);
      border-radius: 14px;
      background: linear-gradient(180deg, rgba(16,44,55,0.95), rgba(9,30,38,0.95));
      padding: 12px;
    }
    .grid {
      display: grid;
      gap: 10px;
      grid-template-columns: repeat(auto-fill, minmax(280px, 1fr));
    }
    .card {
      border: 1px solid var(--line);
      border-radius: 10px;
      padding: 8px;
      background: rgba(10, 28, 35, 0.92);
    }
    .row {
      display: flex;
      align-items: center;
      gap: 8px;
      flex-wrap: wrap;
      margin-bottom: 6px;
    }
    .label {
      font-size: 14px;
      font-weight: 700;
      color: var(--accent);
      min-width: 68px;
    }
    .camname {
      font-size: 15px;
      font-weight: 700;
      margin-right: auto;
    }
    select, input {
      border-radius: 8px;
      border: 1px solid var(--line);
      background: #0a222c;
      color: var(--text);
      padding: 6px 8px;
    }
    img {
      width: 100%;
      height: auto;
      display: block;
      border-radius: 8px;
      border: 1px solid #1f4350;
      background: #000;
      min-height: 150px;
      object-fit: contain;
    }
    code {
      color: var(--accent);
      font-size: 12px;
      word-break: break-word;
    }
  </style>
</head>
<body>
  <div class="wrap">
    <h1>Vision USB Camera Mapper</h1>
    <p>Preview USB indexes, map them to configured camera names, then save to runtime config.</p>
    <div class="toolbar">
      <button id="rescanBtn">Rescan USB Indexes</button>
      <button id="saveBtn" class="primary">Save Mapping To Config</button>
      <small id="status"></small>
    </div>
    <div class="panel">
      <div><strong>Config</strong>: <code id="configPath"></code></div>
      <div><strong>Detected USB indexes</strong>: <span id="detectedList"></span></div>
    </div>
    <div class="panel">
      <div class="label">Detected Streams</div>
      <div class="grid" id="detectedGrid"></div>
    </div>
    <div class="panel">
      <div class="label">Camera Mapping</div>
      <div class="grid" id="cameraGrid"></div>
    </div>
  </div>
  <script>
    const state = {
      payload: null,
      detectedImgs: [],
      cameraImgs: [],
    };

    const statusEl = document.getElementById("status");
    const configPathEl = document.getElementById("configPath");
    const detectedListEl = document.getElementById("detectedList");
    const detectedGridEl = document.getElementById("detectedGrid");
    const cameraGridEl = document.getElementById("cameraGrid");

    function setStatus(msg, isError=false) {
      statusEl.textContent = msg;
      statusEl.className = isError ? "bad" : "ok";
    }

    async function fetchState() {
      const res = await fetch("/api/state");
      if (!res.ok) throw new Error("failed to fetch state");
      return await res.json();
    }

    function optionList(available, selected) {
      const set = new Set(available);
      set.add(Number(selected));
      return Array.from(set).sort((a,b)=>a-b)
        .map(v => `<option value="${v}" ${Number(v)===Number(selected) ? "selected" : ""}>${v}</option>`)
        .join("");
    }

    function render(payload) {
      state.payload = payload;
      state.detectedImgs = [];
      state.cameraImgs = [];
      configPathEl.textContent = payload.config_path;
      detectedListEl.textContent = payload.available_indexes.length ? payload.available_indexes.join(", ") : "(none)";

      detectedGridEl.innerHTML = "";
      payload.available_indexes.forEach((idx) => {
        const card = document.createElement("div");
        card.className = "card";
        card.innerHTML = `
          <div class="row"><span class="camname">USB ${idx}</span></div>
          <img data-index="${idx}" alt="usb-${idx}" />
        `;
        const img = card.querySelector("img");
        state.detectedImgs.push({img, index: Number(idx)});
        detectedGridEl.appendChild(card);
      });

      cameraGridEl.innerHTML = "";
      payload.cameras.forEach((cam) => {
        const card = document.createElement("div");
        card.className = "card";
        card.dataset.cameraName = cam.name;
        card.innerHTML = `
          <div class="row">
            <span class="camname">${cam.name}</span>
            <span>USB</span>
            <select class="indexSelect">${optionList(payload.available_indexes, cam.usb_index)}</select>
          </div>
          <img class="cameraPreview" alt="camera-${cam.name}" />
        `;
        const img = card.querySelector(".cameraPreview");
        const select = card.querySelector(".indexSelect");
        state.cameraImgs.push({img, select});
        cameraGridEl.appendChild(card);
      });

      tickImages();
    }

    function tickImages() {
      const stamp = Date.now();
      for (const entry of state.detectedImgs) {
        entry.img.src = `/frame.jpg?index=${entry.index}&width=400&height=240&t=${stamp}`;
      }
      for (const entry of state.cameraImgs) {
        const idx = Number(entry.select.value);
        entry.img.src = `/frame.jpg?index=${idx}&width=640&height=360&t=${stamp}`;
      }
    }

    function collectMappings() {
      const out = {};
      for (const card of cameraGridEl.querySelectorAll(".card")) {
        const camName = card.dataset.cameraName;
        const select = card.querySelector(".indexSelect");
        out[camName] = Number(select.value);
      }
      return out;
    }

    async function doSave() {
      const mappings = collectMappings();
      const res = await fetch("/api/save", {
        method: "POST",
        headers: {"Content-Type": "application/json"},
        body: JSON.stringify({mappings}),
      });
      if (!res.ok) {
        throw new Error(await res.text());
      }
      const payload = await res.json();
      render(payload);
      setStatus("Saved camera USB mapping to config.");
    }

    async function doRescan() {
      const res = await fetch("/api/rescan", {method: "POST"});
      if (!res.ok) throw new Error(await res.text());
      const payload = await res.json();
      render(payload);
      setStatus("USB index scan complete.");
    }

    document.getElementById("saveBtn").addEventListener("click", () => {
      doSave().catch((err) => setStatus(String(err), true));
    });
    document.getElementById("rescanBtn").addEventListener("click", () => {
      doRescan().catch((err) => setStatus(String(err), true));
    });

    cameraGridEl.addEventListener("change", (ev) => {
      if (ev.target && ev.target.classList.contains("indexSelect")) {
        tickImages();
      }
    });

    setInterval(tickImages, 750);

    fetchState()
      .then((payload) => {
        render(payload);
        setStatus("Ready.");
      })
      .catch((err) => setStatus(String(err), true));
  </script>
</body>
</html>
"""


def run_camera_mapper_ui(
    config_path: str | None = None,
    host: str = "0.0.0.0",
    port: int = 5808,
    scan_max_index: int = 8,
) -> None:
    resolved_config = resolve_runtime_config_path(config_path)
    state = MapperState(
        config_path=resolved_config,
        scan_max_index=int(scan_max_index),
        cameras=_read_camera_rows(resolved_config),
        available_indexes=_scan_available_indexes(int(scan_max_index)),
        lock=threading.Lock(),
    )
    html = _render_html().encode("utf-8")

    class Handler(BaseHTTPRequestHandler):
        def _send_json(self, payload: dict[str, Any], status: int = 200) -> None:
            body = json.dumps(payload).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _send_text(self, text: str, status: int = 200) -> None:
            body = text.encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "text/plain; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _send_jpeg(self, jpeg: bytes) -> None:
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(jpeg)))
            self.end_headers()
            self.wfile.write(jpeg)

        def do_GET(self) -> None:  # noqa: N802
            parsed = urlparse(self.path)
            if parsed.path in ("/", "/index.html"):
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(html)))
                self.end_headers()
                self.wfile.write(html)
                return
            if parsed.path == "/api/state":
                self._send_json(_state_payload(state))
                return
            if parsed.path == "/frame.jpg":
                q = parse_qs(parsed.query)
                index = int(q.get("index", ["0"])[0])
                width = int(q.get("width", ["640"])[0])
                height = int(q.get("height", ["360"])[0])
                frame = _read_frame(index=index, width=width, height=height)
                if frame is None:
                    frame = _error_frame(index=index, width=width, height=height)
                self._send_jpeg(_encode_frame_jpeg(frame))
                return
            self._send_text("not found", status=404)

        def do_POST(self) -> None:  # noqa: N802
            parsed = urlparse(self.path)
            if parsed.path == "/api/rescan":
                with state.lock:
                    state.available_indexes = _scan_available_indexes(state.scan_max_index)
                self._send_json(_state_payload(state))
                return
            if parsed.path == "/api/save":
                try:
                    length = int(self.headers.get("Content-Length", "0"))
                    raw_body = self.rfile.read(length)
                    payload = json.loads(raw_body.decode("utf-8"))
                    mappings_raw = payload.get("mappings", {})
                    if not isinstance(mappings_raw, dict):
                        raise ValueError("mappings must be an object")
                    mappings = {str(k): int(v) for k, v in mappings_raw.items()}
                    save_runtime_camera_usb_indexes(mappings, str(state.config_path))
                    with state.lock:
                        state.cameras = _read_camera_rows(state.config_path)
                        state.available_indexes = _scan_available_indexes(state.scan_max_index)
                    self._send_json(_state_payload(state))
                except Exception as exc:
                    self._send_text(str(exc), status=400)
                return
            self._send_text("not found", status=404)

    server = ThreadingHTTPServer((host, int(port)), Handler)
    print(f"[camera-mapper] using config: {resolved_config}")
    print(f"[camera-mapper] web ui: http://{host}:{int(port)}/")
    print("[camera-mapper] press Ctrl+C to stop")
    try:
        server.serve_forever()
    finally:
        server.server_close()

