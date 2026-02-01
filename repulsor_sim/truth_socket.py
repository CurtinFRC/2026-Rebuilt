# repulsor_sim/truth_socket.py
from __future__ import annotations
import json
import socket
import struct
import time
from typing import Optional, Sequence

from repulsor_sim.types import WorldObject

class TruthSocketSender:
    def __init__(self, host: str, port: int, hz: float = 10.0, decimals: int = 3):
        self.host = host
        self.port = int(port)
        self.period_s = 1.0 / max(1e-6, float(hz))
        self.decimals = max(0, int(decimals))

        self._sock: Optional[socket.socket] = None
        self._next_send = 0.0
        self._next_reconnect = 0.0
        self._reconnect_delay = 0.75

    def _connect(self, now_s: float) -> None:
        if now_s < self._next_reconnect:
            return
        self._next_reconnect = now_s + self._reconnect_delay
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(0.5)
            s.connect((self.host, self.port))
            s.settimeout(None)
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self._sock = s
        except Exception:
            try:
                s.close()
            except Exception:
                pass
            self._sock = None

    def _close(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            except Exception:
                pass
        self._sock = None

    def send(self, now_s: float, objects: Sequence[WorldObject]) -> None:
        if now_s < self._next_send:
            return
        self._next_send = now_s + self.period_s

        if self._sock is None:
            self._connect(now_s)
            if self._sock is None:
                return

        # Flatten positions to reduce JSON overhead.
        if self.decimals > 0:
            rd = self.decimals
            fuel = [round(v, rd) for o in objects for v in (o.x, o.y, o.z)]
        else:
            fuel = [float(v) for o in objects for v in (o.x, o.y, o.z)]

        payload = json.dumps(
            {"t": now_s, "fuel": fuel},
            separators=(",", ":"),
        ).encode("utf-8")
        header = struct.pack(">I", len(payload))
        try:
            self._sock.sendall(header + payload)
        except Exception:
            self._close()
