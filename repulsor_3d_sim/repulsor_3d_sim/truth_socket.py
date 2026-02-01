# repulsor_3d_sim/truth_socket.py
from __future__ import annotations
import json
import socket
import struct
import threading
import time
from typing import List

from repulsor_3d_sim.model import FieldVisionObject

class TruthSocketReceiver:
    def __init__(self, host: str, port: int):
        self.host = host
        self.port = int(port)
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._latest: List[FieldVisionObject] = []
        self._t = threading.Thread(target=self._run, name="truth-socket-recv", daemon=True)

    def start(self) -> None:
        self._t.start()

    def stop(self) -> None:
        self._stop.set()
        self._t.join(timeout=0.5)

    def latest(self) -> List[FieldVisionObject]:
        with self._lock:
            return list(self._latest)

    def _recv_exact(self, conn: socket.socket, n: int) -> bytes | None:
        buf = bytearray()
        while len(buf) < n and not self._stop.is_set():
            try:
                chunk = conn.recv(n - len(buf))
            except socket.timeout:
                continue
            except Exception:
                return None
            if not chunk:
                return None
            buf.extend(chunk)
        return bytes(buf)

    def _run(self) -> None:
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((self.host, self.port))
        srv.listen(1)
        srv.settimeout(0.25)

        while not self._stop.is_set():
            try:
                conn, _addr = srv.accept()
            except socket.timeout:
                continue
            except Exception:
                break

            try:
                conn.settimeout(0.25)
                while not self._stop.is_set():
                    header = self._recv_exact(conn, 4)
                    if not header:
                        break
                    size = struct.unpack(">I", header)[0]
                    if size <= 0 or size > 5_000_000:
                        break
                    payload = self._recv_exact(conn, size)
                    if not payload:
                        break
                    try:
                        msg = json.loads(payload.decode("utf-8"))
                    except Exception:
                        continue
                    fuel = msg.get("fuel", [])
                    out: List[FieldVisionObject] = []
                    for i in range(0, len(fuel) - 2, 3):
                        out.append(
                            FieldVisionObject(
                                oid=str(i // 3),
                                typ="fuel",
                                x=float(fuel[i]),
                                y=float(fuel[i + 1]),
                                z=float(fuel[i + 2]),
                                roll=0.0,
                                pitch=0.0,
                                yaw=0.0,
                            )
                        )
                    with self._lock:
                        self._latest = out
            except Exception:
                pass
            finally:
                try:
                    conn.close()
                except Exception:
                    pass

        try:
            srv.close()
        except Exception:
            pass
