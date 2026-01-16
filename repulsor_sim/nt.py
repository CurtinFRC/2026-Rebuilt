# repulsor_sim/nt.py
import time
from ntcore import NetworkTableInstance


class NTClient:
    def __init__(self, server: str):
        self.inst = NetworkTableInstance.getDefault()
        self.inst.startClient4("repulsor_sim")
        self.inst.setServer(server, NetworkTableInstance.kDefaultPort4)
        self.server = server

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

    def pose(self):
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
