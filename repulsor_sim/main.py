# repulsor_sim/main.py
import os
import time
from repulsor_sim.config import load_config
from repulsor_sim.nt import NTClient
from repulsor_sim.providers import make_provider
from repulsor_sim.publishers.fieldvision import publish_fieldvision
from repulsor_sim.publishers.fieldvision_truth import publish_fieldvision_truth
from repulsor_sim.publishers.repulsorvision import publish_repulsorvision
from repulsor_sim.truth_socket import TruthSocketSender


def main():
    cfg = load_config()
    nt = NTClient(cfg)

    fv = nt.table("FieldVision/" + cfg.fieldvision_name)
    truth_over_nt = os.getenv("TRUTH_OVER_NT", "0").strip() == "1"
    fvt = nt.table(cfg.fieldvision_truth_path) if truth_over_nt else None
    rv = nt.table("RepulsorVision")

    provider_name = os.getenv("PROVIDER", "mock")
    provider = make_provider(provider_name)
    provider.reset(cfg)

    dt = 1.0 / max(1e-6, cfg.fps)
    truth_max_per_tick = int(os.getenv("TRUTH_MAX_PER_TICK", "12"))
    truth_socket_enabled = os.getenv("TRUTH_SOCKET_ENABLED", "1").strip() != "0"
    truth_socket_host = os.getenv("TRUTH_SOCKET_HOST", "127.0.0.1")
    truth_socket_port = int(os.getenv("TRUTH_SOCKET_PORT", "5809"))
    truth_socket_hz = float(os.getenv("TRUTH_SOCKET_HZ", "10"))
    truth_socket_decimals = int(os.getenv("TRUTH_SOCKET_DECIMALS", "3"))
    truth_sender = None
    if truth_socket_enabled:
        truth_sender = TruthSocketSender(
            host=truth_socket_host,
            port=truth_socket_port,
            hz=truth_socket_hz,
            decimals=truth_socket_decimals,
        )
    while True:
        t0 = time.time()
        frame = provider.step(t0, nt.pose())
        publish_fieldvision(fv, frame.objects, frame.cameras, nt.pose())
        if truth_over_nt and fvt is not None:
            publish_fieldvision_truth(fvt, frame.truth_objects, max_per_tick=truth_max_per_tick)
        publish_repulsorvision(rv, frame.obstacles)
        if truth_sender is not None:
            truth_sender.send(t0, frame.truth_objects)
        nt.flush()
        NTClient.sleep_dt(dt - (time.time() - t0))


if __name__ == "__main__":
    main()
