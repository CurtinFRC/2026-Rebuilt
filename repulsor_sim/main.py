# repulsor_sim/main.py
import os
import time
from repulsor_sim.config import load_config
from repulsor_sim.nt import NTClient
from repulsor_sim.providers import make_provider
from repulsor_sim.publishers.fieldvision import publish_fieldvision
from repulsor_sim.publishers.repulsorvision import publish_repulsorvision


def main():
    cfg = load_config()
    nt = NTClient(cfg)

    fv = nt.table("FieldVision/" + cfg.fieldvision_name)
    rv = nt.table("RepulsorVision")

    provider_name = os.getenv("PROVIDER", "mock")
    provider = make_provider(provider_name)
    provider.reset(cfg)

    dt = 1.0 / max(1e-6, cfg.fps)
    while True:
        t0 = time.time()
        frame = provider.step(t0, nt.pose())
        publish_fieldvision(fv, frame.objects, frame.cameras, nt.pose())
        publish_repulsorvision(rv, frame.obstacles)
        nt.flush()
        NTClient.sleep_dt(dt - (time.time() - t0))


if __name__ == "__main__":
    main()
