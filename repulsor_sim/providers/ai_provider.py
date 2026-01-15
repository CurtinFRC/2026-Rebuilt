# repulsor_sim/providers/ai_provider.py
from __future__ import annotations
from repulsor_sim.providers.base import RepulsorProvider
from repulsor_sim.config import Config
from repulsor_sim.types import ProviderFrame

class AIProvider(RepulsorProvider):
    def reset(self, cfg: Config) -> None:
        self.cfg = cfg

    def step(self, now_s: float) -> ProviderFrame:
        return ProviderFrame(objects=(), obstacles=(), extrinsics_xyzrpy=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
