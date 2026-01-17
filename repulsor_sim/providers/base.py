# repulsor_sim/providers/base.py
from __future__ import annotations
from abc import ABC, abstractmethod
from repulsor_sim.config import Config
from repulsor_sim.types import ProviderFrame
from wpimath.geometry import Pose2d, Rotation2d

class RepulsorProvider(ABC):
    @abstractmethod
    def reset(self, cfg: Config) -> None:
        ...

    @abstractmethod
    def step(self, now_s: float, pose: Pose2d) -> ProviderFrame:
        ...
