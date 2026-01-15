# repulsor_sim/providers/__init__.py
from repulsor_sim.providers.base import RepulsorProvider
from repulsor_sim.providers.mock_provider import MockProvider
from repulsor_sim.providers.ai_provider import AIProvider

def make_provider(name: str) -> RepulsorProvider:
    n = (name or "").strip().lower()
    if n in ("ai", "aiprovider"):
        return AIProvider()
    return MockProvider()
