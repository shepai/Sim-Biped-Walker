"""RL agents for the walker. All share :class:`BaseRLAgent`."""

import json
from pathlib import Path

from .base import META_FILE, BaseRLAgent
from .es import ESAgent

AGENTS = {"es": ESAgent}
try:  # optional dependency
    from .ppo import PPOAgent

    AGENTS["ppo"] = PPOAgent
except ImportError:  # pragma: no cover
    PPOAgent = None


def make_agent(algo: str, **kwargs) -> BaseRLAgent:
    if algo == "ppo" and "ppo" not in AGENTS:
        raise ImportError("PPO needs stable-baselines3 + torch: pip install -e '.[ppo]'")
    return AGENTS[algo](**kwargs)


def load_agent(directory, **overrides) -> BaseRLAgent:
    """Load any saved agent, picking the class from its ``agent.json``."""
    algo = json.loads((Path(directory) / META_FILE).read_text())["algo"]
    if algo == "ppo" and "ppo" not in AGENTS:
        raise ImportError("PPO needs stable-baselines3 + torch: pip install -e '.[ppo]'")
    return AGENTS[algo].load(directory, **overrides)


__all__ = ["BaseRLAgent", "ESAgent", "PPOAgent", "make_agent", "load_agent", "AGENTS"]
