"""MuJoCo simulation, Gymnasium environments and RL agents for the 4-DoF biped walker."""

from gymnasium.envs.registration import register, registry

from .model import (FOOT_NAMES, JOINT_NAMES, Biped10Config, ModelConfig, TactileConfig, Walker4Config,
                    build_mjcf, write_mjcf)
from .robots import ROBOTS, get_robot

__version__ = "0.3.0"

_ENVS = {
    "SimBiped-v0": {"robot": "walker4", "task": "mixed"},
    "SimBiped-Forward-v0": {"robot": "walker4", "task": "forward"},
    "SimBiped-Turn-v0": {"robot": "walker4", "task": "turn"},
    "SimBiped10-v0": {"robot": "biped10", "task": "mixed"},
    "SimBiped10-Forward-v0": {"robot": "biped10", "task": "forward"},
    "SimBiped10-Turn-v0": {"robot": "biped10", "task": "turn"},
}
ENV_IDS = {(kw["robot"], kw["task"]): env_id for env_id, kw in _ENVS.items()}
for _id, _kw in _ENVS.items():
    if _id not in registry:
        register(id=_id, entry_point="biped_walker.envs.walker_env:BipedWalkerEnv",
                 max_episode_steps=1000, kwargs=_kw)


def __getattr__(name):
    # Lazy imports so `import biped_walker` stays light (no mujoco needed for build_mjcf).
    if name == "Biped":
        from .robot import Biped
        return Biped
    if name in ("BipedWalkerEnv", "EnvFactory", "make_env", "CommandConfig", "RewardConfig"):
        from . import envs
        return getattr(envs, name)
    raise AttributeError(name)


__all__ = ["Biped", "BipedWalkerEnv", "EnvFactory", "make_env", "ModelConfig", "Walker4Config",
           "Biped10Config", "ROBOTS", "ENV_IDS", "get_robot", "TactileConfig",
           "CommandConfig", "RewardConfig", "build_mjcf", "write_mjcf", "JOINT_NAMES", "FOOT_NAMES"]
