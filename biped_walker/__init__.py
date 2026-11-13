"""MuJoCo simulation, Gymnasium environments and RL agents for the 4-DoF biped walker."""

from gymnasium.envs.registration import register, registry

from .model import FOOT_NAMES, JOINT_NAMES, ModelConfig, TactileConfig, build_mjcf, write_mjcf

__version__ = "0.2.0"

_ENVS = {
    "SimBiped-v0": {"task": "mixed"},
    "SimBiped-Forward-v0": {"task": "forward"},
    "SimBiped-Turn-v0": {"task": "turn"},
}
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


__all__ = ["Biped", "BipedWalkerEnv", "EnvFactory", "make_env", "ModelConfig", "TactileConfig",
           "CommandConfig", "RewardConfig", "build_mjcf", "write_mjcf", "JOINT_NAMES", "FOOT_NAMES"]
