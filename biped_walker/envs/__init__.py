from .walker_env import TASKS, BipedWalkerEnv, CommandConfig, RewardConfig

__all__ = ["BipedWalkerEnv", "CommandConfig", "RewardConfig", "TASKS", "EnvFactory", "make_env"]

DEFAULT_ENV_ID = "SimBiped-v0"


def make_env(env_id: str = DEFAULT_ENV_ID, render_mode=None, max_episode_steps=None, **kwargs):
    """``gym.make`` for the walker envs (imports the package so the ids are registered)."""
    import gymnasium as gym

    import biped_walker  # noqa: F401  (registers the env ids)

    if max_episode_steps is not None:
        kwargs["max_episode_steps"] = max_episode_steps
    return gym.make(env_id, render_mode=render_mode, **kwargs)


class EnvFactory:
    """Picklable env constructor, usable with subprocess vector envs."""

    def __init__(self, env_id: str = DEFAULT_ENV_ID, **env_kwargs):
        self.env_id = env_id
        self.env_kwargs = env_kwargs

    def __call__(self, render_mode=None, **overrides):
        return make_env(self.env_id, render_mode=render_mode, **{**self.env_kwargs, **overrides})
