"""PPO agent built on Stable-Baselines3 (``pip install biped-walker[ppo]``)."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np

from .base import BaseRLAgent

DEFAULT_PPO_KWARGS: Dict[str, Any] = {
    "n_steps": 512,          # per env -> 4096 samples per update with 8 envs
    "batch_size": 512,
    "n_epochs": 10,
    "learning_rate": 3e-4,
    "gamma": 0.99,
    "gae_lambda": 0.95,
    "clip_range": 0.2,
    "ent_coef": 0.0,
    "max_grad_norm": 0.5,
}


def _require_sb3():
    try:
        import stable_baselines3  # noqa: F401
    except ImportError as e:  # pragma: no cover
        raise ImportError("PPOAgent needs stable-baselines3 and torch: pip install -e '.[ppo]'") from e


class PPOAgent(BaseRLAgent):
    """Proximal Policy Optimisation with observation/reward normalisation.

    Parameters
    ----------
    n_envs: parallel environments (subprocesses when > 1).
    net_arch: hidden layers of both the policy and value networks.
    log_std_init: initial log std of the Gaussian policy (small = gentle servos).
    ppo_kwargs: overrides for :data:`DEFAULT_PPO_KWARGS`.
    device: torch device ("auto", "cpu", "cuda").
    """

    algo = "ppo"

    def __init__(self, env_id: str = "SimBiped-v0", env_kwargs: Optional[Dict[str, Any]] = None,
                 seed: Optional[int] = 0, n_envs: int = 8, net_arch: List[int] = (128, 128),
                 log_std_init: float = -1.0, ppo_kwargs: Optional[Dict[str, Any]] = None,
                 device: str = "auto"):
        _require_sb3()
        super().__init__(env_id, env_kwargs, seed)
        self.n_envs = int(n_envs)
        self.net_arch = list(net_arch)
        self.log_std_init = float(log_std_init)
        self.ppo_kwargs = {**DEFAULT_PPO_KWARGS, **(ppo_kwargs or {})}
        self.device = device
        self.model = None
        self.vecnorm = None
        self._train_env_ready = False

    def hyperparameters(self) -> Dict[str, Any]:
        return {"n_envs": self.n_envs, "net_arch": self.net_arch, "log_std_init": self.log_std_init,
                "ppo_kwargs": self.ppo_kwargs, "device": self.device}

    # ------------------------------------------------------------------ envs
    def _make_vec_env(self, n_envs: int, training: bool):
        from stable_baselines3.common.env_util import make_vec_env
        from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv, VecNormalize

        venv = make_vec_env(self.env_factory, n_envs=n_envs, seed=self.seed,
                            vec_env_cls=SubprocVecEnv if n_envs > 1 else DummyVecEnv)
        vn = VecNormalize(venv, norm_obs=True, norm_reward=training, clip_obs=10.0,
                          gamma=self.ppo_kwargs["gamma"], training=training)
        if self.vecnorm is not None:  # carry over statistics
            vn.obs_rms = self.vecnorm.obs_rms
            vn.ret_rms = self.vecnorm.ret_rms
        return vn

    # -------------------------------------------------------------- training
    def train(self, total_timesteps: int, log_dir: Optional[str] = None, checkpoint_every: int = 200_000,
              progress_bar: bool = False, **learn_kwargs) -> "PPOAgent":
        import torch.nn as nn
        from stable_baselines3 import PPO
        from stable_baselines3.common.callbacks import BaseCallback

        if not self._train_env_ready:
            old = self.vecnorm
            self.vecnorm = self._make_vec_env(self.n_envs, training=True)
            if old is not None:
                old.close()
            self._train_env_ready = True

        if self.model is None:
            tb = None
            if log_dir is not None:
                try:
                    import tensorboard  # noqa: F401
                    tb = str(Path(log_dir) / "tb")
                except ImportError:
                    pass
            self.model = PPO(
                "MlpPolicy", self.vecnorm, seed=self.seed, device=self.device, verbose=1,
                tensorboard_log=tb,
                policy_kwargs={"net_arch": {"pi": self.net_arch, "vf": self.net_arch},
                               "activation_fn": nn.Tanh, "log_std_init": self.log_std_init},
                **self.ppo_kwargs,
            )
        elif self.model.n_envs != self.vecnorm.num_envs:
            # A loaded model is sized for 1 env: reload it onto the training envs.
            import tempfile

            with tempfile.TemporaryDirectory() as td:
                self.model.save(Path(td) / "m.zip")
                self.model = PPO.load(Path(td) / "m.zip", env=self.vecnorm, device=self.device)
        else:
            self.model.set_env(self.vecnorm)

        agent = self

        class _Checkpoint(BaseCallback):
            def __init__(self):
                super().__init__()
                self.next = checkpoint_every

            def _on_step(self) -> bool:
                if log_dir is not None and checkpoint_every and self.num_timesteps >= self.next:
                    self.next += checkpoint_every
                    agent.save(Path(log_dir) / "checkpoints" / f"step_{self.num_timesteps}")
                return True

            def _on_rollout_end(self) -> None:
                buf = self.model.ep_info_buffer
                if buf:
                    agent.train_history.append({
                        "timesteps": int(self.num_timesteps),
                        "mean_return": float(np.mean([e["r"] for e in buf])),
                        "mean_length": float(np.mean([e["l"] for e in buf])),
                    })

        self.model.learn(total_timesteps=int(total_timesteps), callback=_Checkpoint(),
                         reset_num_timesteps=False, progress_bar=progress_bar, **learn_kwargs)
        if log_dir is not None:
            self.save(Path(log_dir) / "latest")
        return self

    # ------------------------------------------------------------- inference
    def predict(self, obs: np.ndarray, deterministic: bool = True) -> np.ndarray:
        if self.model is None:
            raise RuntimeError("agent has not been trained or loaded")
        x = self.vecnorm.normalize_obs(np.asarray(obs, dtype=np.float32)) if self.vecnorm else obs
        action, _ = self.model.predict(x, deterministic=deterministic)
        return action

    # ------------------------------------------------------------- save/load
    def _save(self, directory: Path) -> None:
        self.model.save(directory / "model.zip")
        self.vecnorm.save(str(directory / "vecnormalize.pkl"))

    def _load(self, directory: Path) -> None:
        from stable_baselines3 import PPO
        from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

        venv = DummyVecEnv([self.env_factory])
        self.vecnorm = VecNormalize.load(str(directory / "vecnormalize.pkl"), venv)
        self.vecnorm.training = False
        self.vecnorm.norm_reward = False
        self.model = PPO.load(directory / "model.zip", env=self.vecnorm, device=self.device)
        self._train_env_ready = False  # train() will rebuild n_envs training envs

    def close(self) -> None:
        if self.vecnorm is not None:
            self.vecnorm.close()
