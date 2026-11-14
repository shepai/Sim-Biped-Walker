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
    def train(self, total_timesteps: int, log_dir: Optional[str] = "logs", run_name: Optional[str] = None,
              video_every: Optional[float] = 0.1, video_episodes: int = 1, video_steps: Optional[int] = 500,
              progress_bar: bool = True, sb3_verbose: int = 0, **learn_kwargs) -> "PPOAgent":
        """Train with PPO; see :meth:`BaseRLAgent.train` for the logging options.

        Metrics are logged after every rollout (``n_steps * n_envs`` steps),
        including SB3's training statistics (losses, KL, entropy, clip
        fraction, explained variance). If TensorBoard is installed, SB3 also
        writes TensorBoard logs to ``<run>/tb``.
        """
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
            self.model = PPO(
                "MlpPolicy", self.vecnorm, seed=self.seed, device=self.device, verbose=sb3_verbose,
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
        self.model.verbose = sb3_verbose

        start = int(self.model.num_timesteps)
        logger = self._start_run(total_timesteps, log_dir, run_name, video_every, video_episodes,
                                 video_steps, progress_bar, start_timesteps=start)
        self.run_dir = logger.dir
        import torch
        dev = self.model.device
        gpu = f" ({torch.cuda.get_device_name(dev)})" if dev.type == "cuda" else ""
        logger.text(f"PPO policy on {dev}{gpu} (device={self.device!r}); simulation runs on the CPU "
                    f"in {self.n_envs} env process(es)", echo=True)
        if logger.dir is not None:
            try:
                import tensorboard  # noqa: F401
                self.model.tensorboard_log = str(logger.dir / "tb")
            except ImportError:
                pass

        agent = self

        class _LogCallback(BaseCallback):
            """Feeds finished episodes, SB3 stats and milestones to the run logger."""

            def __init__(self):
                super().__init__()
                self.rollouts = 0
                self.level = 0.0

            def _on_step(self) -> bool:
                dones = self.locals.get("dones")
                infos = self.locals.get("infos", [])
                if dones is not None and np.any(dones):
                    logger.log_episodes(self.num_timesteps, [info.get("episode_summary")
                                                             for info, d in zip(infos, dones) if d])
                logger.advance(self.num_timesteps)
                return True

            def _on_rollout_start(self) -> None:
                # robustness curriculum: tell every env how far training is
                progress = (self.num_timesteps - start) / max(int(total_timesteps), 1)
                self.level = agent.curriculum_level(progress)
                if self.level > 0 or self.rollouts == 0:
                    self.training_env.env_method("set_training_progress", progress)

            def _on_rollout_end(self) -> None:
                self.rollouts += 1
                # SB3 records train/* during the previous update and dumps them right after this
                sb3 = {k.replace("/", "_"): v for k, v in self.model.logger.name_to_value.items()
                       if isinstance(v, (int, float, np.floating, np.integer))}
                buf = self.model.ep_info_buffer
                lr = self.model.lr_schedule(self.model._current_progress_remaining)
                row = logger.log_metrics(self.num_timesteps, {
                    "rollout": self.rollouts, "learning_rate": float(lr),
                    "curriculum_level": self.level,
                    "policy_log_std_mean": float(self.model.policy.log_std.mean().item()),
                    "monitor_return_mean": float(np.mean([e["r"] for e in buf])) if buf else float("nan"),
                    **sb3,
                })
                agent.train_history.append({k: row[k] for k in ("timesteps", "ep_return_mean", "ep_length_mean")
                                            if k in row})
                logger.maybe_save_best(self.num_timesteps)
                logger.handle_milestones(self.num_timesteps)

        try:
            self.model.learn(total_timesteps=int(total_timesteps), callback=_LogCallback(),
                             reset_num_timesteps=False, progress_bar=False, **learn_kwargs)
        finally:
            if logger.dir is not None:
                self.save(logger.dir / "latest")
            logger.close(int(self.model.num_timesteps))
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
