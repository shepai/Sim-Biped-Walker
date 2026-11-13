"""Common interface for the RL agents: train, predict, evaluate, visualise, save/load."""

from __future__ import annotations

import json
import time
from abc import ABC, abstractmethod
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

import numpy as np

from ..envs import DEFAULT_ENV_ID, EnvFactory
from ..utils.video import save_video

META_FILE = "agent.json"


class BaseRLAgent(ABC):
    """Base class every walker agent derives from.

    Subclasses implement :meth:`train`, :meth:`predict`, :meth:`_save` and
    :meth:`_load`. Evaluation, visualisation (with optional video recording)
    and the save/load bookkeeping are shared.

    Parameters
    ----------
    env_id:
        Registered environment id, e.g. ``"SimBiped-v0"`` (mixed walking and
        turning), ``"SimBiped-Forward-v0"`` or ``"SimBiped-Turn-v0"``.
    env_kwargs:
        Keyword arguments for the environment (JSON-serialisable, so they can
        be saved with the agent and the exact env rebuilt at load time).
    seed:
        Random seed.
    """

    algo: str = "base"

    def __init__(self, env_id: str = DEFAULT_ENV_ID, env_kwargs: Optional[Dict[str, Any]] = None,
                 seed: Optional[int] = None):
        self.env_id = env_id
        self.env_kwargs = dict(env_kwargs or {})
        self.seed = seed
        self.env_factory = EnvFactory(env_id, **self.env_kwargs)
        self.train_history: List[Dict[str, float]] = []

    # ------------------------------------------------------------ abstract
    @abstractmethod
    def train(self, total_timesteps: int, log_dir: Optional[str] = None, **kwargs) -> "BaseRLAgent":
        """Train for ``total_timesteps`` environment steps."""

    @abstractmethod
    def predict(self, obs: np.ndarray, deterministic: bool = True) -> np.ndarray:
        """Map an observation (or a batch) to an action in ``[-1, 1]``."""

    @abstractmethod
    def _save(self, directory: Path) -> None:
        """Write the algorithm-specific files into ``directory``."""

    @abstractmethod
    def _load(self, directory: Path) -> None:
        """Restore the algorithm-specific state from ``directory``."""

    def hyperparameters(self) -> Dict[str, Any]:
        """Constructor arguments needed to rebuild the agent (saved with it)."""
        return {}

    # ------------------------------------------------------------- helpers
    def make_env(self, render_mode: Optional[str] = None, **overrides):
        return self.env_factory(render_mode=render_mode, **overrides)

    # ------------------------------------------------------------ save/load
    def save(self, directory: str | Path) -> Path:
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        meta = {
            "algo": self.algo,
            "env_id": self.env_id,
            "env_kwargs": self.env_kwargs,
            "seed": self.seed,
            "hyperparameters": self.hyperparameters(),
            "train_history": self.train_history,
        }
        (directory / META_FILE).write_text(json.dumps(meta, indent=2, default=_json_default))
        self._save(directory)
        return directory

    @classmethod
    def load(cls, directory: str | Path, **overrides) -> "BaseRLAgent":
        directory = Path(directory)
        meta = json.loads((directory / META_FILE).read_text())
        if cls.algo != "base" and meta["algo"] != cls.algo:
            raise ValueError(f"{directory} holds a {meta['algo']!r} agent, not {cls.algo!r}")
        kwargs = {**meta.get("hyperparameters", {}), **overrides}
        agent = cls(env_id=meta["env_id"], env_kwargs=meta["env_kwargs"], seed=meta.get("seed"), **kwargs)
        agent.train_history = meta.get("train_history", [])
        agent._load(directory)
        return agent

    # ----------------------------------------------------------- evaluation
    def run_episode(self, env, deterministic: bool = True, command: Optional[Sequence[float]] = None,
                    max_steps: Optional[int] = None, on_step=None, seed: Optional[int] = None) -> Dict[str, Any]:
        options = {"command": list(command)} if command is not None else None
        obs, info = env.reset(seed=seed, options=options)
        total, steps, done = 0.0, 0, False
        while not done:
            action = self.predict(obs, deterministic=deterministic)
            obs, reward, terminated, truncated, info = env.step(action)
            total += reward
            steps += 1
            done = terminated or truncated or (max_steps is not None and steps >= max_steps)
            if on_step is not None and on_step(env, info) is False:
                break
        return {
            "return": total,
            "length": steps,
            "fell": bool(terminated),
            "distance": info["distance"],
            "energy": info["energy"],
            "cost_of_transport": info["cost_of_transport"],
            "command": info["command"].tolist(),
        }

    def evaluate(self, n_episodes: int = 5, deterministic: bool = True,
                 command: Optional[Sequence[float]] = None, seed: int = 12345, **env_overrides) -> Dict[str, Any]:
        """Average return / distance / energy over ``n_episodes`` (no rendering)."""
        env = self.make_env(**env_overrides)
        try:
            eps = [self.run_episode(env, deterministic, command, seed=seed + i) for i in range(n_episodes)]
        finally:
            env.close()
        cot = [e["cost_of_transport"] for e in eps if np.isfinite(e["cost_of_transport"])]
        return {
            "mean_return": float(np.mean([e["return"] for e in eps])),
            "std_return": float(np.std([e["return"] for e in eps])),
            "mean_length": float(np.mean([e["length"] for e in eps])),
            "fall_rate": float(np.mean([e["fell"] for e in eps])),
            "mean_distance": float(np.mean([e["distance"] for e in eps])),
            "mean_cost_of_transport": float(np.mean(cot)) if cot else float("inf"),
            "episodes": eps,
        }

    # --------------------------------------------------------- visualisation
    def visualize(self, n_episodes: int = 1, record: bool = False, video_path: str | Path = "videos/walker.mp4",
                  show: bool = True, deterministic: bool = True, command: Optional[Sequence[float]] = None,
                  max_steps: Optional[int] = None, realtime: bool = True, width: int = 640, height: int = 480,
                  **env_overrides) -> Dict[str, Any]:
        """Roll out the policy, optionally in the MuJoCo viewer and/or recorded to video.

        Parameters
        ----------
        record:
            If True, render every control step offscreen and write all
            episodes to ``video_path`` (``.mp4`` or ``.gif``).
        show:
            Open the interactive MuJoCo viewer. Set False on headless
            machines (use ``MUJOCO_GL=egl`` or ``osmesa`` for recording).
        command:
            Fixed ``(forward_speed m/s, yaw_rate rad/s)``; if None, commands
            are sampled as in training.
        realtime:
            Slow the viewer down to real time.
        """
        render_mode = "human" if show else None
        env = self.make_env(render_mode=render_mode, width=width, height=height, **env_overrides)
        base = env.unwrapped
        frames: List[np.ndarray] = []
        results = []

        def on_step(_env, _info):
            if record:
                frames.append(base.render_rgb())
            if show:
                if realtime:
                    time.sleep(max(0.0, base.dt - (time.perf_counter() - on_step.t)))
                    on_step.t = time.perf_counter()
                if not base.viewer_running:
                    return False
            return True

        on_step.t = time.perf_counter()
        try:
            for ep in range(n_episodes):
                res = self.run_episode(env, deterministic, command, max_steps, on_step, seed=ep)
                results.append(res)
                print(f"episode {ep}: return={res['return']:.2f} steps={res['length']} "
                      f"distance={res['distance']:.3f} m CoT={res['cost_of_transport']:.2f} "
                      f"command={np.round(res['command'], 3).tolist()}")
                if show and not base.viewer_running:
                    break
        finally:
            env.close()
        out: Dict[str, Any] = {"episodes": results}
        if record and frames:
            out["video"] = str(save_video(frames, video_path, fps=base.metadata["render_fps"]))
            print(f"saved video to {out['video']}")
        return out


def _json_default(o):
    if isinstance(o, (np.floating, np.integer)):
        return o.item()
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, Path):
        return str(o)
    if hasattr(o, "__dataclass_fields__"):
        from dataclasses import asdict
        return asdict(o)
    raise TypeError(f"not JSON serialisable: {type(o)}")
