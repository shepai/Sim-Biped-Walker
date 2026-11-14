"""Per-run logging for training: metrics, episodes, videos, checkpoints, plots.

Every call to ``agent.train(...)`` creates ``logs/<run_name>/``::

    logs/20261114-091800_biped8_ppo_forward/
      config.json        agent, env and training settings
      metrics.csv        one row per update (PPO rollout / ES generation): rewards,
      metrics.jsonl      episode stats over the last 100 episodes, losses, fps ...
      episodes.csv       one row per finished training episode: return, length,
                         distance, energy, cost of transport, fell, command,
                         velocity tracking errors and every reward term
      train.log          human-readable log of the above + events
      videos/            010pct_step000123456.mp4 ... 100pct_...mp4
      checkpoints/       010pct/ ... 100pct/   (loadable with load_agent)
      best/              agent with the best mean episode return so far
      latest/            agent at the end of training
      progress.png       training curves (needs matplotlib)
      summary.json       final numbers
"""

from __future__ import annotations

import csv
import json
import math
import time
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional

import numpy as np

from ..utils.progress import ProgressBar

# Shown in the progress bar, in this order, when present.
BAR_KEYS = (("ep_return_mean", "R"), ("ep_length_mean", "len"), ("ep_distance_mean", "dist"),
            ("ep_fall_rate", "fall"), ("ep_cost_of_transport_mean", "CoT"), ("fps", "fps"))


class _CSV:
    """CSV file whose columns can grow (the file is rewritten when a new key appears)."""

    def __init__(self, path: Path):
        self.path = path
        self.columns: List[str] = []
        self.rows: List[Dict[str, Any]] = []

    def append(self, row: Dict[str, Any]) -> None:
        row = {k: _scalar(v) for k, v in row.items()}
        self.rows.append(row)
        new = [k for k in row if k not in self.columns]
        if new or not self.path.exists():
            self.columns += new
            with open(self.path, "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=self.columns)
                w.writeheader()
                w.writerows(self.rows)
        else:
            with open(self.path, "a", newline="") as f:
                csv.DictWriter(f, fieldnames=self.columns).writerow(row)


def _scalar(v):
    if isinstance(v, (np.floating, np.integer)):
        return v.item()
    if isinstance(v, np.ndarray):
        return v.tolist() if v.size > 1 else v.item()
    return v


def _finite_mean(xs: Iterable[float]) -> float:
    xs = [x for x in xs if x is not None and np.isfinite(x)]
    return float(np.mean(xs)) if xs else float("nan")


class RunLogger:
    """Creates the run folder and handles all logging during ``train``.

    Parameters
    ----------
    agent: the agent being trained (used for config, videos and checkpoints).
    total_timesteps: steps this ``train`` call will run (for progress and milestones).
    log_root: parent folder for runs (default ``logs``).
    run_name: folder name; default ``<date-time>_<robot>_<algo>_<task>``.
    video_every: record a video (and checkpoint) every this fraction of training
        (0.1 = every 10 %); ``0`` or ``None`` disables videos.
    video_episodes / video_steps: episodes per video and max control steps each.
    progress_bar: show a progress bar.
    window: number of recent episodes the ``ep_*`` statistics average over.
    """

    def __init__(self, agent, total_timesteps: int, log_root: str | Path = "logs", run_name: Optional[str] = None,
                 video_every: Optional[float] = 0.1, video_episodes: int = 1, video_steps: Optional[int] = 500,
                 progress_bar: bool = True, window: int = 100, start_timesteps: int = 0):
        self.agent = agent
        self.total = int(total_timesteps)
        self.start = int(start_timesteps)
        cfg = self._env_config()
        if run_name is None:
            stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
            run_name = f"{stamp}_{cfg.get('robot', 'robot')}_{agent.algo}_{cfg.get('task', 'task')}"
        self.dir = Path(log_root) / run_name
        k = 1
        while self.dir.exists() and any(self.dir.iterdir()):
            k += 1
            self.dir = Path(log_root) / f"{run_name}_{k}"
        (self.dir / "videos").mkdir(parents=True, exist_ok=True)
        (self.dir / "checkpoints").mkdir(exist_ok=True)

        self.metrics_csv = _CSV(self.dir / "metrics.csv")
        self.episodes_csv = _CSV(self.dir / "episodes.csv")
        self._jsonl = open(self.dir / "metrics.jsonl", "a")
        self._log = open(self.dir / "train.log", "a")
        self.recent = deque(maxlen=window)
        self.n_episodes = 0
        self.t0 = time.time()
        self._last_t, self._last_steps = self.t0, self.start
        self.best_return = -math.inf
        self.videos: List[str] = []
        self.video_steps = video_steps
        self.video_episodes = video_episodes
        self._video_failed = False
        self.milestones: List[tuple] = []
        if video_every:
            n = int(round(1 / video_every))
            self.milestones = [(int(round(100 * i / n)), self.start + int(round(self.total * i / n)))
                               for i in range(1, n + 1)]
        self.last_metrics: Dict[str, Any] = {}

        config = {"run_name": self.dir.name, "algo": agent.algo, "env_id": agent.env_id,
                  "env_kwargs": agent.env_kwargs, "env": cfg, "hyperparameters": agent.hyperparameters(),
                  "seed": agent.seed, "total_timesteps": self.total, "start_timesteps": self.start,
                  "video_every": video_every, "started": datetime.now().isoformat(timespec="seconds")}
        from .base import _json_default
        (self.dir / "config.json").write_text(json.dumps(config, indent=2, default=_json_default))
        self.bar = ProgressBar(self.total, desc=f"{agent.algo} {cfg.get('robot', '')}".strip(),
                               enabled=progress_bar)
        self.text(f"run folder: {self.dir.resolve()}", echo=True)

    def _env_config(self) -> Dict[str, Any]:
        try:
            env = self.agent.make_env()
            base = env.unwrapped
            cfg = base.config_dict()
            cfg.update(obs_dim=int(env.observation_space.shape[0]), act_dim=int(env.action_space.shape[0]),
                       dt=base.dt, joints=list(base.robot.joint_names))
            env.close()
            return cfg
        except Exception as e:  # pragma: no cover - logging must never stop training
            return {"error": repr(e)}

    # --------------------------------------------------------------- text
    def text(self, msg: str, echo: bool = False) -> None:
        line = f"[{datetime.now().strftime('%H:%M:%S')}] {msg}"
        self._log.write(line + "\n")
        self._log.flush()
        if echo:
            if hasattr(self, "bar"):
                self.bar.write(line)
            else:
                print(line, flush=True)

    # ----------------------------------------------------------- episodes
    def log_episodes(self, timesteps: int, summaries: Iterable[Dict[str, Any]]) -> None:
        """Record finished training episodes (dicts from ``info['episode_summary']``)."""
        for s in summaries:
            if not s:
                continue
            self.n_episodes += 1
            row = {"episode": self.n_episodes, "timesteps": int(timesteps), **s}
            self.episodes_csv.append(row)
            self.recent.append(s)

    def episode_stats(self) -> Dict[str, float]:
        if not self.recent:
            return {}
        keys = ("return", "length", "distance", "energy", "cost_of_transport", "mean_forward_velocity",
                "forward_velocity_error", "mean_yaw_rate", "yaw_rate_error", "mean_power")
        out = {f"ep_{k}_mean": _finite_mean(e.get(k) for e in self.recent) for k in keys}
        out["ep_return_std"] = float(np.std([e["return"] for e in self.recent]))
        out["ep_fall_rate"] = _finite_mean(float(e.get("fell", 0)) for e in self.recent)
        for k in self.recent[-1]:
            if k.startswith("r_"):
                out[f"ep_{k}_mean"] = _finite_mean(e.get(k) for e in self.recent)
        out["episodes"] = self.n_episodes
        return out

    # ------------------------------------------------------------ metrics
    def log_metrics(self, timesteps: int, metrics: Dict[str, Any]) -> Dict[str, Any]:
        """Log one update's metrics (merged with the recent-episode statistics)."""
        now = time.time()
        fps = (timesteps - self._last_steps) / max(now - self._last_t, 1e-9)
        self._last_t, self._last_steps = now, timesteps
        row = {"timesteps": int(timesteps), "progress": (timesteps - self.start) / max(self.total, 1),
               "time_s": round(now - self.t0, 2), "fps": fps, **self.episode_stats(), **metrics}
        row = {k: _scalar(v) for k, v in row.items()}
        self.metrics_csv.append(row)
        self._jsonl.write(json.dumps(row, default=float) + "\n")
        self._jsonl.flush()
        self.last_metrics = row
        shown = {short: row[k] for k, short in BAR_KEYS if k in row and row[k] == row[k]}
        self.bar.set_postfix(shown)
        self.text("  ".join(f"{k}={v:.4g}" if isinstance(v, float) else f"{k}={v}" for k, v in row.items()))
        return row

    def advance(self, timesteps: int) -> None:
        """Move the progress bar to ``timesteps`` (absolute)."""
        delta = timesteps - self.start - self.bar.n
        if delta > 0:
            self.bar.update(delta)

    # ------------------------------------------------- best / milestones
    def maybe_save_best(self, timesteps: int) -> bool:
        r = self.last_metrics.get("ep_return_mean")
        if r is None or not np.isfinite(r) or len(self.recent) < min(10, self.recent.maxlen):
            return False
        if r > self.best_return:
            self.best_return = float(r)
            self.agent.save(self.dir / "best")
            self.text(f"new best mean return {r:.3f} at step {timesteps:,} -> best/")
            return True
        return False

    def milestones_due(self, timesteps: int) -> List[tuple]:
        due = [m for m in self.milestones if timesteps >= m[1]]
        self.milestones = [m for m in self.milestones if timesteps < m[1]]
        return due

    def handle_milestones(self, timesteps: int) -> None:
        """Checkpoint and record a video for every 10 % (``video_every``) milestone passed."""
        due = self.milestones_due(timesteps)
        if not due:
            return
        pct = due[-1][0]
        ckpt = self.dir / "checkpoints" / f"{pct:03d}pct"
        self.agent.save(ckpt)
        self.text(f"{pct}% checkpoint -> {ckpt.relative_to(self.dir)}")
        self.record_video(f"{pct:03d}pct_step{timesteps:09d}", seed=pct)

    def record_video(self, name: str, seed: int = 0) -> Optional[str]:
        if self._video_failed:
            return None
        path = self.dir / "videos" / f"{name}.mp4"
        try:
            out = self.agent.visualize(n_episodes=self.video_episodes, record=True, show=False, video_path=path,
                                       max_steps=self.video_steps, verbose=False, seed=1000 * seed)
        except Exception as e:  # rendering can fail on headless machines without EGL/OSMesa
            self._video_failed = True
            self.text(f"video recording failed ({e!r}); continuing without videos. "
                      "On a headless machine set MUJOCO_GL=egl or MUJOCO_GL=osmesa.", echo=True)
            return None
        video = out.get("video")
        if video:
            self.videos.append(video)
            eps = out["episodes"]
            self.text(f"video {Path(video).name}: return={np.mean([e['return'] for e in eps]):.3f} "
                      f"distance={np.mean([e['distance'] for e in eps]):.3f} m "
                      f"command={eps[0]['command']}", echo=True)
        return video

    # -------------------------------------------------------------- close
    def close(self, timesteps: int) -> None:
        self.bar.close()
        summary = {"run_name": self.dir.name, "timesteps": int(timesteps), "episodes": self.n_episodes,
                   "wall_time_s": round(time.time() - self.t0, 1), "best_mean_return": self.best_return,
                   "final": self.last_metrics, "videos": [Path(v).name for v in self.videos],
                   "finished": datetime.now().isoformat(timespec="seconds")}
        (self.dir / "summary.json").write_text(json.dumps(summary, indent=2, default=float))
        try:
            self.plot()
        except Exception as e:  # matplotlib missing or a plotting hiccup
            self.text(f"no progress plot ({e!r})")
        self.text(f"finished: {timesteps:,} steps, {self.n_episodes} episodes, logs in {self.dir}", echo=True)
        self._jsonl.close()
        self._log.close()

    def plot(self) -> Path:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        rows = self.metrics_csv.rows
        eps = self.episodes_csv.rows
        if not rows:
            raise ValueError("no metrics yet")
        t = np.array([r["timesteps"] for r in rows])
        panels = [("ep_return_mean", "return", "mean episode return"),
                  ("ep_length_mean", "length", "episode length (steps)"),
                  ("ep_distance_mean", "distance", "distance per episode (m)"),
                  ("ep_forward_velocity_error_mean", "forward_velocity_error", "|v - v_cmd| (m/s)"),
                  ("ep_cost_of_transport_mean", "cost_of_transport", "cost of transport"),
                  ("ep_fall_rate", "fell", "fall rate")]
        fig, axes = plt.subplots(2, 3, figsize=(14, 7.5))
        for ax, (key, ep_key, title) in zip(axes.ravel(), panels):
            if eps and ep_key in eps[0]:
                et = [e["timesteps"] for e in eps]
                ev = [float(e[ep_key]) if e[ep_key] is not None else np.nan for e in eps]
                ax.scatter(et, ev, s=3, alpha=0.15, color="tab:gray", label="episodes")
            y = np.array([r.get(key, np.nan) if r.get(key) is not None else np.nan for r in rows], float)
            ax.plot(t, y, color="tab:blue", lw=1.8, label=f"mean of last {self.recent.maxlen}")
            ax.set_title(title)
            ax.set_xlabel("timesteps")
            ax.grid(alpha=0.3)
            if key == "ep_cost_of_transport_mean" and np.isfinite(y).any():
                ax.set_ylim(0, np.nanpercentile(y[np.isfinite(y)], 95) * 1.5 + 1e-9)
        axes[0, 0].legend(fontsize=8)
        fig.suptitle(self.dir.name)
        fig.tight_layout()
        path = self.dir / "progress.png"
        fig.savefig(path, dpi=110)
        plt.close(fig)
        return path


class ConsoleLogger(RunLogger):
    """Same interface as :class:`RunLogger` but writes no files (``log_dir=None``)."""

    def __init__(self, agent, total_timesteps: int, progress_bar: bool = True, window: int = 100,
                 start_timesteps: int = 0, **_):
        self.agent = agent
        self.total = int(total_timesteps)
        self.start = int(start_timesteps)
        self.dir = None
        self.recent = deque(maxlen=window)
        self.n_episodes = 0
        self.t0 = time.time()
        self._last_t, self._last_steps = self.t0, self.start
        self.best_return = -math.inf
        self.videos = []
        self.milestones = []
        self.last_metrics = {}
        self.bar = ProgressBar(self.total, desc=agent.algo, enabled=progress_bar)

    def text(self, msg: str, echo: bool = False) -> None:
        if echo:
            self.bar.write(msg)

    def log_episodes(self, timesteps, summaries) -> None:
        for s in summaries:
            if s:
                self.n_episodes += 1
                self.recent.append(s)

    def log_metrics(self, timesteps, metrics):
        now = time.time()
        fps = (timesteps - self._last_steps) / max(now - self._last_t, 1e-9)
        self._last_t, self._last_steps = now, timesteps
        row = {"timesteps": int(timesteps), "fps": fps, **self.episode_stats(), **metrics}
        self.last_metrics = row
        self.bar.set_postfix({short: row[k] for k, short in BAR_KEYS if k in row and row[k] == row[k]})
        return row

    def maybe_save_best(self, timesteps) -> bool:
        return False

    def handle_milestones(self, timesteps) -> None:
        return None

    def close(self, timesteps) -> None:
        self.bar.close()
