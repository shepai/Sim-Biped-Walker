"""Evolution-strategies agent (OpenAI-ES) in pure NumPy.

No deep-learning framework needed, parallelises over CPU cores with
``multiprocessing``, and is a natural step on from the genetic algorithms in
``legacy/``. Uses antithetic sampling, centred-rank fitness shaping, Adam and
running observation normalisation.
"""

from __future__ import annotations

import multiprocessing as mp
import os
import time
from pathlib import Path
from typing import Any, Dict, Optional, Sequence, Tuple

import numpy as np

from ..envs import EnvFactory
from .base import BaseRLAgent


class MLPPolicy:
    """Small tanh MLP whose parameters live in one flat vector."""

    def __init__(self, obs_dim: int, act_dim: int, hidden: Sequence[int] = (64, 64)):
        self.sizes = [obs_dim, *hidden, act_dim]
        self.shapes = []
        for a, b in zip(self.sizes[:-1], self.sizes[1:]):
            self.shapes += [(a, b), (b,)]
        self.n_params = int(sum(np.prod(s) for s in self.shapes))
        self.params = np.zeros(self.n_params)

    def init(self, rng: np.random.Generator) -> np.ndarray:
        chunks = []
        n_layers = len(self.sizes) - 1
        for k, (a, b) in enumerate(zip(self.sizes[:-1], self.sizes[1:])):
            scale = 0.01 if k == n_layers - 1 else 1.0 / np.sqrt(a)
            chunks += [rng.normal(0, scale, (a, b)).ravel(), np.zeros(b)]
        self.params = np.concatenate(chunks)
        return self.params

    def forward(self, params: np.ndarray, x: np.ndarray) -> np.ndarray:
        i = 0
        h = x
        n_layers = len(self.sizes) - 1
        for k in range(n_layers):
            (a, b), _ = self.shapes[2 * k], self.shapes[2 * k + 1]
            W = params[i:i + a * b].reshape(a, b)
            i += a * b
            bias = params[i:i + b]
            i += b
            h = np.tanh(h @ W + bias)  # tanh on the output too -> actions in [-1, 1]
        return h


class RunningNorm:
    """Running mean / variance for observation normalisation."""

    def __init__(self, dim: int, clip: float = 5.0):
        self.sum = np.zeros(dim)
        self.sumsq = np.zeros(dim)
        self.count = 1e-4
        self.clip = clip

    @property
    def mean(self) -> np.ndarray:
        return self.sum / self.count

    @property
    def std(self) -> np.ndarray:
        var = self.sumsq / self.count - self.mean ** 2
        return np.sqrt(np.maximum(var, 1e-6))

    def update(self, s: np.ndarray, ss: np.ndarray, n: float) -> None:
        self.sum += s
        self.sumsq += ss
        self.count += n

    def __call__(self, obs: np.ndarray, mean=None, std=None) -> np.ndarray:
        mean = self.mean if mean is None else mean
        std = self.std if std is None else std
        return np.clip((obs - mean) / std, -self.clip, self.clip)


# ---------------------------------------------------------------- workers
_WORKER: Dict[str, Any] = {}


def _init_worker(factory: EnvFactory, policy_args: Tuple) -> None:
    _WORKER["env"] = factory()
    _WORKER["policy"] = MLPPolicy(*policy_args)


def _rollout(job):
    params, mean, std, seed, max_steps, clip, progress = job
    env, policy = _WORKER["env"], _WORKER["policy"]
    base = env.unwrapped
    if hasattr(base, "set_training_progress"):
        base.set_training_progress(progress)       # robustness curriculum
    obs, _ = env.reset(seed=int(seed))
    total, steps = 0.0, 0
    s = np.zeros_like(mean)
    ss = np.zeros_like(mean)
    info = {}
    for _ in range(max_steps):
        s += obs
        ss += obs * obs
        x = np.clip((obs - mean) / std, -clip, clip)
        obs, r, term, trunc, info = env.step(policy.forward(params, x))
        total += r
        steps += 1
        if term or trunc:
            break
    summary = info.get("episode_summary") or {"return": total, "length": steps}
    return total, steps, s, ss, summary


def _centered_ranks(x: np.ndarray) -> np.ndarray:
    ranks = np.empty(x.size)
    ranks[x.ravel().argsort()] = np.arange(x.size)
    return (ranks / (x.size - 1) - 0.5).reshape(x.shape)


class ESAgent(BaseRLAgent):
    """OpenAI-style evolution strategies with an MLP policy.

    Parameters
    ----------
    hidden: hidden layer sizes of the policy.
    pop_size: population per generation (even; antithetic pairs).
    sigma: parameter noise std.
    lr: Adam learning rate.
    weight_decay: L2 pull of the parameters towards zero.
    episode_steps: max control steps per evaluation rollout.
    n_workers: processes for parallel rollouts (``1`` = no multiprocessing).
    """

    algo = "es"

    def __init__(self, env_id: str = "SimBiped-v0", env_kwargs: Optional[Dict[str, Any]] = None,
                 seed: Optional[int] = 0, hidden: Sequence[int] = (64, 64), pop_size: int = 48,
                 sigma: float = 0.03, lr: float = 0.02, weight_decay: float = 0.005,
                 episode_steps: int = 500, n_workers: Optional[int] = None):
        super().__init__(env_id, env_kwargs, seed)
        self.hidden = tuple(hidden)
        self.pop_size = int(pop_size) + int(pop_size) % 2
        self.sigma, self.lr, self.weight_decay = sigma, lr, weight_decay
        self.episode_steps = int(episode_steps)
        self.n_workers = n_workers or max(1, (os.cpu_count() or 2) - 1)
        self.rng = np.random.default_rng(seed)

        probe = self.make_env()
        self.obs_dim = int(probe.observation_space.shape[0])
        self.act_dim = int(probe.action_space.shape[0])
        probe.close()

        self.policy = MLPPolicy(self.obs_dim, self.act_dim, self.hidden)
        self.theta = self.policy.init(self.rng)
        self.obs_norm = RunningNorm(self.obs_dim)
        self._m = np.zeros_like(self.theta)
        self._v = np.zeros_like(self.theta)
        self._t = 0
        self.timesteps = 0

    def hyperparameters(self) -> Dict[str, Any]:
        return {"hidden": list(self.hidden), "pop_size": self.pop_size, "sigma": self.sigma, "lr": self.lr,
                "weight_decay": self.weight_decay, "episode_steps": self.episode_steps}

    # ------------------------------------------------------------- inference
    def predict(self, obs: np.ndarray, deterministic: bool = True) -> np.ndarray:
        x = self.obs_norm(np.asarray(obs, dtype=float))
        return self.policy.forward(self.theta, x).astype(np.float32)

    # -------------------------------------------------------------- training
    def train(self, total_timesteps: int, log_dir: Optional[str] = "logs", run_name: Optional[str] = None,
              video_every: Optional[float] = 0.1, video_episodes: int = 1, video_steps: Optional[int] = 500,
              progress_bar: bool = True, eval_every: int = 0, log_every: int = 1, **_) -> "ESAgent":
        """Run ES generations until ``total_timesteps`` more env steps are used.

        Logs every ``log_every`` generations (see :meth:`BaseRLAgent.train`);
        ``eval_every`` > 0 adds a 3-episode deterministic evaluation of the
        mean policy every that many generations.
        """
        policy_args = (self.obs_dim, self.act_dim, self.hidden)
        logger = self._start_run(total_timesteps, log_dir, run_name, video_every, video_episodes,
                                 video_steps, progress_bar, start_timesteps=self.timesteps)
        self.run_dir = logger.dir
        pool = None
        if self.n_workers > 1:
            pool = mp.get_context().Pool(self.n_workers, initializer=_init_worker,
                                         initargs=(self.env_factory, policy_args))
            run = pool.map
        else:
            _init_worker(self.env_factory, policy_args)
            run = lambda f, jobs: list(map(f, jobs))  # noqa: E731

        start = self.timesteps
        target = self.timesteps + int(total_timesteps)
        gen = len(self.train_history)
        try:
            while self.timesteps < target:
                t0 = time.time()
                half = self.pop_size // 2
                eps = self.rng.standard_normal((half, self.theta.size))
                seeds = self.rng.integers(0, 2 ** 31 - 1, size=half)
                mean, std = self.obs_norm.mean, self.obs_norm.std
                progress = (self.timesteps - start) / max(int(total_timesteps), 1)
                jobs = []
                for k in range(half):  # same seed for each antithetic pair
                    for sgn in (1.0, -1.0):
                        jobs.append((self.theta + sgn * self.sigma * eps[k], mean, std, seeds[k],
                                     self.episode_steps, self.obs_norm.clip, progress))
                results = run(_rollout, jobs)

                returns = np.array([r[0] for r in results]).reshape(half, 2)
                steps = int(sum(r[1] for r in results))
                for r in results:
                    self.obs_norm.update(r[2], r[3], r[1])
                self.timesteps += steps
                logger.log_episodes(self.timesteps, [r[4] for r in results])

                ranks = _centered_ranks(returns)
                grad = (ranks[:, 0] - ranks[:, 1]) @ eps / (self.pop_size * self.sigma)
                grad -= self.weight_decay * self.theta
                self._adam_ascent(grad)

                gen += 1
                stats = {"generation": gen, "timesteps": self.timesteps,
                         "mean_return": float(returns.mean()), "max_return": float(returns.max()),
                         "mean_length": steps / self.pop_size, "seconds": time.time() - t0}
                if eval_every and gen % eval_every == 0:
                    ev = self.evaluate(n_episodes=3)
                    stats["eval_return"] = ev["mean_return"]
                    stats["eval_distance"] = ev["mean_distance"]
                self.train_history.append(stats)

                logger.advance(self.timesteps)
                if gen % max(1, log_every) == 0 or self.timesteps >= target:
                    logger.log_metrics(self.timesteps, {
                        **stats,
                        "pop_return_min": float(returns.min()), "pop_return_std": float(returns.std()),
                        "grad_norm": float(np.linalg.norm(grad)), "param_norm": float(np.linalg.norm(self.theta)),
                        "sigma": self.sigma, "lr": self.lr,
                        "curriculum_level": self.curriculum_level(progress),
                    })
                    logger.maybe_save_best(self.timesteps)
                logger.handle_milestones(self.timesteps)
        finally:
            if pool is not None:
                pool.close()
                pool.join()
            if logger.dir is not None:
                self.save(logger.dir / "latest")
            logger.close(self.timesteps)
        return self

    def _adam_ascent(self, g: np.ndarray, b1: float = 0.9, b2: float = 0.999, eps: float = 1e-8) -> None:
        self._t += 1
        self._m = b1 * self._m + (1 - b1) * g
        self._v = b2 * self._v + (1 - b2) * g * g
        mhat = self._m / (1 - b1 ** self._t)
        vhat = self._v / (1 - b2 ** self._t)
        self.theta = self.theta + self.lr * mhat / (np.sqrt(vhat) + eps)

    # ------------------------------------------------------------- save/load
    def _save(self, directory: Path) -> None:
        np.savez(directory / "es_policy.npz", theta=self.theta, obs_sum=self.obs_norm.sum,
                 obs_sumsq=self.obs_norm.sumsq, obs_count=self.obs_norm.count,
                 adam_m=self._m, adam_v=self._v, adam_t=self._t, timesteps=self.timesteps)

    def _load(self, directory: Path) -> None:
        d = np.load(directory / "es_policy.npz")
        self.theta = d["theta"]
        self.obs_norm.sum, self.obs_norm.sumsq = d["obs_sum"], d["obs_sumsq"]
        self.obs_norm.count = float(d["obs_count"])
        self._m, self._v, self._t = d["adam_m"], d["adam_v"], int(d["adam_t"])
        self.timesteps = int(d["timesteps"])
