"""Train a walker policy, with a progress bar and a log folder per run.

    python examples/train.py --algo ppo --task mixed --timesteps 3000000
    python examples/train.py --algo es  --task forward --timesteps 2000000 --workers 8
    python examples/train.py --robot biped8 --algo ppo --task forward --timesteps 10000000

Each run writes to ``logs/<date-time>_<robot>_<algo>_<task>/`` (or
``--log-dir/--run-name``):

    metrics.csv / metrics.jsonl  rewards, episode stats, losses ... after every update
    episodes.csv                 every finished training episode, incl. each reward term
    train.log                    readable log
    videos/                      a video every 10 % of training (--video-every)
    checkpoints/                 an agent every 10 % of training
    best/  latest/               best-so-far and final agents (load with visualize.py)
    progress.png  summary.json   curves and final numbers

Headless machines need ``MUJOCO_GL=egl`` (GPU) or ``MUJOCO_GL=osmesa`` (CPU) for
the videos; without them training continues and videos are skipped.
"""

import argparse
import json

from biped_walker import ENV_IDS, ROBOTS
from biped_walker.rl import make_agent


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--robot", choices=sorted(ROBOTS), default="walker4",
                   help="walker4 = 4-DoF walker, biped8 = 8-DoF servo-bracket biped")
    p.add_argument("--algo", choices=["ppo", "es"], default="ppo")
    p.add_argument("--task", choices=["mixed", "forward", "turn"], default="mixed")
    p.add_argument("--timesteps", type=int, default=3_000_000)
    p.add_argument("--n-envs", type=int, default=8, help="PPO parallel envs")
    p.add_argument("--workers", type=int, default=None, help="ES worker processes")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--history", type=int, default=3, help="frames of tactile+motor history in the obs")
    p.add_argument("--energy-weight", type=float, default=None, help="override the energy penalty weight")
    p.add_argument("--randomize", action="store_true", help="randomise friction / masses each episode")
    p.add_argument("--log-dir", default="logs", help="parent folder for run folders")
    p.add_argument("--run-name", default=None, help="run folder name (default: date-time_robot_algo_task)")
    p.add_argument("--video-every", type=float, default=0.1,
                   help="video + checkpoint every this fraction of training (0 = off)")
    p.add_argument("--video-steps", type=int, default=500, help="control steps per video (50 = 1 s)")
    p.add_argument("--video-episodes", type=int, default=1)
    p.add_argument("--no-progress", action="store_true", help="hide the progress bar")
    args = p.parse_args()

    env_id = ENV_IDS[(args.robot, args.task)]
    env_kwargs = {"history_len": args.history, "randomize_dynamics": args.randomize}
    if args.energy_weight is not None:
        env_kwargs["rewards"] = {"energy": args.energy_weight}

    if args.algo == "ppo":
        agent = make_agent("ppo", env_id=env_id, env_kwargs=env_kwargs, seed=args.seed, n_envs=args.n_envs)
    else:
        agent = make_agent("es", env_id=env_id, env_kwargs=env_kwargs, seed=args.seed, n_workers=args.workers)

    agent.train(args.timesteps, log_dir=args.log_dir, run_name=args.run_name,
                video_every=args.video_every or None, video_steps=args.video_steps,
                video_episodes=args.video_episodes, progress_bar=not args.no_progress)

    stats = agent.evaluate(n_episodes=5)
    stats.pop("episodes")
    (agent.run_dir / "final_eval.json").write_text(json.dumps(stats, indent=2))
    print(json.dumps(stats, indent=2))
    print(f"\nlogs: {agent.run_dir}\nwatch: python examples/visualize.py {agent.run_dir / 'best'}")


if __name__ == "__main__":  # required for multiprocessing on macOS / Windows
    main()
