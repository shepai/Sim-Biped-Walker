"""Train a walker policy, with a progress bar and a log folder per run.

    python examples/train.py --algo ppo --task mixed --timesteps 3000000
    python examples/train.py --algo es  --task forward --timesteps 2000000 --workers 8
    python examples/train.py --robot biped10 --algo ppo --task forward --timesteps 10000000
    python examples/train.py --robot biped10 --task mixed --curriculum-start 0.4 --payload 0.3

The robustness curriculum is on by default: from --curriculum-start (fraction of
training) the floor friction, link masses and a payload in the body box are
randomised, growing to full strength at --curriculum-end.

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
                   help="walker4 = 4-DoF walker, biped10 = 10-DoF servo-bracket biped")
    p.add_argument("--algo", choices=["ppo", "es"], default="ppo")
    p.add_argument("--task", choices=["mixed", "forward", "turn"], default="mixed")
    p.add_argument("--timesteps", type=int, default=3_000_000)
    p.add_argument("--n-envs", type=int, default=8, help="PPO parallel envs")
    p.add_argument("--device", default="auto", help="PPO torch device: auto (CUDA if available), cpu, cuda, cuda:1 ...")
    p.add_argument("--workers", type=int, default=None, help="ES worker processes")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--history", type=int, default=3, help="frames of tactile+motor history in the obs")
    p.add_argument("--energy-weight", type=float, default=None, help="override the energy penalty weight")
    p.add_argument("--randomize", action="store_true",
                   help="full-strength robustness randomisation from the start (instead of the curriculum)")
    p.add_argument("--no-curriculum", action="store_true", help="train on the nominal robot and floor only")
    p.add_argument("--curriculum-start", type=float, default=0.3, help="progress where randomisation begins")
    p.add_argument("--curriculum-end", type=float, default=0.8, help="progress where it reaches full strength")
    p.add_argument("--friction", type=float, nargs=2, default=(0.4, 1.4), metavar=("LO", "HI"),
                   help="floor friction scale range at full strength")
    p.add_argument("--mass-scale", type=float, nargs=2, default=(0.85, 1.15), metavar=("LO", "HI"),
                   help="per-link mass scale range at full strength")
    p.add_argument("--payload", type=float, default=0.25, help="max payload in the body box, fraction of robot mass")
    p.add_argument("--no-imu", action="store_true", help="leave the IMU out of the observation")
    p.add_argument("--imu-noise", type=float, default=0.0, help="Gaussian noise on the normalised IMU readings")
    p.add_argument("--gait-clock", action="store_true",
                   help="add a left/right stepping clock to the obs and reward feet for following it")
    p.add_argument("--gait-period", type=float, default=None, help="gait clock period in s (robot default)")
    p.add_argument("--action-smoothing", type=float, default=0.0, help="low-pass filter on servo targets, 0..0.9")
    p.add_argument("--reward", nargs="*", default=[], metavar="NAME=VALUE",
                   help="override RewardConfig fields, e.g. --reward slip=1.0 clearance=1.0 air_time=2")
    p.add_argument("--log-dir", default="logs", help="parent folder for run folders")
    p.add_argument("--run-name", default=None, help="run folder name (default: date-time_robot_algo_task)")
    p.add_argument("--video-every", type=float, default=0.1,
                   help="video + checkpoint every this fraction of training (0 = off)")
    p.add_argument("--video-steps", type=int, default=500, help="control steps per video (50 = 1 s)")
    p.add_argument("--video-episodes", type=int, default=1)
    p.add_argument("--no-progress", action="store_true", help="hide the progress bar")
    args = p.parse_args()

    env_id = ENV_IDS[(args.robot, args.task)]
    env_kwargs = {"history_len": args.history, "include_imu": not args.no_imu}
    if args.imu_noise:
        env_kwargs["imu_noise"] = args.imu_noise
    if not args.no_curriculum:
        env_kwargs["curriculum"] = {"start": args.curriculum_start, "end": args.curriculum_end,
                                    "friction": list(args.friction), "mass_scale": list(args.mass_scale),
                                    "payload": [0.0, args.payload]}
    if args.randomize:
        env_kwargs["randomization_level"] = 1.0
    rewards = {k: float(v) for k, v in (kv.split("=", 1) for kv in args.reward)}
    if args.energy_weight is not None:
        rewards["energy"] = args.energy_weight
    if rewards:
        env_kwargs["rewards"] = rewards
    if args.gait_clock:
        env_kwargs["gait_clock"] = True
        if args.gait_period:
            env_kwargs["gait_period"] = args.gait_period
    if args.action_smoothing:
        env_kwargs["action_smoothing"] = args.action_smoothing

    if args.algo == "ppo":
        agent = make_agent("ppo", env_id=env_id, env_kwargs=env_kwargs, seed=args.seed, n_envs=args.n_envs,
                           device=args.device)
    else:
        agent = make_agent("es", env_id=env_id, env_kwargs=env_kwargs, seed=args.seed, n_workers=args.workers)

    agent.train(args.timesteps, log_dir=args.log_dir, run_name=args.run_name,
                video_every=args.video_every or None, video_steps=args.video_steps,
                video_episodes=args.video_episodes, progress_bar=not args.no_progress)

    stats = {"nominal": agent.evaluate(n_episodes=5)}
    if not args.no_curriculum:   # also test on randomised robots / floors
        stats["randomised"] = agent.evaluate(n_episodes=5, randomization_level=1.0)
    for v in stats.values():
        v.pop("episodes")
    (agent.run_dir / "final_eval.json").write_text(json.dumps(stats, indent=2))
    print(json.dumps(stats, indent=2))
    print(f"\nlogs: {agent.run_dir}\nwatch: python examples/visualize.py {agent.run_dir / 'best'}")


if __name__ == "__main__":  # required for multiprocessing on macOS / Windows
    main()
