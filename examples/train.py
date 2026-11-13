"""Train a walker policy.

    python examples/train.py --algo ppo --task mixed --timesteps 3000000
    python examples/train.py --algo es  --task forward --timesteps 2000000 --workers 8
<<<<<<< Updated upstream
=======
    python examples/train.py --robot biped8 --algo ppo --task forward --timesteps 10000000
>>>>>>> Stashed changes

The run directory gets ``latest/`` (and ``best/`` for ES) agents that
``examples/visualize.py`` can load. Add ``--record`` to save a video of the
trained policy at the end.
"""

import argparse
import json
from pathlib import Path

<<<<<<< Updated upstream
from biped_walker.rl import make_agent

ENV_IDS = {"mixed": "SimBiped-v0", "forward": "SimBiped-Forward-v0", "turn": "SimBiped-Turn-v0"}


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--algo", choices=["ppo", "es"], default="ppo")
    p.add_argument("--task", choices=list(ENV_IDS), default="mixed")
=======
from biped_walker import ENV_IDS, ROBOTS
from biped_walker.rl import make_agent


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--robot", choices=sorted(ROBOTS), default="walker4",
                   help="walker4 = 4-DoF walker, biped8 = 8-DoF servo-bracket biped")
    p.add_argument("--algo", choices=["ppo", "es"], default="ppo")
    p.add_argument("--task", choices=["mixed", "forward", "turn"], default="mixed")
>>>>>>> Stashed changes
    p.add_argument("--timesteps", type=int, default=3_000_000)
    p.add_argument("--n-envs", type=int, default=8, help="PPO parallel envs")
    p.add_argument("--workers", type=int, default=None, help="ES worker processes")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--history", type=int, default=3, help="frames of tactile+motor history in the obs")
    p.add_argument("--energy-weight", type=float, default=None, help="override the energy penalty weight")
    p.add_argument("--randomize", action="store_true", help="randomise friction / masses each episode")
    p.add_argument("--out", type=Path, default=None, help="run directory (default runs/<algo>_<task>)")
    p.add_argument("--record", action="store_true", help="record a video of the result when done")
    args = p.parse_args()

<<<<<<< Updated upstream
    out = args.out or Path("runs") / f"{args.algo}_{args.task}"
=======
    out = args.out or Path("runs") / f"{args.robot}_{args.algo}_{args.task}"
    env_id = ENV_IDS[(args.robot, args.task)]
>>>>>>> Stashed changes
    env_kwargs = {"history_len": args.history, "randomize_dynamics": args.randomize}
    if args.energy_weight is not None:
        env_kwargs["rewards"] = {"energy": args.energy_weight}

    if args.algo == "ppo":
<<<<<<< Updated upstream
        agent = make_agent("ppo", env_id=ENV_IDS[args.task], env_kwargs=env_kwargs,
                           seed=args.seed, n_envs=args.n_envs)
    else:
        agent = make_agent("es", env_id=ENV_IDS[args.task], env_kwargs=env_kwargs,
=======
        agent = make_agent("ppo", env_id=env_id, env_kwargs=env_kwargs,
                           seed=args.seed, n_envs=args.n_envs)
    else:
        agent = make_agent("es", env_id=env_id, env_kwargs=env_kwargs,
>>>>>>> Stashed changes
                           seed=args.seed, n_workers=args.workers)

    agent.train(args.timesteps, log_dir=str(out))
    agent.save(out / "latest")

    stats = agent.evaluate(n_episodes=5)
    stats.pop("episodes")
    print(json.dumps(stats, indent=2))

    if args.record:
        agent.visualize(n_episodes=2, record=True, show=False, video_path=out / "trained.mp4")


if __name__ == "__main__":  # required for multiprocessing on macOS / Windows
    main()
