"""Watch and/or record a trained agent.

<<<<<<< Updated upstream
    python examples/visualize.py runs/ppo_mixed/latest
    python examples/visualize.py runs/ppo_mixed/latest --command 0.05 0      # walk forward
    python examples/visualize.py runs/ppo_mixed/latest --command 0 0.4       # turn left on the spot
    python examples/visualize.py runs/es_forward/best --record --no-show --video out.mp4
=======
    python examples/visualize.py runs/walker4_ppo_mixed/latest
    python examples/visualize.py runs/walker4_ppo_mixed/latest --command 0.05 0   # walk forward
    python examples/visualize.py runs/walker4_ppo_mixed/latest --command 0 0.4    # turn left on the spot
    python examples/visualize.py runs/biped8_es_forward/best --record --no-show --video out.mp4

The robot (walker4 / biped8) is stored with the agent, so nothing else is needed.
>>>>>>> Stashed changes

Headless recording: set MUJOCO_GL=egl (GPU) or MUJOCO_GL=osmesa (CPU).
On macOS the interactive viewer needs ``mjpython`` instead of ``python``.
"""

import argparse

from biped_walker.rl import load_agent


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("agent_dir")
    p.add_argument("--episodes", type=int, default=3)
    p.add_argument("--command", type=float, nargs=2, metavar=("SPEED", "YAW_RATE"), default=None,
                   help="fixed command: forward speed (m/s) and yaw rate (rad/s, +ve = left)")
    p.add_argument("--record", action="store_true", help="save a video")
    p.add_argument("--video", default="videos/walker.mp4")
    p.add_argument("--no-show", action="store_true", help="don't open the viewer")
    p.add_argument("--max-steps", type=int, default=None)
    p.add_argument("--stochastic", action="store_true")
    args = p.parse_args()

    agent = load_agent(args.agent_dir)
    agent.visualize(n_episodes=args.episodes, record=args.record, video_path=args.video,
                    show=not args.no_show, command=args.command, max_steps=args.max_steps,
                    deterministic=not args.stochastic)


if __name__ == "__main__":
    main()
