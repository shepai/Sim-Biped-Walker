"""Scripted motions on the 8-DoF servo-bracket biped, with live foot-pressure readout.

Squats, then shifts its weight from foot to foot with the hip and ankle roll
joints. A starting point for hand-made gaits, and a quick check
that the model and tactile feet behave.

    python examples/biped8_demo.py                       # viewer (mjpython on macOS)
    python examples/biped8_demo.py --record --no-show    # videos/biped8_demo.mp4
"""

import argparse
import time

import numpy as np

from biped_walker import make_env


def scripted_action(t, names):
    """Normalised targets (0 = standing pose) at time t."""
    a = np.zeros(len(names))
    idx = {n: i for i, n in enumerate(names)}
    if t < 4.0:                                   # squat: knee = -hip keeps the feet flat
        s = 0.3 * (1 - np.cos(2 * np.pi * t / 2.0))
        for side in ("left", "right"):
            a[idx[f"{side}_hip_pitch"]] = s / 0.8   # action scales: hip 0.8, knee 0.9 rad
            a[idx[f"{side}_knee"]] = -s / 0.9
    else:                                         # side-to-side weight shift
        s = np.sin(2 * np.pi * (t - 4.0) / 2.5)
        # +roll is outwards on both legs: opposite signs sway the body sideways,
        # and ankle roll = -hip roll keeps both feet flat (parallelogram sway)
        for side, sign in (("left", 1.0), ("right", -1.0)):
            a[idx[f"{side}_hip_roll"]] = 0.6 * s * sign             # 0.18 rad
            a[idx[f"{side}_ankle_roll"]] = -0.45 * s * sign         # -0.18 rad
    return a


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--seconds", type=float, default=12.0)
    p.add_argument("--record", action="store_true")
    p.add_argument("--video", default="videos/biped8_demo.mp4")
    p.add_argument("--no-show", action="store_true")
    args = p.parse_args()

    env = make_env("SimBiped8-v0", render_mode=None if args.no_show else "human",
                   command=[0.0, 0.0], max_episode_steps=100_000)
    base = env.unwrapped
    names = base.robot.joint_names
    obs, info = env.reset(seed=0)
    frames = []
    for k in range(int(args.seconds / base.dt)):
        t0 = time.perf_counter()
        obs, r, term, trunc, info = env.step(scripted_action(k * base.dt, names))
        if args.record:
            frames.append(base.render_rgb())
        if k % 25 == 0:
            tac = info["tactile"]
            print(f"t={k * base.dt:5.2f}s  left={tac[0].sum():5.2f} N  right={tac[1].sum():5.2f} N  "
                  f"P={info['power']:.2f} W")
        if term:
            print("fell over")
            break
        if not args.no_show:
            if not base.viewer_running:
                break
            time.sleep(max(0.0, base.dt - (time.perf_counter() - t0)))
    env.close()
    if args.record:
        from biped_walker.utils import save_video
        print("saved", save_video(frames, args.video, fps=base.metadata["render_fps"]))


if __name__ == "__main__":
    main()
