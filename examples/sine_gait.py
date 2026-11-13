"""Open-loop sine gait using the low-level Biped API (no RL), with live tactile readout.

Ankles roll to shift the weight onto one foot while the hips swing the
unloaded foot forward: the classic Otto-style shuffle. The parameters are a
hand-picked starting point; flip the sign of ``--phase`` if it walks backwards.

    python examples/sine_gait.py                   # viewer
    python examples/sine_gait.py --record --no-show --video sine.mp4
"""

import argparse
import time

import numpy as np

from biped_walker import make_env


def sine_action(t, freq=1.2, hip_amp=0.45, ankle_amp=0.55, phase=np.pi / 2):
    """Normalised servo targets at time t."""
    w = 2 * np.pi * freq
    roll = ankle_amp * np.sin(w * t)
    swing = hip_amp * np.sin(w * t + phase)
    # [left_hip, left_ankle, right_hip, right_ankle] in [-1, 1]
    return np.array([swing, roll, swing, roll])


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--seconds", type=float, default=10.0)
    p.add_argument("--freq", type=float, default=1.2)
    p.add_argument("--phase", type=float, default=np.pi / 2, help="hip/ankle phase offset (rad)")
    p.add_argument("--record", action="store_true")
    p.add_argument("--video", default="videos/sine_gait.mp4")
    p.add_argument("--no-show", action="store_true")
    args = p.parse_args()

    env = make_env("SimBiped-Forward-v0", render_mode=None if args.no_show else "human",
                   command=[0.05, 0.0], max_episode_steps=100_000)
    base = env.unwrapped
    obs, info = env.reset(seed=0)
    frames, ret = [], 0.0
    for k in range(int(args.seconds / base.dt)):
        t0 = time.perf_counter()
        obs, r, term, trunc, info = env.step(sine_action(k * base.dt, args.freq, phase=args.phase))
        ret += r
        if args.record:
            frames.append(base.render_rgb())
        if k % 25 == 0:
            tac = info["tactile"]  # (2 feet, 3 across, 5 heel->toe) newtons
            print(f"t={k * base.dt:5.2f}s v={info['forward_velocity']:+.3f} m/s "
                  f"P={info['power']:.2f} W  left={tac[0].sum():.2f} N right={tac[1].sum():.2f} N")
        if term:
            print("fell over")
            break
        if not args.no_show:
            if not base.viewer_running:
                break
            time.sleep(max(0.0, base.dt - (time.perf_counter() - t0)))
    print(f"return={ret:.2f} distance={info['distance']:.3f} m CoT={info['cost_of_transport']:.2f}")
    env.close()
    if args.record:
        from biped_walker.utils import save_video
        print("saved", save_video(frames, args.video, fps=base.metadata["render_fps"]))


if __name__ == "__main__":
    main()
