"""Drive the servos directly with the Biped class and plot the 3x5 foot pressure maps.

    python examples/tactile_demo.py                 # 4-DoF walker
    python examples/tactile_demo.py --robot biped10 # 10-DoF servo-bracket biped
"""

import argparse

import matplotlib.pyplot as plt
import numpy as np

from biped_walker import Biped

p = argparse.ArgumentParser()
p.add_argument("--robot", default="walker4", choices=["walker4", "biped10"])
args = p.parse_args()

bot = Biped(args.robot)
bot.reset(settle_steps=200)
# Tilt the feet with the ankle roll joints. walker4's ankles roll both feet the
# same way; biped10's are mirrored, so opposite signs tilt both feet the same way.
roll = np.array([1.0 if n.endswith("_ankle") or n == "left_ankle_roll" else
                 -1.0 if n == "right_ankle_roll" else 0.0 for n in bot.joint_names])
snapshots = {}
for label, a in [("standing", 0.0), ("ankles -0.5", -0.5), ("ankles +0.5", 0.5)]:
    bot.set_normalized(a * roll)
    bot.step(500)  # 1 s
    snapshots[label] = bot.tactile()
    print(label, "total load: %.2f N (weight %.2f N)" % (snapshots[label].sum(), bot.total_mass * 9.81))

fig, axes = plt.subplots(len(snapshots), 2, figsize=(4, 2.2 * len(snapshots)))
vmax = max(s.max() for s in snapshots.values())
for r, (label, tac) in enumerate(snapshots.items()):
    for f, name in enumerate(["left foot", "right foot"]):
        ax = axes[r, f]
        # rows = heel->toe (toe at top), columns = robot left->right
        ax.imshow(tac[f].T[::-1], vmin=0, vmax=vmax, cmap="magma")
        ax.set_title(f"{label}: {name}", fontsize=8)
        ax.set_xticks([]), ax.set_yticks([])
plt.tight_layout()
plt.savefig(f"tactile_demo_{args.robot}.png", dpi=150)
print(f"saved tactile_demo_{args.robot}.png")
