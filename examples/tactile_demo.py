"""Drive the servos directly with the Biped class and plot the 3x5 foot pressure maps."""

import matplotlib.pyplot as plt
import numpy as np

from biped_walker import Biped

bot = Biped()
bot.reset(settle_steps=200)
snapshots = {}
for label, target in [("standing", [0, 0, 0, 0]),
                      ("ankles -0.4", [0, -0.4, 0, -0.4]),
                      ("ankles +0.4", [0, 0.4, 0, 0.4])]:
    bot.set_targets(target)
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
plt.savefig("tactile_demo.png", dpi=150)
print("saved tactile_demo.png")
