"""Pieces shared by every robot model: tactile feet and the robot spec."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Tuple

import numpy as np


@dataclass
class TactileConfig:
    """Layout of the tactile array under each foot.

    ``cols`` taxels across the width of the foot and ``rows`` along its
    length. The default is the 3 x 5 array.
    """

    cols: int = 3
    rows: int = 5
    pad_fill: float = 0.9          # fraction of the cell pitch covered by a pad
    pad_thickness: float = 0.003   # m, pads protrude 2/3 of this below the sole
    rgba: Tuple[float, float, float, float] = (0.15, 0.45, 0.85, 1.0)

    @property
    def shape(self) -> Tuple[int, int]:
        return (self.cols, self.rows)

    @property
    def n_taxels(self) -> int:
        return self.cols * self.rows


def taxel_name(foot: str, i: int, j: int) -> str:
    """Name of taxel ``(i, j)`` on ``foot``: i across the width (robot left -> right), j heel -> toe."""
    return f"{foot}_taxel_{i}_{j}"


def taxel_layout(foot: str, sole: Dict[str, Any], tactile: TactileConfig) -> List[dict]:
    """Pad / site placement for one foot, in the body frame the sole belongs to.

    ``sole`` gives the sole rectangle ``x=(x0, x1)``, ``y=(y0, y1)`` at height
    ``z`` (sole normal = -z), plus ``across`` / ``along``: the body axis running
    from the robot's left to right across the foot, and from heel to toe,
    e.g. ``"x"``, ``"-y"``.
    """
    across, along = sole.get("across", "x"), sole.get("along", "y")
    ax_c, sign_c = across[-1], (-1 if across.startswith("-") else 1)
    ax_l, sign_l = along[-1], (-1 if along.startswith("-") else 1)
    lo_c, hi_c = sole[ax_c]
    lo_l, hi_l = sole[ax_l]
    pc = (hi_c - lo_c) / tactile.cols
    pl = (hi_l - lo_l) / tactile.rows
    z = sole["z"]
    t = tactile.pad_thickness
    out = []
    for i in range(tactile.cols):
        for j in range(tactile.rows):
            c = lo_c + (i + 0.5) * pc if sign_c > 0 else hi_c - (i + 0.5) * pc
            l_ = lo_l + (j + 0.5) * pl if sign_l > 0 else hi_l - (j + 0.5) * pl
            pos = {ax_c: c, ax_l: l_}
            half = {ax_c: 0.5 * pc, ax_l: 0.5 * pl}
            xy = (pos["x"], pos["y"])
            out.append({
                "i": i, "j": j, "name": taxel_name(foot, i, j),
                # pad: from z - 2t/3 to z + t/3 (protrudes below the sole)
                "pad_pos": (*xy, z - t / 6.0),
                "pad_size": (half["x"] * tactile.pad_fill, half["y"] * tactile.pad_fill, t / 2.0),
                # site: tiles the sole without overlap, generous in z so the
                # contact point (which sits in the penetration region) is inside.
                "site_pos": (*xy, z - t),
                "site_size": (half["x"], half["y"], 1.5 * t),
            })
    return out


def fmt(v) -> str:
    return " ".join(f"{x:.6g}" for x in v)


def taxel_xml(foot: str, sole: Dict[str, Any], tactile: TactileConfig, indent: str) -> Tuple[str, str]:
    """``(geoms+sites, sensors)`` XML for one foot's tactile pads."""
    geoms, sensors = [], []
    for tx in taxel_layout(foot, sole, tactile):
        geoms.append(f'{indent}<geom name="{tx["name"]}" class="taxel" type="box" '
                     f'pos="{fmt(tx["pad_pos"])}" size="{fmt(tx["pad_size"])}"/>')
        geoms.append(f'{indent}<site name="{tx["name"]}" class="taxel" type="box" '
                     f'pos="{fmt(tx["site_pos"])}" size="{fmt(tx["site_size"])}"/>')
        sensors.append(f'    <touch name="{tx["name"]}" site="{tx["name"]}"/>')
    return "\n".join(geoms), "\n".join(sensors)


@dataclass(frozen=True)
class RobotSpec:
    """Everything the rest of the package needs to know about a robot body.

    ``config_cls`` instances must provide ``tactile``, ``timestep``,
    ``spawn_yaw``, ``default_pose()`` and ``action_scale()`` (both in joint
    order) and ``from_dict()``.
    """

    name: str
    description: str
    config_cls: type
    build_mjcf: Callable[[Any], str]
    joint_names: Tuple[str, ...]
    foot_names: Tuple[str, ...]
    base_body: str
    forward_local: Tuple[float, float, float]
    env_defaults: Dict[str, Any] = field(default_factory=dict)
    camera_distance: float = 0.6

    def make_config(self, config=None):
        if config is None:
            return self.config_cls()
        if isinstance(config, dict):
            return self.config_cls.from_dict(config)
        return config
