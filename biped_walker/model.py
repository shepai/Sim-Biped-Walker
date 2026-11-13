"""Backwards-compatible entry point for model building (see :mod:`biped_walker.robots`).

``build_mjcf()`` with no robot argument still builds the original 4-DoF walker.
"""

from __future__ import annotations

from pathlib import Path

from .robots import ROBOTS, get_robot
from .robots.biped8 import Biped8Config
from .robots.common import TactileConfig, taxel_name
from .robots.walker4 import (BASE_BODY, FOOT_NAMES, FORWARD_LOCAL, JOINT_NAMES, MESH_DIR, SOLE_GEOMETRY,
                             ModelConfig, Walker4Config, taxel_layout)


def build_mjcf(cfg=None, robot: str = "walker4") -> str:
    """MJCF XML for ``robot`` ("walker4" or "biped8") with config ``cfg``."""
    spec = get_robot(robot)
    return spec.build_mjcf(spec.make_config(cfg))


def write_mjcf(path: str | Path, cfg=None, robot: str = "walker4") -> Path:
    """Write the generated MJCF to ``path`` (handy for ``python -m mujoco.viewer``)."""
    path = Path(path)
    path.write_text(build_mjcf(cfg, robot))
    return path


__all__ = ["build_mjcf", "write_mjcf", "ModelConfig", "Walker4Config", "Biped8Config", "TactileConfig",
           "JOINT_NAMES", "FOOT_NAMES", "BASE_BODY", "FORWARD_LOCAL", "SOLE_GEOMETRY", "MESH_DIR",
           "taxel_layout", "taxel_name", "ROBOTS"]
