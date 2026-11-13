"""Robot bodies available in the simulator.

* ``walker4``: the original 4-DoF Otto-style walker (hip yaw + ankle roll).
* ``biped8``: an 8-DoF MG996R servo-bracket biped (hip roll, hip pitch, knee,
  ankle roll), designed in ``cad/biped8``.

Both have 3 x 5 tactile arrays under each foot.
"""

from typing import Dict

from .common import RobotSpec, TactileConfig, taxel_layout, taxel_name
from . import biped8, walker4

ROBOTS: Dict[str, RobotSpec] = {walker4.SPEC.name: walker4.SPEC, biped8.SPEC.name: biped8.SPEC}
DEFAULT_ROBOT = "walker4"


def get_robot(name: str) -> RobotSpec:
    try:
        return ROBOTS[name]
    except KeyError:
        raise ValueError(f"unknown robot {name!r}; choose from {sorted(ROBOTS)}") from None


def build_mjcf(robot: str = DEFAULT_ROBOT, config=None) -> str:
    spec = get_robot(robot)
    return spec.build_mjcf(spec.make_config(config))


__all__ = ["ROBOTS", "DEFAULT_ROBOT", "RobotSpec", "TactileConfig", "get_robot", "build_mjcf",
           "taxel_layout", "taxel_name"]
