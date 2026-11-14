"""Procedural MJCF model of the original 4-DoF walker ("walker4").

The robot is an "Otto"-style walker: each leg has a hip that yaws about the
vertical axis and an ankle that rolls the foot about the forward axis.

Coordinate conventions (robot base frame, inherited from the CAD export):

* ``+y`` is the robot's forward direction (the feet stick out towards +y),
* ``+z`` is up,
* ``+x`` is the robot's right, so foot "A" (at -x) is the left foot and
  foot "B" (at +x) is the right foot.

The robot is spawned yawed by -90 deg so that its forward direction lines up
with world ``+x``.

Each foot's sole is covered by a ``rows x cols`` grid of thin box pads (the
default is 3 across the width and 5 along the length). Every pad is the
contact surface for one taxel and has a MuJoCo ``touch`` sensor attached to
a site that encloses it, so the 15 sensors per foot report the normal
contact force on each pad.

This module only builds an XML string and needs no MuJoCo import.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Tuple

import numpy as np

from .common import RobotSpec, TactileConfig, fmt as _f, taxel_layout as _layout, taxel_name, taxel_xml

MESH_DIR = Path(__file__).resolve().parents[1] / "assets" / "walker4" / "meshes"

# Actuated joints, in actuator / observation order.
JOINT_NAMES: Tuple[str, ...] = ("left_hip", "left_ankle", "right_hip", "right_ankle")
FOOT_NAMES: Tuple[str, ...] = ("left_foot", "right_foot")
BASE_BODY = "base"

# Robot forward direction expressed in the base frame.
FORWARD_LOCAL = (0.0, 1.0, 0.0)

# Flat sole rectangle of each foot mesh, in the foot body frame (measured from
# the STL files: the lowest face of each foot is flat and spans the full
# bounding box in x and y).
SOLE_GEOMETRY: Dict[str, Dict[str, Tuple[float, float] | float]] = {
    "left_foot": {"x": (-0.0626, -0.0024), "y": (-0.0432, 0.0569), "z": -0.0177, "across": "x", "along": "y"},
    "right_foot": {"x": (-0.0231, 0.0413), "y": (-0.0309, 0.0712), "z": -0.0182, "across": "x", "along": "y"},
}


@dataclass
class Walker4Config:
    """Physical and simulation parameters of the 4-DoF walker."""

    timestep: float = 0.002
    floor_friction: float = 1.0
    # Servo model (position actuators). The values are small-hobby-servo-ish.
    hip_range: float = 1.0         # rad, +/- joint limit of the hip yaw joints
    ankle_range: float = 0.7       # rad, +/- joint limit of the ankle roll joints
    kp: float = 3.0                # N m / rad
    force_limit: float = 0.35      # N m, servo stall torque
    joint_damping: float = 0.04    # N m s / rad
    armature: float = 0.0005       # kg m^2, reflected gearbox inertia
    spawn_yaw: float = -math.pi / 2  # aligns robot forward (+y body) with world +x
    tactile: TactileConfig = field(default_factory=TactileConfig)
    mesh_dir: Path = MESH_DIR
    offscreen_width: int = 1280
    offscreen_height: int = 720

    @classmethod
    def from_dict(cls, d: Dict) -> "Walker4Config":
        d = dict(d)
        if isinstance(d.get("tactile"), dict):
            d["tactile"] = TactileConfig(**d["tactile"])
        return cls(**d)

    def default_pose(self) -> np.ndarray:
        return np.zeros(len(JOINT_NAMES))

    def action_scale(self) -> np.ndarray:
        """Action +/-1 spans each joint's full range (as in the original env)."""
        return np.array([self.hip_range, self.ankle_range, self.hip_range, self.ankle_range])


ModelConfig = Walker4Config  # backwards-compatible name


# Inertial properties copied from the SolidWorks URDF export.
# fullinertia order: ixx iyy izz ixy ixz iyz
_INERTIA = {
    "base": ("-0.00028313 0.029706 -0.035079", 0.14933,
             "4.0229e-05 4.9381e-05 5.1205e-05 -1.2116e-11 2.4938e-11 2.8162e-07"),
    "left_hip": ("-0.00065839 0.0066104 0.0024618", 0.03832,
                 "3.1302e-05 1.8239e-05 1.6784e-05 2.5158e-08 0 0"),
    "left_foot": ("-0.032552 0.015574 -0.0049319", 0.058051,
                  "1.6057e-05 1.3652e-05 2.528e-05 1.4647e-07 1.0225e-09 -5.3564e-07"),
    "right_hip": ("-0.00042934 0.0076229 0.0024618", 0.03832,
                  "3.1261e-05 1.828e-05 1.6784e-05 7.3346e-07 0 0"),
    "right_foot": ("0.0086447 0.0288397 -0.0052064", 0.058051,
                   "1.60656e-05 1.3643e-05 2.52802e-05 -3.7819e-09 3.0114e-08 -5.3482e-07"),
}


def taxel_layout(foot: str, tactile: TactileConfig):
    """Pad / site placement for one walker4 foot (see :func:`common.taxel_layout`)."""
    return _layout(foot, SOLE_GEOMETRY[foot], tactile)


def _taxel_xml(foot: str, tactile: TactileConfig, indent: str):
    return taxel_xml(foot, SOLE_GEOMETRY[foot], tactile, indent)


def build_mjcf(cfg: Walker4Config | None = None) -> str:
    """Return the MJCF XML string for the walker on a flat floor."""
    cfg = cfg or Walker4Config()
    tac = cfg.tactile
    mesh_dir = Path(cfg.mesh_dir).resolve().as_posix()
    half = cfg.spawn_yaw / 2.0
    quat = (math.cos(half), 0.0, 0.0, math.sin(half))

    def inertial(body):
        pos, mass, full = _INERTIA[body]
        return f'<inertial pos="{pos}" mass="{mass}" fullinertia="{full}"/>'

    left_geoms, left_sensors = _taxel_xml("left_foot", tac, " " * 10)
    right_geoms, right_sensors = _taxel_xml("right_foot", tac, " " * 10)
    pad_rgba = _f(tac.rgba)

    return f"""<mujoco model="biped_walker">
  <compiler angle="radian" meshdir="{mesh_dir}" autolimits="true"/>
  <option timestep="{cfg.timestep}" integrator="implicitfast" gravity="0 0 -9.81"/>
  <visual>
    <global offwidth="{cfg.offscreen_width}" offheight="{cfg.offscreen_height}"/>
    <quality shadowsize="4096"/>
    <map znear="0.005"/>
  </visual>

  <default>
    <joint damping="{cfg.joint_damping}" armature="{cfg.armature}" limited="true"/>
    <position kp="{cfg.kp}" forcelimited="true" forcerange="-{cfg.force_limit} {cfg.force_limit}" ctrllimited="true"/>
    <default class="visual">
      <geom type="mesh" contype="0" conaffinity="0" group="2" density="0"/>
    </default>
    <default class="collision">
      <geom type="mesh" contype="1" conaffinity="1" group="3" density="0" rgba="0.5 0.5 0.5 0.3"/>
    </default>
    <default class="taxel">
      <geom contype="1" conaffinity="1" group="1" density="0" condim="3"
            friction="{cfg.floor_friction} 0.005 0.0001" solref="0.008 1" solimp="0.9 0.95 0.001"
            rgba="{pad_rgba}"/>
      <site group="4" rgba="1 0.3 0.3 0.25"/>
    </default>
  </default>

  <asset>
    <mesh name="base" file="base_link.STL"/>
    <mesh name="hipA" file="hipA.STL"/>
    <mesh name="footA" file="footA.STL"/>
    <mesh name="hipB" file="hipB.STL"/>
    <mesh name="footB" file="footB.STL"/>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.82 0.84 0.86" rgb2="0.68 0.70 0.73"
             width="512" height="512" mark="edge" markrgb="0.55 0.57 0.6"/>
    <material name="grid" texture="grid" texrepeat="10 10" texuniform="true" reflectance="0.05"/>
    <texture name="sky" type="skybox" builtin="gradient" rgb1="0.6 0.75 0.9" rgb2="0.15 0.2 0.3"
             width="512" height="512"/>
  </asset>

  <worldbody>
    <light name="sun" pos="0 0 3" dir="0 0 -1" directional="true" diffuse="0.8 0.8 0.8" castshadow="true"/>
    <geom name="floor" type="plane" size="0 0 0.05" material="grid"
          friction="{cfg.floor_friction} 0.005 0.0001" contype="1" conaffinity="1"/>

    <body name="{BASE_BODY}" pos="0 0 0.2" quat="{_f(quat)}">
      <freejoint name="root"/>
      {inertial("base")}
      <geom name="base_visual" class="visual" mesh="base" rgba="0.894 0.914 0.929 1"/>
      <geom name="base_collision" class="collision" mesh="base"/>
      <site name="imu" pos="0 0.03 -0.04" size="0.004" group="4"/>

      <body name="left_hip" pos="-0.0247156 0.0484689 -0.081">
        {inertial("left_hip")}
        <joint name="left_hip" axis="0 0 1" range="-{cfg.hip_range} {cfg.hip_range}"/>
        <geom name="left_hip_visual" class="visual" mesh="hipA" rgba="0.894 0.914 0.929 1"/>
        <body name="left_foot" pos="0.0218049 -0.0145258 -0.0480205">
          {inertial("left_foot")}
          <joint name="left_ankle" axis="0.00192582 -0.999998 0" range="-{cfg.ankle_range} {cfg.ankle_range}"/>
          <geom name="left_foot_visual" class="visual" mesh="footA" rgba="0.12 0.12 0.12 1"/>
{left_geoms}
        </body>
      </body>

      <body name="right_hip" pos="0.0246839 0.0487666 -0.081">
        {inertial("right_hip")}
        <joint name="right_hip" axis="0 0 1" range="-{cfg.hip_range} {cfg.hip_range}"/>
        <geom name="right_hip_visual" class="visual" mesh="hipB" rgba="0.894 0.914 0.929 1"/>
        <body name="right_foot" pos="0.00183572 -0.0261381 -0.0479323">
          {inertial("right_foot")}
          <joint name="right_ankle" axis="0.0562337 -0.998418 0" range="-{cfg.ankle_range} {cfg.ankle_range}"/>
          <geom name="right_foot_visual" class="visual" mesh="footB" rgba="0.12 0.12 0.12 1"/>
{right_geoms}
        </body>
      </body>
    </body>
  </worldbody>

  <actuator>
    <position name="left_hip" joint="left_hip" ctrlrange="-{cfg.hip_range} {cfg.hip_range}"/>
    <position name="left_ankle" joint="left_ankle" ctrlrange="-{cfg.ankle_range} {cfg.ankle_range}"/>
    <position name="right_hip" joint="right_hip" ctrlrange="-{cfg.hip_range} {cfg.hip_range}"/>
    <position name="right_ankle" joint="right_ankle" ctrlrange="-{cfg.ankle_range} {cfg.ankle_range}"/>
  </actuator>

  <sensor>
{left_sensors}
{right_sensors}
    <framequat name="base_quat" objtype="body" objname="{BASE_BODY}"/>
    <gyro name="gyro" site="imu"/>
    <accelerometer name="accel" site="imu"/>
  </sensor>
</mujoco>
"""


SPEC = RobotSpec(
    name="walker4",
    description="original 4-DoF Otto-style walker (hip yaw + ankle roll per leg)",
    config_cls=Walker4Config,
    build_mjcf=build_mjcf,
    joint_names=JOINT_NAMES,
    foot_names=FOOT_NAMES,
    base_body=BASE_BODY,
    forward_local=FORWARD_LOCAL,
    env_defaults={
        "tactile_max_force": 1.5,
        "gait_period": 0.8,
        # small robot: lower foot lift, slower feet
        "rewards": {"clearance_target": 0.008, "air_time_target": 0.2, "foot_v_ref": 0.06},
    },
    camera_distance=0.6,
)
