"""MJCF model of the 10-DoF MG996R servo-bracket biped ("biped10").

Each leg: hip yaw, hip roll, hip pitch, knee, ankle roll. The kinematic
layout, masses and inertias come from ``assets/biped10/biped10.json``, which
``biped_walker.cad.biped10_design`` generates from the part designs in
``cad/biped10/biped10_parts.scad`` (mirrored in ``biped_walker/cad/parts.py``).

Frames: robot x forward, y left, z up; it spawns facing world +x.
Sign conventions (both legs, mirrored so equal commands give mirror-symmetric
motion): +hip_yaw turns the toe outwards, +hip_roll swings the leg outwards, +hip_pitch swings the thigh
forward, the knee bends backwards with negative angles, +ankle_roll tips the
shin outwards over the foot (the foot's inner edge presses down).

With no ankle pitch, the foot stays flat when ``knee = -hip_pitch``: the
standing pose has the thigh slightly forward and the shin vertical.

The hip-yaw servos stand inside the body box; the other servos lie on their
sides, which keeps the legs short (hip roll axis to sole about 0.17 m). Joint
ranges stay inside the bracket clearances of the CAD layout.

Servos: TowerPro MG996R at 6 V, about 11 kgf cm (1.08 N m) stall torque and
0.14 s / 60 deg no-load speed (about 7.5 rad/s), modelled as position actuators
with a torque limit, plus joint damping of stall torque / no-load speed to
mimic the motor's back-EMF speed limit.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Tuple

import numpy as np

from .common import RobotSpec, TactileConfig, fmt, taxel_xml

ASSET_DIR = Path(__file__).resolve().parents[1] / "assets" / "biped10"
MODEL_JSON = ASSET_DIR / "biped10.json"
MESH_DIR = ASSET_DIR / "meshes"

JOINT_TYPES = ("hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle_roll")
JOINT_NAMES: Tuple[str, ...] = tuple(f"{s}_{j}" for s in ("left", "right") for j in JOINT_TYPES)
FOOT_NAMES: Tuple[str, ...] = ("left_foot", "right_foot")
BASE_BODY = "torso"
FORWARD_LOCAL = (1.0, 0.0, 0.0)


def _default_ranges() -> Dict[str, Tuple[float, float]]:
    # limits on the servo-body side come from bracket clearance in the CAD layout
    return {"hip_yaw": (-0.5, 0.5), "hip_roll": (-0.5, 0.55), "hip_pitch": (-1.0, 1.6),
            "knee": (-2.0, 0.1), "ankle_roll": (-0.6, 0.6)}


def _default_stand() -> Dict[str, float]:
    return {"hip_yaw": 0.0, "hip_roll": 0.0, "hip_pitch": 0.25, "knee": -0.25, "ankle_roll": 0.0}


def _default_action_scale() -> Dict[str, float]:
    return {"hip_yaw": 0.4, "hip_roll": 0.35, "hip_pitch": 0.8, "knee": 0.9, "ankle_roll": 0.4}


@dataclass
class Biped10Config:
    """Physical and simulation parameters of the 10-DoF biped.

    Joint settings are per joint type and shared by both legs.
    Actions map to ``stand_pose + action * action_scale`` (clipped to range).
    """

    timestep: float = 0.002
    floor_friction: float = 1.0
    kp: float = 12.0                 # N m / rad
    force_limit: float = 1.08        # N m, MG996R stall torque at 6 V
    joint_damping: float = 0.14      # N m s / rad ~ stall torque / no-load speed
    armature: float = 0.004          # kg m^2, reflected gearbox inertia
    joint_ranges: Dict[str, Tuple[float, float]] = field(default_factory=_default_ranges)
    stand_pose: Dict[str, float] = field(default_factory=_default_stand)
    action_scales: Dict[str, float] = field(default_factory=_default_action_scale)
    spawn_yaw: float = 0.0
    tactile: TactileConfig = field(default_factory=TactileConfig)
    offscreen_width: int = 1280
    offscreen_height: int = 720

    @classmethod
    def from_dict(cls, d: Dict) -> "Biped10Config":
        d = dict(d)
        if isinstance(d.get("tactile"), dict):
            d["tactile"] = TactileConfig(**d["tactile"])
        return cls(**d)

    def _per_joint(self, table) -> np.ndarray:
        return np.array([table[n.split("_", 1)[1]] for n in JOINT_NAMES], dtype=float)

    def default_pose(self) -> np.ndarray:
        return self._per_joint(self.stand_pose)

    def action_scale(self) -> np.ndarray:
        return self._per_joint(self.action_scales)


def load_model_data() -> dict:
    return json.loads(MODEL_JSON.read_text())


def build_mjcf(cfg: Biped10Config | None = None) -> str:
    cfg = cfg or Biped10Config()
    data = load_model_data()
    tac = cfg.tactile
    sole = {"x": tuple(data["sole"]["x"]), "y": tuple(data["sole"]["y"]), "z": 0.0,
            "across": "-y", "along": "x"}
    children: Dict[str, list] = {}
    for b in data["bodies"]:
        children.setdefault(b["parent"], []).append(b)
    sensors = []

    def body_xml(b, depth):
        ind = "  " * depth
        lines = [f'{ind}<body name="{b["name"]}" pos="{fmt(b["pos"])}" quat="{fmt(b["quat"])}">']
        if b["parent"] is None:
            lines[0] = (f'{ind}<body name="{b["name"]}" pos="0 0 0.4" '
                        f'quat="{fmt((math.cos(cfg.spawn_yaw / 2), 0, 0, math.sin(cfg.spawn_yaw / 2)))}">')
            lines.append(f'{ind}  <freejoint name="root"/>')
            lines.append(f'{ind}  <site name="imu" pos="0 0 0.06" size="0.005" group="4"/>')
        if b.get("joint"):
            jtype = b["joint"].split("_", 1)[1]
            lo, hi = cfg.joint_ranges[jtype]
            lines.append(f'{ind}  <joint name="{b["joint"]}" axis="{fmt(b["axis"])}" range="{lo} {hi}"/>')
        if "inertial" in b:
            i = b["inertial"]
            lines.append(f'{ind}  <inertial pos="{fmt(i["pos"])}" mass="{i["mass"]:.6g}" '
                         f'fullinertia="{fmt(i["fullinertia"])}"/>')
        for k, v in enumerate(b["visuals"]):
            rgba = data["parts"][v["part"]]["rgba"]
            lines.append(f'{ind}  <geom name="{b["name"]}_vis{k}" class="visual" mesh="{v["part"]}" '
                         f'pos="{fmt(v["pos"])}" quat="{fmt(v["quat"])}" rgba="{fmt(rgba)}"/>')
        if "collision_box" in b:
            c = b["collision_box"]
            lines.append(f'{ind}  <geom name="{b["name"]}_col" class="collision" type="box" '
                         f'pos="{fmt(c["pos"])}" size="{fmt(c["size"])}"/>')
        if b["name"] in FOOT_NAMES:
            g, s = taxel_xml(b["name"], sole, tac, ind + "  ")
            lines.append(g)
            sensors.append(s)
        for c in children.get(b["name"], []):
            lines.append(body_xml(c, depth + 1))
        lines.append(f"{ind}</body>")
        return "\n".join(lines)

    tree = "\n".join(body_xml(b, 2) for b in children[None])
    meshes = "\n".join(f'    <mesh name="{n}" file="{p["mesh"]}" scale="0.001 0.001 0.001"/>'
                       for n, p in data["parts"].items())
    actuators = "\n".join(
        f'    <position name="{n}" joint="{n}" ctrlrange="{cfg.joint_ranges[n.split("_", 1)[1]][0]} '
        f'{cfg.joint_ranges[n.split("_", 1)[1]][1]}"/>' for n in JOINT_NAMES)
    f = cfg.floor_friction
    return f"""<mujoco model="biped10">
  <compiler angle="radian" meshdir="{MESH_DIR.as_posix()}" autolimits="true"/>
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
    <!-- links touch the floor (bit 1) but not each other -->
    <default class="collision">
      <geom contype="2" conaffinity="1" group="3" density="0" rgba="0.5 0.5 0.5 0.3"/>
    </default>
    <!-- tactile pads touch the floor and the other foot's pads -->
    <default class="taxel">
      <geom contype="4" conaffinity="5" group="1" density="0" condim="3"
            friction="{f} 0.005 0.0001" solref="0.008 1" solimp="0.9 0.95 0.001" rgba="{fmt(tac.rgba)}"/>
      <site group="4" rgba="1 0.3 0.3 0.25"/>
    </default>
  </default>

  <asset>
{meshes}
    <texture name="grid" type="2d" builtin="checker" rgb1="0.82 0.84 0.86" rgb2="0.68 0.70 0.73"
             width="512" height="512" mark="edge" markrgb="0.55 0.57 0.6"/>
    <material name="grid" texture="grid" texrepeat="10 10" texuniform="true" reflectance="0.05"/>
    <texture name="sky" type="skybox" builtin="gradient" rgb1="0.6 0.75 0.9" rgb2="0.15 0.2 0.3"
             width="512" height="512"/>
  </asset>

  <worldbody>
    <light name="sun" pos="0 0 3" dir="0 0 -1" directional="true" diffuse="0.8 0.8 0.8" castshadow="true"/>
    <geom name="floor" type="plane" size="0 0 0.05" material="grid"
          friction="{f} 0.005 0.0001" contype="1" conaffinity="1"/>
{tree}
  </worldbody>

  <actuator>
{actuators}
  </actuator>

  <sensor>
{chr(10).join(sensors)}
    <framequat name="base_quat" objtype="body" objname="{BASE_BODY}"/>
    <gyro name="gyro" site="imu"/>
    <accelerometer name="accel" site="imu"/>
  </sensor>
</mujoco>
"""


SPEC = RobotSpec(
    name="biped10",
    description="10-DoF MG996R servo-bracket biped (hip yaw, hip roll, hip pitch, knee, ankle roll per leg)",
    config_cls=Biped10Config,
    build_mjcf=build_mjcf,
    joint_names=JOINT_NAMES,
    foot_names=FOOT_NAMES,
    base_body=BASE_BODY,
    forward_local=FORWARD_LOCAL,
    env_defaults={
        "tactile_max_force": 6.0,
        "commands": {"forward_speed": (0.03, 0.12), "yaw_rate": (0.2, 0.5)},
        "rewards": {"energy": 0.05, "lin_sigma": 0.05,
                    "clearance_target": 0.02, "air_time_target": 0.25, "foot_v_ref": 0.1},
        "gait_period": 0.7,
    },
    camera_distance=0.85,
)
