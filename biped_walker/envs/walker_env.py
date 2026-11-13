"""Gymnasium environment: energy-efficient forward walking and turning."""

from __future__ import annotations

import math
from collections import deque
from dataclasses import asdict, dataclass, field
from typing import Any, Dict, Optional, Sequence, Tuple

import gymnasium as gym
import mujoco
import numpy as np
from gymnasium import spaces

from ..model import JOINT_NAMES, ModelConfig
from ..robot import Biped, wrap_angle

TASKS = ("forward", "turn", "mixed")


@dataclass
class CommandConfig:
    """Ranges the velocity commands are sampled from.

    ``task="forward"``: walk straight ahead at a speed from ``forward_speed``.
    ``task="turn"``:    turn on the spot at a yaw rate of +/- ``yaw_rate``.
    ``task="mixed"``:   a mix of forward, turning in place and walking in arcs.
    """

    forward_speed: Tuple[float, float] = (0.02, 0.08)   # m/s
    yaw_rate: Tuple[float, float] = (0.2, 0.6)          # rad/s, magnitude
    p_forward: float = 0.4                              # mixed-task probabilities
    p_turn: float = 0.3                                 # (rest: arcs)
    resample_steps: Optional[int] = None                # resample mid-episode every N steps


@dataclass
class RewardConfig:
    """Weights of the reward terms (all per control step)."""

    lin_vel: float = 1.0          # exp tracking of forward speed
    lin_sigma: float = 0.03       # m/s
    yaw_vel: float = 0.5          # exp tracking of yaw rate
    yaw_sigma: float = 0.25       # rad/s
    energy: float = 0.2           # per watt of absolute mechanical servo power
    action_rate: float = 0.02     # * ||a_t - a_{t-1}||^2
    lateral: float = 20.0         # * (sideways velocity)^2
    tilt: float = 0.5             # * (1 - up_z)
    fall: float = 5.0             # one-off penalty on falling


class BipedWalkerEnv(gym.Env):
    """Command-conditioned locomotion for the 4-DoF tactile-footed biped.

    **Action** (4,): servo targets in ``[-1, 1]``, mapped to each joint's
    range, in order ``(left_hip, left_ankle, right_hip, right_ankle)``.

    **Observation**: the last ``history_len`` frames of

    * tactile forces, 2 feet x 3 x 5 taxels = 30 values, scaled by
      ``tactile_max_force`` and clipped to [0, 1];
    * the 4 motor positions, normalised to [-1, 1] over the joint ranges;

    followed (if ``include_command``) by the 2 normalised command values
    ``(forward speed, yaw rate)``. The command is needed so one policy can be
    told whether to walk or turn; set ``include_command=False`` and pass a
    fixed ``command`` if you want the observation to be exactly tactile +
    motor positions. A single frame can't reveal velocity, which is why a short
    history is stacked by default.

    **Reward**: tracking of the commanded forward speed and yaw rate, minus
    the absolute mechanical power of the servos (efficiency), action jerk,
    sideways drift and body tilt, with a penalty for falling over.
    """

    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 50}

    def __init__(
        self,
        task: str = "mixed",
        command: Optional[Sequence[float]] = None,
        frame_skip: int = 10,
        history_len: int = 3,
        include_command: bool = True,
        tactile_max_force: float = 1.5,
        tactile_noise: float = 0.0,
        joint_noise: float = 0.0,
        reset_noise: float = 0.05,
        randomize_dynamics: bool = False,
        max_tilt: float = 0.9,
        commands: Optional[CommandConfig | Dict[str, Any]] = None,
        rewards: Optional[RewardConfig | Dict[str, Any]] = None,
        model_config: Optional[ModelConfig | Dict[str, Any]] = None,
        render_mode: Optional[str] = None,
        width: int = 640,
        height: int = 480,
        colour_taxels: bool = True,
    ):
        if task not in TASKS:
            raise ValueError(f"task must be one of {TASKS}, got {task!r}")
        self.task = task
        self.fixed_command = None if command is None else np.asarray(command, dtype=float)
        self.frame_skip = int(frame_skip)
        self.history_len = int(history_len)
        self.include_command = include_command
        self.tactile_max_force = float(tactile_max_force)
        self.tactile_noise = float(tactile_noise)
        self.joint_noise = float(joint_noise)
        self.reset_noise = float(reset_noise)
        self.randomize_dynamics = randomize_dynamics
        self.min_up_z = math.cos(max_tilt)
        self.cmd_cfg = commands if isinstance(commands, CommandConfig) else CommandConfig(**(commands or {}))
        self.rew_cfg = rewards if isinstance(rewards, RewardConfig) else RewardConfig(**(rewards or {}))

        if isinstance(model_config, dict):
            model_config = ModelConfig.from_dict(model_config)
        self.robot = Biped(model_config)
        self.model, self.data = self.robot.model, self.robot.data
        self.dt = self.robot.timestep * self.frame_skip
        self.metadata = {**self.metadata, "render_fps": int(round(1.0 / self.dt))}

        self.n_taxels = int(np.prod(self.robot.tactile_shape))
        self.frame_size = self.n_taxels + len(JOINT_NAMES)
        obs_dim = self.frame_size * self.history_len + (2 if include_command else 0)
        self.observation_space = spaces.Box(-np.inf, np.inf, shape=(obs_dim,), dtype=np.float32)
        self.action_space = spaces.Box(-1.0, 1.0, shape=(len(JOINT_NAMES),), dtype=np.float32)

        self._cmd_scale = np.array([max(self.cmd_cfg.forward_speed[1], 1e-6),
                                    max(self.cmd_cfg.yaw_rate[1], 1e-6)])
        self._nominal_friction = self.model.geom_friction.copy()
        self._nominal_mass = self.model.body_mass.copy()
        self._nominal_inertia = self.model.body_inertia.copy()

        self.render_mode = render_mode
        self.width, self.height = width, height
        self.colour_taxels = colour_taxels
        self._viewer = None
        self._renderer = None
        self._camera = None

        self.command = np.zeros(2)
        self._history: deque = deque(maxlen=self.history_len)
        self._prev_action = np.zeros(len(JOINT_NAMES))
        self._steps = 0
        self._energy = 0.0
        self._start_xy = np.zeros(2)
        self._start_base_z = 0.0
        self._last_tactile = np.zeros(self.robot.tactile_shape)

    # ------------------------------------------------------------- commands
    def sample_command(self) -> np.ndarray:
        if self.fixed_command is not None:
            return self.fixed_command.copy()
        c, rng = self.cmd_cfg, self.np_random
        v = rng.uniform(*c.forward_speed)
        w = rng.uniform(*c.yaw_rate) * rng.choice([-1.0, 1.0])
        if self.task == "forward":
            return np.array([v, 0.0])
        if self.task == "turn":
            return np.array([0.0, w])
        u = rng.random()
        if u < c.p_forward:
            return np.array([v, 0.0])
        if u < c.p_forward + c.p_turn:
            return np.array([0.0, w])
        return np.array([v, w])

    def set_command(self, forward_speed: float, yaw_rate: float) -> None:
        """Fix the command (m/s, rad/s) from now on, e.g. for interactive demos."""
        self.fixed_command = np.array([forward_speed, yaw_rate], dtype=float)
        self.command = self.fixed_command.copy()

    # ---------------------------------------------------------- observation
    def _frame(self) -> np.ndarray:
        tac = self.robot.tactile()
        self._last_tactile = tac
        if self.tactile_noise > 0:
            tac = tac + self.np_random.normal(0.0, self.tactile_noise, tac.shape)
        tac = np.clip(tac / self.tactile_max_force, 0.0, 1.0).ravel()
        q = self.robot.joint_positions_normalized()
        if self.joint_noise > 0:
            q = q + self.np_random.normal(0.0, self.joint_noise, q.shape)
        return np.concatenate([tac, q])

    def _obs(self) -> np.ndarray:
        parts = list(self._history)
        if self.include_command:
            parts.append(self.command / self._cmd_scale)
        return np.concatenate(parts).astype(np.float32)

    def obs_layout(self) -> Dict[str, slice]:
        """Slices of the observation vector (for the most recent frame)."""
        start = self.frame_size * (self.history_len - 1)
        out = {
            "tactile": slice(start, start + self.n_taxels),
            "joints": slice(start + self.n_taxels, start + self.frame_size),
        }
        if self.include_command:
            out["command"] = slice(self.frame_size * self.history_len, None)
        return out

    # ---------------------------------------------------------------- reset
    def reset(self, *, seed: Optional[int] = None, options: Optional[Dict[str, Any]] = None):
        super().reset(seed=seed)
        options = options or {}
        rng = self.np_random
        if self.randomize_dynamics:
            self.model.geom_friction[:] = self._nominal_friction * rng.uniform(0.7, 1.3)
            scale = rng.uniform(0.85, 1.15, size=self.model.nbody)
            self.model.body_mass[:] = self._nominal_mass * scale
            self.model.body_inertia[:] = self._nominal_inertia * scale[:, None]

        q0 = rng.uniform(-self.reset_noise, self.reset_noise, len(JOINT_NAMES))
        self.robot.reset(joint_positions=q0 * self.robot.ctrl_high, settle_steps=50)
        mujoco.mj_forward(self.model, self.data)

        if "command" in options:
            self.command = np.asarray(options["command"], dtype=float)
        else:
            self.command = self.sample_command()

        self._steps = 0
        self._energy = 0.0
        self._prev_action = self.robot.joint_positions_normalized()
        self._start_xy = self.robot.com_position()[:2]
        self._start_base_z = self.robot.base_position()[2]
        frame = self._frame()
        self._history.clear()
        for _ in range(self.history_len):
            self._history.append(frame)
        if self.render_mode == "human":
            self.render()
        return self._obs(), self._info(0.0, 0.0, 0.0, 0.0)

    # ----------------------------------------------------------------- step
    def step(self, action):
        a = np.clip(np.asarray(action, dtype=float), -1.0, 1.0)
        r = self.robot
        com0 = r.com_position()[:2]
        h0 = r.heading()

        r.set_normalized(a)
        energy = 0.0
        for _ in range(self.frame_skip):
            mujoco.mj_step(self.model, self.data)
            energy += float(r.mechanical_power().sum()) * r.timestep
        mujoco.mj_forward(self.model, self.data)
        self._energy += energy
        self._steps += 1

        com1 = r.com_position()[:2]
        h1 = r.heading()
        dh = wrap_angle(h1 - h0)
        hm = h0 + 0.5 * dh
        disp = com1 - com0
        v_fwd = (disp[0] * math.cos(hm) + disp[1] * math.sin(hm)) / self.dt
        v_lat = (-disp[0] * math.sin(hm) + disp[1] * math.cos(hm)) / self.dt
        yaw_rate = dh / self.dt
        power = energy / self.dt
        up_z = r.up_z()

        c = self.rew_cfg
        terms = {
            "lin_vel": c.lin_vel * math.exp(-((v_fwd - self.command[0]) / c.lin_sigma) ** 2),
            "yaw_vel": c.yaw_vel * math.exp(-((yaw_rate - self.command[1]) / c.yaw_sigma) ** 2),
            "energy": -c.energy * power,
            "action_rate": -c.action_rate * float(np.sum((a - self._prev_action) ** 2)),
            "lateral": -c.lateral * v_lat ** 2,
            "tilt": -c.tilt * (1.0 - up_z),
        }
        fallen = up_z < self.min_up_z or r.base_position()[2] < 0.6 * self._start_base_z
        if fallen:
            terms["fall"] = -c.fall
        reward = float(sum(terms.values()))
        self._prev_action = a

        if self.cmd_cfg.resample_steps and self._steps % self.cmd_cfg.resample_steps == 0:
            self.command = self.sample_command()

        self._history.append(self._frame())
        info = self._info(v_fwd, v_lat, yaw_rate, power)
        info["reward_terms"] = terms
        if self.render_mode == "human":
            self.render()
        return self._obs(), reward, bool(fallen), False, info

    def _info(self, v_fwd, v_lat, yaw_rate, power) -> Dict[str, Any]:
        dist = float(np.linalg.norm(self.robot.com_position()[:2] - self._start_xy))
        weight = self.robot.total_mass * 9.81
        return {
            "command": self.command.copy(),
            "forward_velocity": v_fwd,
            "lateral_velocity": v_lat,
            "yaw_rate": yaw_rate,
            "power": power,
            "energy": self._energy,
            "distance": dist,
            "cost_of_transport": self._energy / (weight * dist) if dist > 1e-3 else float("inf"),
            "heading": self.robot.heading(),
            "position": self.robot.com_position(),
            "tactile": self._last_tactile.copy(),
            "joint_positions": self.robot.joint_positions(),
        }

    # --------------------------------------------------------------- render
    def _make_camera(self):
        cam = mujoco.MjvCamera()
        cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        cam.trackbodyid = self.robot.base_id
        cam.distance = 0.6
        cam.azimuth = 135.0
        cam.elevation = -20.0
        return cam

    def render_rgb(self) -> np.ndarray:
        """Offscreen RGB frame from the tracking camera, whatever ``render_mode`` is."""
        if self._renderer is None:
            self._renderer = mujoco.Renderer(self.model, self.height, self.width)
            self._camera = self._make_camera()
        if self.colour_taxels:
            self.robot.colour_taxels(max_force=self.tactile_max_force)
        self._renderer.update_scene(self.data, camera=self._camera)
        return self._renderer.render().copy()

    def render(self):
        if self.render_mode == "rgb_array":
            return self.render_rgb()
        if self.render_mode == "human":
            if self._viewer is None:
                import mujoco.viewer  # noqa: F401  (needs a display)
                self._viewer = mujoco.viewer.launch_passive(self.model, self.data)
                cam = self._viewer.cam
                cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
                cam.trackbodyid = self.robot.base_id
                cam.distance, cam.azimuth, cam.elevation = 0.6, 135.0, -20.0
            if self.colour_taxels:
                self.robot.colour_taxels(max_force=self.tactile_max_force)
            self._viewer.sync()
        return None

    @property
    def viewer_running(self) -> bool:
        return self._viewer is not None and self._viewer.is_running()

    def close(self):
        if self._viewer is not None:
            self._viewer.close()
            self._viewer = None
        if self._renderer is not None:
            if hasattr(self._renderer, "close"):
                self._renderer.close()
            self._renderer = None

    def config_dict(self) -> Dict[str, Any]:
        return {"task": self.task, "commands": asdict(self.cmd_cfg), "rewards": asdict(self.rew_cfg)}
