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

from ..robot import Biped, wrap_angle
from ..robots import DEFAULT_ROBOT, get_robot

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
class CurriculumConfig:
    """Robustness randomisation that is brought in gradually during training.

    The randomisation *level* rises linearly from 0 at ``start`` to 1 at ``end``
    (fractions of training progress, which the trainers send to the envs via
    :meth:`BipedWalkerEnv.set_training_progress`). Level 0 is the nominal robot
    on the nominal floor; the ranges below are those used at level 1 and shrink
    proportionally towards nominal at lower levels.
    """

    start: float = 0.3                              # progress where robustness training begins
    end: float = 0.8                                # progress where it reaches full strength
    friction: Tuple[float, float] = (0.4, 1.4)      # floor/foot friction scale
    friction_resample_steps: Optional[int] = 250    # also change the floor friction mid-episode (None = per episode)
    mass_scale: Tuple[float, float] = (0.85, 1.15)  # independent scale on every link's mass and inertia
    payload: Tuple[float, float] = (0.0, 0.25)      # extra mass in the body box, fraction of robot mass
    payload_offset: float = 0.02                    # m, payload centre shifted up to this far in x and y

    def level(self, progress: float) -> float:
        if self.end <= self.start:
            return float(progress >= self.start)
        return float(np.clip((progress - self.start) / (self.end - self.start), 0.0, 1.0))


def curriculum_level(curriculum, progress: float) -> float:
    """Level for ``progress`` given a :class:`CurriculumConfig`, dict or None."""
    if curriculum is None or curriculum is False:
        return 0.0
    cfg = curriculum if isinstance(curriculum, CurriculumConfig) else CurriculumConfig(
        **({} if curriculum is True else curriculum))
    return cfg.level(progress)


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

    # --- gait shaping: feet should lift, swing and land rather than slide ("scooting").
    # A foot is in contact when its tactile array carries > contact_threshold x body weight.
    slip: float = 0.5             # stance feet sliding: sum_contact min(|v_foot| / foot_v_ref, 1)^2
    clearance: float = 0.5        # moving feet kept low: sum (1 - min(h / clearance_target, 1))^2 * min(|v_foot| / foot_v_ref, 1)
    air_time: float = 1.0         # each touchdown: min(air time, 2 x target) - air_time_target
    gait: float = 0.5             # feet match the gait clock (only with gait_clock=True)
    contact_threshold: float = 0.05
    clearance_target: float = 0.02   # m, swing foot height above its standing height
    air_time_target: float = 0.25    # s
    foot_v_ref: float = 0.1          # m/s, foot speed scale


class BipedWalkerEnv(gym.Env):
    """Command-conditioned locomotion for the tactile-footed bipeds.

    ``robot`` picks the body: ``"walker4"`` (4-DoF walker, the default) or
    ``"biped10"`` (10-DoF servo-bracket biped).

    **Action** (n_joints,): servo targets in ``[-1, 1]``, mapped to
    ``default_pose + action * action_scale`` (see ``self.robot.joint_names``
    for the order).

    **Observation**: the last ``history_len`` frames of

    * tactile forces, 2 feet x 3 x 5 taxels = 30 values, scaled by
      ``tactile_max_force`` and clipped to [0, 1];
    * the motor positions in the same normalised units as the action;
    * the body IMU (``include_imu``): gyro / 4 rad/s and accelerometer / g,
      3 axes each, in the body frame;

    followed (if ``include_command``) by the 2 normalised command values
    ``(forward speed, yaw rate)``. The command is needed so one policy can be
    told whether to walk or turn; set ``include_command=False`` and pass a
    fixed ``command`` if you want the observation to be exactly tactile +
    motor positions. A single frame can't reveal velocity, which is why a short
    history is stacked by default.

    **Gait shaping** (to stop the policy shuffling/scooting): foot contact is read
    from the tactile arrays; stance feet are penalised for sliding, moving feet
    for staying close to the floor, and each touchdown after a proper swing is
    rewarded. With ``gait_clock=True`` a left/right stepping clock of period
    ``gait_period`` (robot default if None) is added to the observation as
    (sin, cos) and the feet are rewarded for matching it (each foot in stance
    for ``stance_fraction`` of the cycle, half a cycle apart).
    ``action_smoothing`` in [0, 1) low-pass filters the servo targets.

    **Robustness curriculum** (``curriculum=True`` or a :class:`CurriculumConfig`
    / dict): floor friction (also changing mid-episode), link masses and a
    payload in the body box are randomised, ramping from none to full strength
    as training progresses (the trainers call :meth:`set_training_progress`).
    ``randomization_level`` fixes the level instead, e.g. ``1.0`` to evaluate
    robustness.

    **Reward**: tracking of the commanded forward speed and yaw rate, minus
    the absolute mechanical power of the servos (efficiency), action jerk,
    sideways drift and body tilt, with a penalty for falling over.
    """

    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 50}

    def __init__(
        self,
        robot: str = DEFAULT_ROBOT,
        task: str = "mixed",
        command: Optional[Sequence[float]] = None,
        frame_skip: int = 10,
        history_len: int = 3,
        include_command: bool = True,
        gait_clock: bool = False,
        gait_period: Optional[float] = None,
        stance_fraction: float = 0.6,
        action_smoothing: float = 0.0,
        tactile_max_force: Optional[float] = None,
        tactile_noise: float = 0.0,
        joint_noise: float = 0.0,
        include_imu: bool = True,
        imu_noise: float = 0.0,
        curriculum: Optional[CurriculumConfig | Dict[str, Any] | bool] = None,
        randomization_level: Optional[float] = None,
        reset_noise: float = 0.05,
        randomize_dynamics: bool = False,
        max_tilt: float = 0.9,
        commands: Optional[CommandConfig | Dict[str, Any]] = None,
        rewards: Optional[RewardConfig | Dict[str, Any]] = None,
        model_config: Optional[Any] = None,
        render_mode: Optional[str] = None,
        width: int = 640,
        height: int = 480,
        colour_taxels: bool = True,
    ):
        if task not in TASKS:
            raise ValueError(f"task must be one of {TASKS}, got {task!r}")
        self.task = task
        self.robot_name = robot
        spec = get_robot(robot)
        defaults = spec.env_defaults
        self.fixed_command = None if command is None else np.asarray(command, dtype=float)
        self.frame_skip = int(frame_skip)
        self.history_len = int(history_len)
        self.include_command = include_command
        self.tactile_max_force = float(tactile_max_force if tactile_max_force is not None
                                       else defaults.get("tactile_max_force", 1.5))
        self.tactile_noise = float(tactile_noise)
        self.joint_noise = float(joint_noise)
        self.reset_noise = float(reset_noise)
        self.randomize_dynamics = randomize_dynamics
        self.include_imu = bool(include_imu)
        self.imu_noise = float(imu_noise)
        if curriculum is True:
            curriculum = CurriculumConfig()
        elif isinstance(curriculum, dict):
            curriculum = CurriculumConfig(**curriculum)
        self.curriculum: Optional[CurriculumConfig] = curriculum or None
        # a fixed level overrides the curriculum; randomize_dynamics=True (old flag) means full strength
        self.fixed_level = (float(randomization_level) if randomization_level is not None
                            else 1.0 if randomize_dynamics else None)
        if self.fixed_level is not None and self.curriculum is None:
            self.curriculum = CurriculumConfig()
        self.training_progress = 0.0
        self._rand = {"level": 0.0, "friction": 1.0, "mass_scale": 1.0, "payload": 0.0}
        self.min_up_z = math.cos(max_tilt)
        self.cmd_cfg = (commands if isinstance(commands, CommandConfig)
                        else CommandConfig(**{**defaults.get("commands", {}), **(commands or {})}))
        self.rew_cfg = (rewards if isinstance(rewards, RewardConfig)
                        else RewardConfig(**{**defaults.get("rewards", {}), **(rewards or {})}))

        self.robot = Biped(robot, model_config)
        self.camera_distance = spec.camera_distance
        self.model, self.data = self.robot.model, self.robot.data
        self.dt = self.robot.timestep * self.frame_skip
        self.metadata = {**self.metadata, "render_fps": int(round(1.0 / self.dt))}

        self.n_taxels = int(np.prod(self.robot.tactile_shape))
        self.n_joints = self.robot.n_joints
        self.n_imu = 6 if include_imu else 0
        self.frame_size = self.n_taxels + self.n_joints + self.n_imu
        self.gait_clock = bool(gait_clock)
        self.gait_period = float(gait_period or defaults.get("gait_period", 0.7))
        self.stance_fraction = float(stance_fraction)
        self.action_smoothing = float(action_smoothing)
        obs_dim = (self.frame_size * self.history_len + (2 if include_command else 0)
                   + (2 if self.gait_clock else 0))
        self.observation_space = spaces.Box(-np.inf, np.inf, shape=(obs_dim,), dtype=np.float32)
        self.action_space = spaces.Box(-1.0, 1.0, shape=(self.n_joints,), dtype=np.float32)

        self._cmd_scale = np.array([max(self.cmd_cfg.forward_speed[1], 1e-6),
                                    max(self.cmd_cfg.yaw_rate[1], 1e-6)])
        self._nominal_friction = self.model.geom_friction.copy()
        self._nominal_mass = self.model.body_mass.copy()
        self._nominal_inertia = self.model.body_inertia.copy()
        self._nominal_ipos = self.model.body_ipos.copy()

        self.render_mode = render_mode
        self.width, self.height = width, height
        self.colour_taxels = colour_taxels
        self._viewer = None
        self._renderer = None
        self._camera = None

        self.command = np.zeros(2)
        self._history: deque = deque(maxlen=self.history_len)
        self._prev_action = np.zeros(self.n_joints)
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
        if not self.include_imu:
            return np.concatenate([tac, q])
        imu = self.robot.imu() / np.array([4.0, 4.0, 4.0, 9.81, 9.81, 9.81])   # gyro / 4 rad/s, accel / g
        if self.imu_noise > 0:
            imu = imu + self.np_random.normal(0.0, self.imu_noise, imu.shape)
        return np.concatenate([tac, q, imu])

    def _obs(self) -> np.ndarray:
        parts = list(self._history)
        if self.include_command:
            parts.append(self.command / self._cmd_scale)
        if self.gait_clock:
            ang = 2 * math.pi * self._phase
            parts.append(np.array([math.sin(ang), math.cos(ang)]))
        return np.concatenate(parts).astype(np.float32)

    def obs_layout(self) -> Dict[str, slice]:
        """Slices of the observation vector (for the most recent frame)."""
        start = self.frame_size * (self.history_len - 1)
        out = {
            "tactile": slice(start, start + self.n_taxels),
            "joints": slice(start + self.n_taxels, start + self.n_taxels + self.n_joints),
        }
        if self.include_imu:
            out["imu"] = slice(start + self.n_taxels + self.n_joints, start + self.frame_size)
        end = self.frame_size * self.history_len
        if self.include_command:
            out["command"] = slice(end, end + 2)
            end += 2
        if self.gait_clock:
            out["gait_clock"] = slice(end, end + 2)
        return out

    # ------------------------------------------------------ robustness curriculum
    def set_training_progress(self, progress: float) -> float:
        """Tell the env how far training is (0..1); returns the randomisation level.

        Called by the trainers (``env_method`` for vector envs); the level takes
        effect from the next reset.
        """
        self.training_progress = float(progress)
        return self.randomization_level

    @property
    def randomization_level(self) -> float:
        if self.fixed_level is not None:
            return self.fixed_level
        return self.curriculum.level(self.training_progress) if self.curriculum else 0.0

    @staticmethod
    def _scaled(rng, lo_hi, level):
        lo, hi = lo_hi
        return rng.uniform(1 + level * (lo - 1), 1 + level * (hi - 1))

    def _set_friction(self, rng, level):
        f = self._scaled(rng, self.curriculum.friction, level) if level > 0 else 1.0
        # MuJoCo uses the larger friction of the two geoms in contact, so scale them all
        self.model.geom_friction[:, 0] = self._nominal_friction[:, 0] * f
        self._rand["friction"] = f

    def _apply_randomization(self, rng) -> None:
        level = self.randomization_level
        m = self.model
        m.body_mass[:] = self._nominal_mass
        m.body_inertia[:] = self._nominal_inertia
        m.body_ipos[:] = self._nominal_ipos
        self._rand.update(level=level, friction=1.0, mass_scale=1.0, payload=0.0)
        if level <= 0 or self.curriculum is None:
            m.geom_friction[:] = self._nominal_friction
        else:
            c = self.curriculum
            self._set_friction(rng, level)
            scale = np.array([self._scaled(rng, c.mass_scale, level) for _ in range(m.nbody)])
            scale[0] = 1.0                                   # world body
            m.body_mass[:] = self._nominal_mass * scale
            m.body_inertia[:] = self._nominal_inertia * scale[:, None]
            # payload in the body box: extra mass at an offset centre
            b = self.robot.base_id
            total = float(self._nominal_mass.sum())
            p = rng.uniform(*c.payload) * level * total
            if p > 0:
                m0 = m.body_mass[b]
                off = np.array([*rng.uniform(-1, 1, 2) * c.payload_offset * level, 0.0])
                m.body_ipos[b] = (m0 * self._nominal_ipos[b] + p * (self._nominal_ipos[b] + off)) / (m0 + p)
                m.body_inertia[b] *= (m0 + p) / m0
                m.body_mass[b] = m0 + p
            self._rand.update(mass_scale=float(m.body_mass[1:].sum() / self._nominal_mass[1:].sum()),
                              payload=float(p))
        mujoco.mj_setConst(m, self.data)
        self.robot.total_mass = float(m.body_subtreemass[self.robot.base_id])

    # ---------------------------------------------------------------- reset
    def reset(self, *, seed: Optional[int] = None, options: Optional[Dict[str, Any]] = None):
        super().reset(seed=seed)
        options = options or {}
        rng = self.np_random
        self._apply_randomization(rng)

        noise = rng.uniform(-self.reset_noise, self.reset_noise, self.n_joints) * self.robot.action_scale
        self.robot.reset(joint_positions=self.robot.default_pose + noise, settle_steps=50)
        mujoco.mj_forward(self.model, self.data)

        if "command" in options:
            self.command = np.asarray(options["command"], dtype=float)
        else:
            self.command = self.sample_command()

        self._steps = 0
        self._energy = 0.0
        self._ep = {"return": 0.0, "v": 0.0, "v_err": 0.0, "w": 0.0, "w_err": 0.0, "power": 0.0, "fell": 0.0}
        self._ep_terms: Dict[str, float] = {}
        self._prev_action = self.robot.joint_positions_normalized()
        self._action = self._prev_action.copy()            # smoothed action actually applied
        self._phase = float(rng.random()) if self.gait_clock else 0.0
        self._weight = self.robot.total_mass * 9.81
        self._contact = self.robot.foot_forces() > self.rew_cfg.contact_threshold * self._weight
        self._air = np.zeros(2)
        self._foot_ref_z = self.robot.foot_positions()[:, 2].copy()
        self._gait = {"double": 0, "single": 0, "flight": 0, "touchdowns": 0, "slip": 0.0, "slip_n": 0,
                      "swing_h": 0.0, "swing_n": 0, "air": 0.0}
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
        feet0 = r.foot_positions()

        if self.action_smoothing > 0:
            self._action = self.action_smoothing * self._action + (1 - self.action_smoothing) * a
        else:
            self._action = a
        r.set_normalized(self._action)
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
        # ---- feet: contact from the tactile arrays, speed and height from the sole bodies
        feet1 = r.foot_positions()
        foot_speed = np.linalg.norm(feet1[:, :2] - feet0[:, :2], axis=1) / self.dt
        height = np.maximum(feet1[:, 2] - self._foot_ref_z, 0.0)
        contact = r.foot_forces() > c.contact_threshold * self._weight
        touchdown = contact & ~self._contact
        self._air += self.dt
        air_at_touchdown = self._air[touchdown].copy()
        self._air[contact] = 0.0
        self._contact = contact
        moving = abs(self.command[0]) > 0.005 or abs(self.command[1]) > 0.05
        speed_n = np.minimum(foot_speed / c.foot_v_ref, 1.0)
        gait_terms = {
            "slip": -c.slip * float(np.sum(contact * speed_n ** 2)),
            "clearance": -c.clearance * float(np.sum((1 - np.minimum(height / c.clearance_target, 1.0)) ** 2
                                                     * speed_n * ~contact)),
            "air_time": c.air_time * float(np.sum(np.minimum(air_at_touchdown, 2 * c.air_time_target)
                                                  - c.air_time_target)) * moving,
        }
        if self.gait_clock:
            self._phase = (self._phase + self.dt / self.gait_period) % 1.0
            if moving:
                want = np.array([self._phase < self.stance_fraction,
                                 (self._phase + 0.5) % 1.0 < self.stance_fraction])
            else:
                want = np.array([True, True])
            gait_terms["gait"] = c.gait * float(np.mean(contact == want))
        g = self._gait
        n_contact = int(contact.sum())
        g["double" if n_contact == 2 else "single" if n_contact == 1 else "flight"] += 1
        g["touchdowns"] += int(touchdown.sum())
        g["air"] += float(air_at_touchdown.sum())
        g["slip"] += float(np.sum(foot_speed * contact))
        g["slip_n"] += n_contact
        g["swing_h"] += float(np.sum(height * ~contact))
        g["swing_n"] += 2 - n_contact

        terms = {
            "lin_vel": c.lin_vel * math.exp(-((v_fwd - self.command[0]) / c.lin_sigma) ** 2),
            "yaw_vel": c.yaw_vel * math.exp(-((yaw_rate - self.command[1]) / c.yaw_sigma) ** 2),
            "energy": -c.energy * power,
            "action_rate": -c.action_rate * float(np.sum((a - self._prev_action) ** 2)),
            "lateral": -c.lateral * v_lat ** 2,
            "tilt": -c.tilt * (1.0 - up_z),
            **gait_terms,
        }
        fallen = up_z < self.min_up_z or r.base_position()[2] < 0.6 * self._start_base_z
        if fallen:
            terms["fall"] = -c.fall
        reward = float(sum(terms.values()))
        ep = self._ep
        ep["return"] += reward
        ep["v"] += v_fwd
        ep["v_err"] += abs(v_fwd - self.command[0])
        ep["w"] += yaw_rate
        ep["w_err"] += abs(yaw_rate - self.command[1])
        ep["power"] += power
        ep["fell"] = float(fallen)
        for k, v in terms.items():
            self._ep_terms[k] = self._ep_terms.get(k, 0.0) + v
        self._prev_action = a

        if self.cmd_cfg.resample_steps and self._steps % self.cmd_cfg.resample_steps == 0:
            self.command = self.sample_command()
        c_cfg = self.curriculum
        if (c_cfg is not None and self._rand["level"] > 0 and c_cfg.friction_resample_steps
                and self._steps % c_cfg.friction_resample_steps == 0):
            self._set_friction(self.np_random, self._rand["level"])   # the floor changes underfoot

        self._history.append(self._frame())
        info = self._info(v_fwd, v_lat, yaw_rate, power)
        info["reward_terms"] = terms
        info["foot_contact"] = contact.copy()
        info["episode_summary"] = self.episode_summary()
        if self.render_mode == "human":
            self.render()
        return self._obs(), reward, bool(fallen), False, info

    def episode_summary(self) -> Dict[str, float]:
        """Running totals for the current episode (complete once it ends).

        Logged per episode by the trainers: return, length, distance, energy,
        cost of transport, fell, mean velocities and their tracking errors
        against the command, mean power, and the sum of every reward term (``r_*``).
        """
        n = max(self._steps, 1)
        ep = self._ep
        dist = float(np.linalg.norm(self.robot.com_position()[:2] - self._start_xy))
        weight = self.robot.total_mass * 9.81
        out = {
            "return": ep["return"], "length": self._steps, "time_s": self._steps * self.dt,
            "distance": dist, "energy": self._energy,
            "cost_of_transport": self._energy / (weight * dist) if dist > 1e-3 else float("nan"),
            "fell": ep["fell"],
            "cmd_forward": float(self.command[0]), "cmd_yaw_rate": float(self.command[1]),
            "mean_forward_velocity": ep["v"] / n, "forward_velocity_error": ep["v_err"] / n,
            "mean_yaw_rate": ep["w"] / n, "yaw_rate_error": ep["w_err"] / n,
            "mean_power": ep["power"] / n,
            # gait diagnostics: a shuffle shows up as high double support, few touchdowns and slip
            "double_support_frac": self._gait["double"] / n,
            "single_support_frac": self._gait["single"] / n,
            "flight_frac": self._gait["flight"] / n,
            "step_rate": self._gait["touchdowns"] / max(self._steps * self.dt, 1e-9),
            "mean_air_time": self._gait["air"] / max(self._gait["touchdowns"], 1),
            "stance_slip": self._gait["slip"] / max(self._gait["slip_n"], 1),
            "swing_height": self._gait["swing_h"] / max(self._gait["swing_n"], 1),
            # robustness curriculum
            "rand_level": self._rand["level"],
            "friction_scale": self._rand["friction"],
            "mass_scale": self._rand["mass_scale"],
            "payload_kg": self._rand["payload"],
        }
        out.update({f"r_{k}": v for k, v in self._ep_terms.items()})
        return out

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
        cam.distance = self.camera_distance
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
                cam.distance, cam.azimuth, cam.elevation = self.camera_distance, 135.0, -20.0
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
        return {"robot": self.robot_name, "task": self.task, "include_imu": self.include_imu,
                "curriculum": asdict(self.curriculum) if self.curriculum else None,
                "randomization_level": self.fixed_level, "gait_clock": self.gait_clock,
                "gait_period": self.gait_period, "action_smoothing": self.action_smoothing, "commands": asdict(self.cmd_cfg), "rewards": asdict(self.rew_cfg)}
