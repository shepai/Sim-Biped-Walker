"""Low-level MuJoCo interface to the biped: build, reset, command servos, read sensors."""

from __future__ import annotations

import math
from typing import Optional, Sequence

import mujoco
import numpy as np

from .robots import DEFAULT_ROBOT, get_robot
from .robots.common import taxel_name


def wrap_angle(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class Biped:
    """A biped robot body in MuJoCo (any robot in :data:`biped_walker.robots.ROBOTS`).

    ``robot="walker4"``: the original 4-DoF walker, joints
    ``(left_hip, left_ankle, right_hip, right_ankle)``.
    ``robot="biped10"``: the 10-DoF servo-bracket biped, joints
    ``left_{hip_yaw, hip_roll, hip_pitch, knee, ankle_roll}`` then ``right_...``.
    ``self.joint_names`` gives the order used by every array here.

    Servos are position actuators, so :meth:`set_targets` takes joint angles
    in radians. :meth:`set_normalized` takes values in ``[-1, 1]`` mapped to
    ``default_pose + value * action_scale`` (for walker4 that spans each
    joint's full range).

    Example::

        bot = Biped("biped10")
        bot.reset()
        for t in range(1000):
            a = np.zeros(bot.n_joints)
            a[0] = np.sin(t * 0.02)
            bot.set_normalized(a)
            bot.step(10)
        print(bot.tactile())        # (2, 3, 5) contact forces in newtons
    """

    def __init__(self, robot: str = DEFAULT_ROBOT, config=None):
        if not isinstance(robot, str):   # old signature: Biped(config)
            robot, config = DEFAULT_ROBOT, robot
        self.spec = get_robot(robot)
        self.robot_name = robot
        self.config = self.spec.make_config(config)
        self.joint_names = tuple(self.spec.joint_names)
        self.foot_names = tuple(self.spec.foot_names)
        self.n_joints = len(self.joint_names)
        self.xml = self.spec.build_mjcf(self.config)
        self.model = mujoco.MjModel.from_xml_string(self.xml)
        self.data = mujoco.MjData(self.model)
        m = self.model

        def _id(obj, name):
            i = mujoco.mj_name2id(m, obj, name)
            if i < 0:
                raise KeyError(f"{name} not found in model")
            return i

        self.base_id = _id(mujoco.mjtObj.mjOBJ_BODY, self.spec.base_body)
        jids = [_id(mujoco.mjtObj.mjOBJ_JOINT, n) for n in self.joint_names]
        self.qpos_idx = np.array([m.jnt_qposadr[j] for j in jids])
        self.dof_idx = np.array([m.jnt_dofadr[j] for j in jids])
        self.act_ids = np.array([_id(mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in self.joint_names])
        self.ctrl_low = m.actuator_ctrlrange[self.act_ids, 0].copy()
        self.ctrl_high = m.actuator_ctrlrange[self.act_ids, 1].copy()
        self.default_pose = np.clip(np.asarray(self.config.default_pose(), float), self.ctrl_low, self.ctrl_high)
        self.action_scale = np.asarray(self.config.action_scale(), float)

        tac = self.config.tactile
        self.tactile_shape = (len(self.foot_names),) + tac.shape  # (2, cols, rows)
        sens_adr = np.zeros(self.tactile_shape, dtype=int)
        geom_ids = np.zeros(self.tactile_shape, dtype=int)
        for f, foot in enumerate(self.foot_names):
            for i in range(tac.cols):
                for j in range(tac.rows):
                    name = taxel_name(foot, i, j)
                    sid = _id(mujoco.mjtObj.mjOBJ_SENSOR, name)
                    sens_adr[f, i, j] = m.sensor_adr[sid]
                    geom_ids[f, i, j] = _id(mujoco.mjtObj.mjOBJ_GEOM, name)
        self.taxel_sensor_adr = sens_adr

        def _sensor(name):
            sid = _id(mujoco.mjtObj.mjOBJ_SENSOR, name)
            adr, dim = m.sensor_adr[sid], m.sensor_dim[sid]
            return np.arange(adr, adr + dim)

        self.gyro_adr = _sensor("gyro")
        self.accel_adr = _sensor("accel")
        self.taxel_geom_ids = geom_ids
        self.foot_body_ids = np.array([_id(mujoco.mjtObj.mjOBJ_BODY, f) for f in self.foot_names])
        self._pad_rgba = m.geom_rgba[geom_ids.ravel()].copy()

        self.total_mass = float(m.body_subtreemass[self.base_id])
        self._forward_local = np.asarray(self.spec.forward_local, dtype=float)

    # ------------------------------------------------------------------ time
    @property
    def timestep(self) -> float:
        return float(self.model.opt.timestep)

    @property
    def time(self) -> float:
        return float(self.data.time)

    # ----------------------------------------------------------------- reset
    def reset(self, joint_positions: Optional[Sequence[float]] = None, yaw: float = 0.0,
              xy: Sequence[float] = (0.0, 0.0), clearance: float = 0.001,
              settle_steps: int = 0) -> None:
        """Reset the robot standing on the floor.

        ``yaw`` is the heading in world coordinates (0 = facing world +x).
        The base height is chosen so the lowest tactile pad sits ``clearance``
        metres above the floor. ``settle_steps`` physics steps are then run
        holding the initial pose.
        """
        m, d = self.model, self.data
        mujoco.mj_resetData(m, d)
        q = self.default_pose.copy() if joint_positions is None else np.asarray(joint_positions, float)
        q = np.clip(q, self.ctrl_low, self.ctrl_high)

        total_yaw = self.config.spawn_yaw + yaw
        d.qpos[0:3] = (xy[0], xy[1], 0.3)
        d.qpos[3:7] = (math.cos(total_yaw / 2), 0.0, 0.0, math.sin(total_yaw / 2))
        d.qpos[self.qpos_idx] = q
        d.ctrl[self.act_ids] = q
        mujoco.mj_forward(m, d)
        d.qpos[2] -= self._lowest_pad_z() - clearance
        mujoco.mj_forward(m, d)
        for _ in range(settle_steps):
            mujoco.mj_step(m, d)

    def _lowest_pad_z(self) -> float:
        ids = self.taxel_geom_ids.ravel()
        pos = self.data.geom_xpos[ids]                       # (n, 3)
        mats = self.data.geom_xmat[ids].reshape(-1, 3, 3)   # (n, 3, 3)
        size = self.model.geom_size[ids]                     # (n, 3)
        signs = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
        corners = pos[:, None, :] + np.einsum("nij,knj->nki", mats, signs[:, None, :] * size[None])
        return float(corners[..., 2].min())

    # --------------------------------------------------------------- control
    def set_targets(self, targets: Sequence[float]) -> np.ndarray:
        """Set servo targets in radians (clipped to the joint ranges)."""
        t = np.clip(np.asarray(targets, dtype=float), self.ctrl_low, self.ctrl_high)
        self.data.ctrl[self.act_ids] = t
        return t

    def normalized_to_targets(self, action: Sequence[float]) -> np.ndarray:
        """``default_pose + action * action_scale``, clipped to the joint ranges."""
        a = np.clip(np.asarray(action, dtype=float), -1.0, 1.0)
        return np.clip(self.default_pose + a * self.action_scale, self.ctrl_low, self.ctrl_high)

    def set_normalized(self, action: Sequence[float]) -> np.ndarray:
        """Set servo targets from values in [-1, 1] (0 = the default pose)."""
        return self.set_targets(self.normalized_to_targets(action))

    def step(self, n: int = 1) -> None:
        for _ in range(n):
            mujoco.mj_step(self.model, self.data)

    # --------------------------------------------------------------- sensors
    def joint_positions(self) -> np.ndarray:
        return self.data.qpos[self.qpos_idx].copy()

    def joint_positions_normalized(self) -> np.ndarray:
        """Joint angles in action units: ``(q - default_pose) / action_scale``."""
        return (self.joint_positions() - self.default_pose) / self.action_scale

    def joint_velocities(self) -> np.ndarray:
        return self.data.qvel[self.dof_idx].copy()

    def actuator_forces(self) -> np.ndarray:
        return self.data.actuator_force[self.act_ids].copy()

    def mechanical_power(self) -> np.ndarray:
        """Absolute mechanical power of each servo, |tau * qdot| (W)."""
        return np.abs(self.actuator_forces() * self.joint_velocities())

    def tactile(self) -> np.ndarray:
        """Normal contact force on each taxel, shape ``(2, cols, rows)`` in newtons.

        Index ``[foot, i, j]``: foot 0 = left, 1 = right; ``i`` across the width
        of the foot (robot left -> right); ``j`` along the foot (heel -> toe).
        """
        return self.data.sensordata[self.taxel_sensor_adr].copy()

    def base_position(self) -> np.ndarray:
        return self.data.xpos[self.base_id].copy()

    def base_rotation(self) -> np.ndarray:
        return self.data.xmat[self.base_id].reshape(3, 3).copy()

    def foot_positions(self) -> np.ndarray:
        """World positions of the foot (sole) bodies, shape (2, 3): left, right."""
        return self.data.xpos[self.foot_body_ids].copy()

    def imu(self) -> np.ndarray:
        """Body IMU: gyro (rad/s, 3) then accelerometer (m/s^2, 3), in the IMU frame.

        The accelerometer measures proper acceleration, so standing still it
        reads about +9.81 m/s^2 along the body's up axis, like a real MPU6050.
        """
        return np.concatenate([self.data.sensordata[self.gyro_adr], self.data.sensordata[self.accel_adr]])

    def foot_forces(self) -> np.ndarray:
        """Total normal force on each foot's tactile array (N), shape (2,)."""
        return self.tactile().sum(axis=(1, 2))

    def com_position(self) -> np.ndarray:
        return self.data.subtree_com[self.base_id].copy()

    def forward_vector(self) -> np.ndarray:
        """Robot forward direction in world coordinates."""
        return self.base_rotation() @ self._forward_local

    def heading(self) -> float:
        """Yaw of the robot's forward direction in the world (0 = +x)."""
        f = self.forward_vector()
        return math.atan2(f[1], f[0])

    def up_z(self) -> float:
        """z-component of the base's up axis (1 = upright, 0 = lying down)."""
        return float(self.base_rotation()[2, 2])

    # ----------------------------------------------------------- rendering
    def colour_taxels(self, forces: Optional[np.ndarray] = None, max_force: float = 1.0) -> None:
        """Tint the tactile pads by their force (blue = none, red = ``max_force``)."""
        f = self.tactile() if forces is None else forces
        x = np.clip(f.ravel() / max_force, 0.0, 1.0)[:, None]
        hot = np.array([0.95, 0.15, 0.1, 1.0])
        self.model.geom_rgba[self.taxel_geom_ids.ravel()] = (1 - x) * self._pad_rgba + x * hot

    def reset_taxel_colours(self) -> None:
        self.model.geom_rgba[self.taxel_geom_ids.ravel()] = self._pad_rgba
