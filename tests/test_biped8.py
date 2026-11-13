"""Checks on the 8-DoF servo-bracket biped: CAD data, layout and MJCF."""

import json
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from biped_walker.cad.biped8_design import STAND_POSE, forward_kinematics
from biped_walker.cad.mesh import mass_properties, read_stl
from biped_walker.robots import ROBOTS, build_mjcf
from biped_walker.robots.biped8 import JOINT_NAMES, MESH_DIR, MODEL_JSON, Biped8Config


@pytest.fixture(scope="module")
def model():
    return json.loads(MODEL_JSON.read_text())


def test_registry_has_both_robots():
    assert set(ROBOTS) >= {"walker4", "biped8"}
    assert len(ROBOTS["biped8"].joint_names) == 8


def test_mjcf_valid(model):
    root = ET.fromstring(build_mjcf("biped8"))
    assert len(root.findall("./sensor/touch")) == 30
    assert [a.get("name") for a in root.findall("./actuator/position")] == list(JOINT_NAMES)
    assert [j.get("name") for j in root.find("worldbody").iter("joint")] == list(JOINT_NAMES)
    meshes = {m.get("name") for m in root.findall("./asset/mesh")}
    assert all(g.get("mesh") in meshes for g in root.iter("geom") if g.get("mesh"))


def test_meshes_closed_and_outward(model):
    for name, part in model["parts"].items():
        mesh = read_stl(MESH_DIR / part["mesh"])
        assert mass_properties(mesh)["volume"] > 0, name


def test_mass_is_plausible(model):
    # 8 x 55 g servos + brackets + body + battery
    assert 1.1 < model["total_mass"] < 1.8


def test_inertias_physical(model):
    for b in model["bodies"]:
        if "inertial" not in b:
            continue
        ixx, iyy, izz, ixy, ixz, iyz = b["inertial"]["fullinertia"]
        I = np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]])
        ev = np.linalg.eigvalsh(I)
        assert ev.min() > 0, b["name"]
        assert ev[0] + ev[1] >= ev[2] * (1 - 1e-6), b["name"]


def test_standing_pose_feet_flat_and_symmetric(model):
    world = forward_kinematics(model, STAND_POSE)
    (Rl, tl), (Rr, tr) = world["left_foot"], world["right_foot"]
    assert np.allclose(Rl, np.eye(3), atol=1e-9) and np.allclose(Rr, np.eye(3), atol=1e-9)
    assert np.isclose(tl[2], tr[2]) and np.isclose(tl[0], tr[0]) and np.isclose(tl[1], -tr[1])
    # bent knees lower the hips compared with straight legs
    assert world["torso"][1][2] - tl[2] < -forward_kinematics(model, {})["left_foot"][1][2]


def test_mirrored_joints_move_symmetrically(model):
    q = {"left_hip_roll": 0.3, "right_hip_roll": 0.3, "left_hip_pitch": 0.4, "right_hip_pitch": 0.4}
    world = forward_kinematics(model, q)
    tl, tr = world["left_foot"][1], world["right_foot"][1]
    assert np.allclose(tl * [1, -1, 1], tr, atol=1e-9)
    assert tl[1] > forward_kinematics(model, {})["left_foot"][1][1]   # +roll = outwards
    assert tl[0] > 0.03                                                  # +pitch = forwards


def test_stand_pose_within_limits():
    cfg = Biped8Config()
    for jt, v in cfg.stand_pose.items():
        lo, hi = cfg.joint_ranges[jt]
        assert lo <= v <= hi


@pytest.mark.parametrize("env_id", ["SimBiped-v0", "SimBiped8-v0"])
def test_envs_run(env_id):
    pytest.importorskip("mujoco")
    gym = pytest.importorskip("gymnasium")
    import biped_walker  # noqa: F401

    env = gym.make(env_id)
    obs, info = env.reset(seed=0)
    n = env.unwrapped.robot.n_joints
    assert env.action_space.shape == (n,)
    weight = env.unwrapped.robot.total_mass * 9.81
    assert 0.7 * weight < info["tactile"].sum() < 1.3 * weight
    for _ in range(50):
        obs, r, term, trunc, info = env.step(np.zeros(n))
        assert np.isfinite(r) and np.all(np.isfinite(obs))
    assert not term, "standing still with zero action should not fall"
    env.close()
