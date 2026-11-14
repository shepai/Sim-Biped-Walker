"""Tests for the MJCF builder (no MuJoCo needed) and, if available, the env."""

import xml.etree.ElementTree as ET

import numpy as np
import pytest

from biped_walker.model import JOINT_NAMES, ModelConfig, TactileConfig, build_mjcf, taxel_layout


def test_mjcf_is_valid_xml_with_taxels():
    root = ET.fromstring(build_mjcf())
    touches = root.findall("./sensor/touch")
    assert len(touches) == 2 * 15
    assert [a.get("name") for a in root.findall("./actuator/position")] == list(JOINT_NAMES)
    sites = {s.get("name") for s in root.iter("site")}
    assert all(t.get("site") in sites for t in touches)


def test_custom_tactile_grid():
    cfg = ModelConfig(tactile=TactileConfig(cols=4, rows=8))
    assert len(ET.fromstring(build_mjcf(cfg)).findall("./sensor/touch")) == 64


def test_taxel_sites_tile_sole_without_overlap():
    for foot in ("left_foot", "right_foot"):
        lay = taxel_layout(foot, TactileConfig())
        xs = sorted({round(t["site_pos"][0], 6) for t in lay})
        ys = sorted({round(t["site_pos"][1], 6) for t in lay})
        assert len(xs) == 3 and len(ys) == 5
        hx, hy = lay[0]["site_size"][:2]
        assert np.allclose(np.diff(xs), 2 * hx, atol=1e-5) and np.allclose(np.diff(ys), 2 * hy, atol=1e-5)
        # pads are inside their sites
        for t in lay:
            assert t["pad_size"][0] < t["site_size"][0] and t["pad_size"][1] < t["site_size"][1]


def test_env_runs_and_feet_feel_the_floor():
    pytest.importorskip("mujoco")
    gym = pytest.importorskip("gymnasium")
    import biped_walker  # noqa: F401

    env = gym.make("SimBiped-v0")
    obs, info = env.reset(seed=0)
    assert obs.shape == env.observation_space.shape
    tac = info["tactile"]
    assert tac.shape == (2, 3, 5)
    weight = env.unwrapped.robot.total_mass * 9.81
    # standing still, the taxels should carry roughly the robot's weight
    assert 0.7 * weight < tac.sum() < 1.3 * weight, (tac.sum(), weight)
    for _ in range(50):
        obs, r, term, trunc, info = env.step(env.action_space.sample() * 0.2)
        assert np.isfinite(r) and np.all(np.isfinite(obs))
    env.close()
