"""Matplotlib renders of the 10-DoF biped (the assembled robot and a parts sheet).

These are CAD renders from the same meshes and layout the simulator loads;
they don't need MuJoCo or a GPU.

    python -m biped_walker.cad.render            # writes docs/biped10*.png
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Dict, Optional

import numpy as np

from .biped10_design import MESH_DIR, MODEL_JSON, STAND_POSE, posed_parts
from .mesh import read_stl, transform

LIGHT = np.array([0.45, -0.35, 0.82])
LIGHT /= np.linalg.norm(LIGHT)


def _shade(tris: np.ndarray, rgba, ambient=0.38) -> np.ndarray:
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    lam = np.clip(n @ LIGHT, 0, 1)
    spec = np.clip(n @ LIGHT, 0, 1) ** 12 * 0.25
    base = np.asarray(rgba[:3])
    col = np.clip(base[None] * (ambient + (1 - ambient) * lam[:, None]) + spec[:, None], 0, 1)
    return np.concatenate([col, np.ones((len(tris), 1))], 1)


def _load(model):
    return {name: read_stl(MESH_DIR / p["mesh"]) for name, p in model["parts"].items()}


def _setup_axes(fig, rect, elev, azim):
    ax = fig.add_axes(rect, projection="3d", computed_zorder=False)
    ax.view_init(elev=elev, azim=azim)
    ax.set_proj_type("persp", focal_length=0.35)
    ax.set_axis_off()
    return ax


def posed_triangles(q: Optional[Dict[str, float]] = None):
    """All robot triangles (mm, standing on z = 0) and their RGB colours for pose ``q``."""
    model = json.loads(MODEL_JSON.read_text())
    meshes = _load(model)
    q = STAND_POSE if q is None else q
    tris, cols = [], []
    for part, R, t in posed_parts(model, q):
        m = transform(meshes[part], R, t)
        tris.append(m)
        cols.append(np.repeat([model["parts"][part]["rgba"][:3]], len(m), 0))
    tris = np.concatenate(tris)
    tris[..., 2] -= tris[..., 2].min()
    return tris, np.concatenate(cols)


def render_robot(path: str | Path, q: Optional[Dict[str, float]] = None, elev: float = 12, azim: float = -50,
                 distance: float = 900.0, width: int = 900, height: int = 1100, supersample: int = 2) -> Path:
    """Z-buffered, shadowed render of the robot standing on a checker floor."""
    import imageio.v2 as imageio

    from .raster import render

    tris, cols = posed_triangles(q)
    c = tris.reshape(-1, 3)
    target = np.array([(c[:, 0].min() + c[:, 0].max()) / 2, (c[:, 1].min() + c[:, 1].max()) / 2,
                       0.45 * c[:, 2].max()])
    el, az = np.radians(elev), np.radians(azim)
    eye = target + distance * np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
    img = render(tris, cols, eye, target, width, height, fov_deg=24, supersample=supersample)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    imageio.imwrite(path, img)
    return path


def render_parts_sheet(path: str | Path, dpi: int = 120) -> Path:
    """Grid of the bracket-kit parts, each rendered on its own."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    from .raster import render

    model = json.loads(MODEL_JSON.read_text())
    meshes = _load(model)
    names = ["mg996r", "bracket_multi", "u_bracket_30", "u_bracket_42", "yaw_plate", "foot_plate"]
    fig, axes = plt.subplots(2, 3, figsize=(12, 8.4), dpi=dpi)
    for ax, name in zip(axes.ravel(), names):
        m = meshes[name].copy()
        if name.startswith("u_bracket"):   # stand the U on its base
            m = transform(m, np.array([[0, 0, -1], [0, 1, 0], [1, 0, 0]], float))
        m[..., 2] -= m[..., 2].min()
        rgba = np.array(model["parts"][name]["rgba"][:3])
        rgba = np.maximum(rgba, 0.22)                    # lift dark parts so detail shows
        c = m.reshape(-1, 3)
        size = (c.max(0) - c.min(0)).max()
        target = (c.max(0) + c.min(0)) / 2
        el, az = np.radians(28), np.radians(-55)
        eye = target + 3.2 * size * np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
        img = render(m, np.repeat(rgba[None], len(m), 0), eye, target, 420, 340, fov_deg=22,
                     floor_size=2 * size, floor_square=size / 4, supersample=2, shadow_res=1024)
        ax.imshow(img)
        ax.set_axis_off()
        p = model["parts"][name]
        ax.set_title(f"{name}  ({p['mass_g']:.0f} g)\n{p['description']}", fontsize=10)
    plt.tight_layout()
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=dpi, facecolor="white")
    plt.close(fig)
    return path


if __name__ == "__main__":
    docs = Path(__file__).resolve().parents[2] / "docs"
    print(render_robot(docs / "biped10.png"))
    print(render_robot(docs / "biped10_side.png", azim=-100, elev=6, width=800))
    print(render_parts_sheet(docs / "biped10_parts.png"))
