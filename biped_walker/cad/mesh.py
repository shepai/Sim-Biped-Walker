"""Tiny triangle-mesh toolkit (NumPy only) used to mirror the OpenSCAD part designs.

Meshes are ``(N, 3, 3)`` float arrays of triangles with outward (counter-
clockwise) winding, in millimetres like OpenSCAD. Sheet-metal plates with
holes are made by extruding a 2D mask sampled on a fine grid. Bolt holes come
out slightly faceted, which is fine for visuals and mass properties.
"""

from __future__ import annotations

import struct
from pathlib import Path
from typing import Callable, Iterable, Optional, Sequence, Tuple

import numpy as np

Mesh = np.ndarray
Shape2D = Callable[[np.ndarray, np.ndarray], np.ndarray]


# ------------------------------------------------------------------ helpers
def empty() -> Mesh:
    return np.zeros((0, 3, 3))


def merge(*meshes: Mesh) -> Mesh:
    meshes = [m for m in meshes if len(m)]
    return np.concatenate(meshes) if meshes else empty()


def _quad(a, b, c, d, normal) -> Mesh:
    a, b, c, d = map(np.asarray, (a, b, c, d))
    if np.dot(np.cross(b - a, c - a), normal) < 0:
        a, b, c, d = d, c, b, a
    return np.array([[a, b, c], [a, c, d]], dtype=float)


def transform(mesh: Mesh, R=None, t=None) -> Mesh:
    """Apply ``x -> R x + t``; keeps outward winding if ``det(R) < 0`` (mirrors)."""
    R = np.eye(3) if R is None else np.asarray(R, float)
    t = np.zeros(3) if t is None else np.asarray(t, float)
    out = mesh @ R.T + t
    if np.linalg.det(R) < 0:
        out = out[:, ::-1, :]
    return out


def translate(mesh: Mesh, t) -> Mesh:
    return transform(mesh, None, t)


# --------------------------------------------------------------- primitives
def box(x0, x1, y0, y1, z0, z1) -> Mesh:
    """Axis-aligned box from corner ranges (like a translated OpenSCAD cube)."""
    p = lambda x, y, z: (x, y, z)  # noqa: E731
    faces = [
        _quad(p(x0, y0, z0), p(x0, y1, z0), p(x0, y1, z1), p(x0, y0, z1), (-1, 0, 0)),
        _quad(p(x1, y0, z0), p(x1, y1, z0), p(x1, y1, z1), p(x1, y0, z1), (1, 0, 0)),
        _quad(p(x0, y0, z0), p(x1, y0, z0), p(x1, y0, z1), p(x0, y0, z1), (0, -1, 0)),
        _quad(p(x0, y1, z0), p(x1, y1, z0), p(x1, y1, z1), p(x0, y1, z1), (0, 1, 0)),
        _quad(p(x0, y0, z0), p(x1, y0, z0), p(x1, y1, z0), p(x0, y1, z0), (0, 0, -1)),
        _quad(p(x0, y0, z1), p(x1, y0, z1), p(x1, y1, z1), p(x0, y1, z1), (0, 0, 1)),
    ]
    return merge(*faces)


def cylinder(r, z0, z1, cx=0.0, cy=0.0, n=32) -> Mesh:
    """Cylinder along z (like OpenSCAD ``translate([cx,cy,z0]) cylinder(r=r, h=z1-z0)``)."""
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)
    pts = np.stack([cx + r * np.cos(a), cy + r * np.sin(a)], 1)
    tris = []
    for i in range(n):
        p, q = pts[i], pts[(i + 1) % n]
        mid = (p + q) / 2 - (cx, cy)
        tris.append(_quad((*p, z0), (*q, z0), (*q, z1), (*p, z1), (mid[0], mid[1], 0)))
        tris.append(np.array([[(cx, cy, z1), (*p, z1), (*q, z1)]]))
        tris.append(np.array([[(cx, cy, z0), (*q, z0), (*p, z0)]]))
    return merge(*tris)


# ----------------------------------------------------------------- 2D shapes
def rect(u0, u1, v0, v1) -> Shape2D:
    return lambda u, v: (u >= u0) & (u <= u1) & (v >= v0) & (v <= v1)


def circle(cu, cv, r) -> Shape2D:
    return lambda u, v: (u - cu) ** 2 + (v - cv) ** 2 <= r * r


def rounded_rect(u0, u1, v0, v1, r) -> Shape2D:
    def f(u, v):
        cu = np.clip(u, u0 + r, u1 - r)
        cv = np.clip(v, v0 + r, v1 - r)
        return rect(u0, u1, v0, v1)(u, v) & ((u - cu) ** 2 + (v - cv) ** 2 <= r * r)
    return f


def union(*shapes: Shape2D) -> Shape2D:
    return lambda u, v: np.logical_or.reduce([s(u, v) for s in shapes])


def difference(shape: Shape2D, *holes: Shape2D) -> Shape2D:
    def f(u, v):
        m = shape(u, v)
        for h in holes:
            m &= ~h(u, v)
        return m
    return f


# --------------------------------------------------------------- extrusion
def extrude(shape: Shape2D, bounds: Tuple[float, float, float, float], t: float,
            res: float = 0.5) -> Mesh:
    """Extrude a 2D shape over ``w in [0, t]`` in a local (u, v, w) frame.

    The shape is sampled on a ``res`` mm grid inside ``bounds`` = (u0, u1, v0, v1).
    Equivalent to OpenSCAD ``linear_extrude(t) <2D shape>``.
    """
    u0, u1, v0, v1 = bounds
    nu = max(1, int(round((u1 - u0) / res)))
    nv = max(1, int(round((v1 - v0) / res)))
    du, dv = (u1 - u0) / nu, (v1 - v0) / nv
    uc = u0 + (np.arange(nu) + 0.5) * du
    vc = v0 + (np.arange(nv) + 0.5) * dv
    U, V = np.meshgrid(uc, vc)           # rows = v, cols = u
    M = shape(U, V)
    tris = []
    U_ = lambda j: u0 + j * du  # noqa: E731
    V_ = lambda i: v0 + i * dv  # noqa: E731

    def runs(row):
        r = np.concatenate([[False], row, [False]]).astype(int)
        d = np.diff(r)
        return zip(np.where(d == 1)[0], np.where(d == -1)[0])

    for i in range(nv):
        for j0, j1 in runs(M[i]):
            a, b = U_(j0), U_(j1)
            c, d = V_(i), V_(i + 1)
            tris.append(_quad((a, c, t), (b, c, t), (b, d, t), (a, d, t), (0, 0, 1)))
            tris.append(_quad((a, c, 0), (b, c, 0), (b, d, 0), (a, d, 0), (0, 0, -1)))
            tris.append(_quad((a, c, 0), (a, d, 0), (a, d, t), (a, c, t), (-1, 0, 0)))
            tris.append(_quad((b, c, 0), (b, d, 0), (b, d, t), (b, c, t), (1, 0, 0)))
    zero = np.zeros(nu, bool)
    for i in range(nv + 1):
        below = M[i - 1] if i > 0 else zero
        above = M[i] if i < nv else zero
        v = V_(i)
        for j0, j1 in runs(above & ~below):   # wall facing -v
            tris.append(_quad((U_(j0), v, 0), (U_(j1), v, 0), (U_(j1), v, t), (U_(j0), v, t), (0, -1, 0)))
        for j0, j1 in runs(below & ~above):   # wall facing +v
            tris.append(_quad((U_(j0), v, 0), (U_(j1), v, 0), (U_(j1), v, t), (U_(j0), v, t), (0, 1, 0)))
    return merge(*tris)


# Frames for plates, matching the OpenSCAD multmatrix() calls in the .scad files.
def plate_xy(shape, bounds, z0, t, res=0.5) -> Mesh:
    """(u, v, w) -> (x, y, z0 + w)."""
    return translate(extrude(shape, bounds, t, res), (0, 0, z0))


def plate_xz(shape, bounds, y0, t, res=0.5) -> Mesh:
    """(u, v, w) -> (x = u, y = y0 + w, z = v)."""
    R = np.array([[1, 0, 0], [0, 0, 1], [0, 1, 0]], float)
    return transform(extrude(shape, bounds, t, res), R, (0, y0, 0))


def plate_yz(shape, bounds, x0, t, res=0.5) -> Mesh:
    """(u, v, w) -> (x = x0 + w, y = u, z = v)."""
    R = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]], float)
    return transform(extrude(shape, bounds, t, res), R, (x0, 0, 0))


# ------------------------------------------------------------ mass, files
def mass_properties(mesh: Mesh, density: float = 1.0):
    """Volume, mass, centre of mass and inertia tensor about the COM.

    Uses the divergence theorem over origin-based tetrahedra, so the mesh must
    be closed with outward winding (overlapping closed shells are counted twice).
    """
    a, b, c = mesh[:, 0], mesh[:, 1], mesh[:, 2]
    v = np.einsum("ij,ij->i", a, np.cross(b, c)) / 6.0
    vol = v.sum()
    com = ((a + b + c) / 4.0 * v[:, None]).sum(0) / vol
    s = a + b + c
    C = (v[:, None, None] / 20.0) * (np.einsum("ni,nj->nij", a, a) + np.einsum("ni,nj->nij", b, b)
                                     + np.einsum("ni,nj->nij", c, c) + np.einsum("ni,nj->nij", s, s))
    C = C.sum(0)
    I_o = np.trace(C) * np.eye(3) - C
    I_com = I_o - vol * (np.dot(com, com) * np.eye(3) - np.outer(com, com))
    m = vol * density
    return {"volume": vol, "mass": m, "com": com, "inertia": I_com * density}


def write_stl(mesh: Mesh, path: str | Path, name: str = "part") -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    n = np.cross(mesh[:, 1] - mesh[:, 0], mesh[:, 2] - mesh[:, 0])
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    rec = np.zeros(len(mesh), dtype=[("n", "<f4", 3), ("v", "<f4", 9), ("a", "<u2")])
    rec["n"] = n
    rec["v"] = mesh.reshape(-1, 9)
    with open(path, "wb") as f:
        f.write(name.encode()[:80].ljust(80, b" "))
        f.write(struct.pack("<I", len(mesh)))
        f.write(rec.tobytes())
    return path


def read_stl(path: str | Path) -> Mesh:
    b = Path(path).read_bytes()
    n = struct.unpack("<I", b[80:84])[0]
    rec = np.frombuffer(b[84:84 + 50 * n], dtype=[("n", "<f4", 3), ("v", "<f4", 9), ("a", "<u2")])
    return rec["v"].reshape(-1, 3, 3).astype(float)
