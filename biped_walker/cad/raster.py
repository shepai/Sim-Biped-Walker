"""Minimal NumPy software rasterizer: z-buffer, shadow map, checker floor, supersampling.

Good enough for a clean "product photo" of the CAD meshes without OpenGL.
"""

from __future__ import annotations

from typing import Optional, Tuple

import numpy as np


def look_at(eye, target, up=(0, 0, 1)):
    eye, target, up = (np.asarray(v, float) for v in (eye, target, up))
    f = target - eye
    f /= np.linalg.norm(f)
    r = np.cross(f, up)
    r /= np.linalg.norm(r)
    u = np.cross(r, f)
    return np.stack([r, u, f]), eye   # rows: camera right, up, forward


def _rasterize(sx, sy, sz, W, H, depth, on_pixels):
    """Rasterize triangles given screen coords (N,3) and depth (N,3).

    Keeps the nearest fragment in ``depth``; for every triangle calls
    ``on_pixels(k, iy, ix, b0, b1, b2)`` with the winning pixels.
    """
    for k in range(len(sx)):
        x, y, z = sx[k], sy[k], sz[k]
        x0, x1 = max(int(np.floor(x.min())), 0), min(int(np.ceil(x.max())), W - 1)
        y0, y1 = max(int(np.floor(y.min())), 0), min(int(np.ceil(y.max())), H - 1)
        if x0 > x1 or y0 > y1:
            continue
        area = (x[1] - x[0]) * (y[2] - y[0]) - (x[2] - x[0]) * (y[1] - y[0])
        if abs(area) < 1e-12:
            continue
        px, py = np.meshgrid(np.arange(x0, x1 + 1) + 0.5, np.arange(y0, y1 + 1) + 0.5)
        b1 = ((px - x[0]) * (y[2] - y[0]) - (x[2] - x[0]) * (py - y[0])) / area
        b2 = ((x[1] - x[0]) * (py - y[0]) - (px - x[0]) * (y[1] - y[0])) / area
        b0 = 1 - b1 - b2
        inside = (b0 >= -1e-6) & (b1 >= -1e-6) & (b2 >= -1e-6)
        if not inside.any():
            continue
        zz = b0 * z[0] + b1 * z[1] + b2 * z[2]
        iy, ix = np.nonzero(inside)
        iy, ix = iy + y0, ix + x0
        zz = zz[inside]
        win = zz < depth[iy, ix]
        if not win.any():
            continue
        iy, ix, zz = iy[win], ix[win], zz[win]
        depth[iy, ix] = zz
        if on_pixels is not None:
            sel = np.nonzero(inside)
            on_pixels(k, iy, ix, b0[sel][win], b1[sel][win], b2[sel][win])


def render(tris: np.ndarray, colours: np.ndarray, eye, target, width=900, height=1100, fov_deg=28.0,
           light_dir=(0.45, -0.35, 0.82), floor_z: float = 0.0, floor_size: float = 500.0,
           floor_square: float = 40.0, supersample: int = 2, shadow_res: int = 2048,
           background=((0.97, 0.97, 0.98), (0.84, 0.86, 0.9))) -> np.ndarray:
    """Render triangles (N,3,3) with per-triangle RGB ``colours`` (N,3). Returns uint8 HxWx3."""
    W, H = width * supersample, height * supersample
    L = np.asarray(light_dir, float)
    L /= np.linalg.norm(L)

    # add the floor as two big triangles, flagged by index
    s = floor_size
    fl = np.array([[(-s, -s, floor_z), (s, -s, floor_z), (s, s, floor_z)],
                   [(-s, -s, floor_z), (s, s, floor_z), (-s, s, floor_z)]], float)
    n_robot = len(tris)
    all_tris = np.concatenate([tris, fl])
    normals = np.cross(all_tris[:, 1] - all_tris[:, 0], all_tris[:, 2] - all_tris[:, 0])
    normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-12)

    # ---------------------------------------------------------- camera pass
    Rc, eye = look_at(eye, target)
    cam = (all_tris - eye) @ Rc.T                    # (N,3,3): x right, y up, z forward
    keep = (cam[..., 2] > 1e-3).all(1)
    view = all_tris.mean(1) - eye
    keep &= (np.einsum("ij,ij->i", normals, view) < 0) | (np.arange(len(all_tris)) >= n_robot)
    idx = np.nonzero(keep)[0]
    f = 0.5 * H / np.tan(np.radians(fov_deg) / 2)
    sx = W / 2 + f * cam[idx, :, 0] / cam[idx, :, 2]
    sy = H / 2 - f * cam[idx, :, 1] / cam[idx, :, 2]
    sz = cam[idx, :, 2]
    depth = np.full((H, W), np.inf)
    tri_id = np.full((H, W), -1, int)
    bary = np.zeros((H, W, 3))

    def on_cam(k, iy, ix, b0, b1, b2):
        tri_id[iy, ix] = idx[k]
        bary[iy, ix] = np.stack([b0, b1, b2], 1)

    _rasterize(sx, sy, sz, W, H, depth, on_cam)

    # world position of each pixel (perspective-correct barycentrics)
    hit = tri_id >= 0
    ids = tri_id[hit]
    zc = cam[ids, :, 2]
    w = bary[hit] / zc
    w /= w.sum(1, keepdims=True)
    P = np.einsum("nk,nkd->nd", w, all_tris[ids])

    # ---------------------------------------------------------- shadow pass
    Rl, _ = look_at(L * 1000, np.zeros(3), up=(0, 1, 0) if abs(L[2]) > 0.99 else (0, 0, 1))
    lp = tris @ Rl.T
    lo, hi = lp.reshape(-1, 3).min(0), lp.reshape(-1, 3).max(0)
    pad = 0.05 * (hi - lo).max()
    lo, hi = lo - pad, hi + pad
    scale = (shadow_res - 1) / (hi[:2] - lo[:2]).max()
    lsx = (lp[..., 0] - lo[0]) * scale
    lsy = (lp[..., 1] - lo[1]) * scale
    ldepth = np.full((shadow_res, shadow_res), np.inf)
    _rasterize(lsx, lsy, lp[..., 2], shadow_res, shadow_res, ldepth, None)

    Pl = P @ Rl.T
    u = (Pl[:, 0] - lo[0]) * scale
    v = (Pl[:, 1] - lo[1]) * scale
    bias = 1.5
    lit = np.zeros(len(P))
    taps = [(dx, dy) for dx in (-1.5, 0, 1.5) for dy in (-1.5, 0, 1.5)]
    for dx, dy in taps:   # percentage-closer filtering
        ui = np.clip((u + dx).astype(int), 0, shadow_res - 1)
        vi = np.clip((v + dy).astype(int), 0, shadow_res - 1)
        outside = (u + dx < 0) | (u + dx >= shadow_res) | (v + dy < 0) | (v + dy >= shadow_res)
        lit += (outside | (Pl[:, 2] <= ldepth[vi, ui] + bias)).astype(float)
    lit /= len(taps)

    # -------------------------------------------------------------- shading
    n = normals[ids]
    is_floor = ids >= n_robot
    base = np.zeros((len(ids), 3))
    base[~is_floor] = colours[ids[~is_floor]]
    cx = np.floor(P[is_floor, 0] / floor_square).astype(int)
    cy = np.floor(P[is_floor, 1] / floor_square).astype(int)
    chk = ((cx + cy) % 2 == 0)[:, None]
    base[is_floor] = np.where(chk, (0.80, 0.82, 0.85), (0.89, 0.90, 0.92))
    lam = np.clip(n @ L, 0, 1)
    vdir = eye - P
    vdir /= np.linalg.norm(vdir, axis=1, keepdims=True)
    hvec = vdir + L
    hvec /= np.linalg.norm(hvec, axis=1, keepdims=True)
    spec = np.clip(np.einsum("ij,ij->i", n, hvec), 0, 1) ** 40 * np.where(is_floor, 0.05, 0.35)
    sky = 0.5 + 0.5 * n[:, 2]                         # hemispherical ambient
    amb = 0.22 + 0.16 * sky
    shade = amb[:, None] * base + (0.75 * lam * lit)[:, None] * base + (spec * lit)[:, None]
    # fade the floor towards the background with distance
    dist = np.linalg.norm(P[:, :2] - np.asarray(target)[:2], axis=1)
    fade = np.clip((dist - 0.45 * floor_size) / (0.5 * floor_size), 0, 1)[:, None]

    top, bottom = (np.asarray(c, float) for c in background)
    t = np.linspace(0, 1, H)[:, None, None]
    img = np.broadcast_to(top * (1 - t) + bottom * t, (H, W, 3)).copy()
    bg = img[hit]
    shade = np.where(is_floor[:, None], shade * (1 - fade) + bg * fade, shade)
    img[hit] = np.clip(shade, 0, 1)

    if supersample > 1:
        img = img.reshape(height, supersample, width, supersample, 3).mean((1, 3))
    return (np.clip(img, 0, 1) ** (1 / 1.1) * 255).astype(np.uint8)
