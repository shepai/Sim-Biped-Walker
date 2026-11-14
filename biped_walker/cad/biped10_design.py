"""Layout of the 10-DoF servo-bracket biped, built from the parts in ``parts.py``.

Each leg has hip yaw, hip roll, hip pitch, knee and ankle roll. Every joint is
an MG996R "servo unit": a servo in a multi-purpose bracket with a U bracket
straddling it (top arm on the horn, bottom arm on the bracket's bearing).

* The two **hip-yaw** servos stand inside the body box on their mounting tabs,
  shafts pointing down through cut-outs in the box floor. A yaw plate on each
  horn, just under the box, carries the hip-roll servo.
* To keep the legs short and the centre of mass low, the other servos **lie on
  their side**: each bracket is bolted to its neighbour by one of its large side
  walls, so a servo unit only takes ~26 mm of leg height. The hip-roll servos
  point outwards so the space under the box between the hips stays clear for
  the yaw links; the hip-pitch servo points backwards and the knee servo
  forwards, so the thigh's forward swing and the knee bend are unobstructed.
* The shin is two U brackets bolted base to base, and the ankle-roll servo lies
  flat across the foot with its horn facing forwards.

Axis spacing (mm): hip yaw (horn) -> hip roll 15, hip roll -> hip pitch 39,
hip pitch -> knee 57, knee -> ankle 64, ankle -> sole 15.

Robot frame: x forward, y left, z up; origin midway between the hip axes.
The right leg is the mirror image of the left (all parts are symmetric in their
local y, so mirroring is a proper rotation ``M R M`` with ``M = diag(1,-1,1)``;
joint axes mirror as ``-M a`` so equal commands give mirror-symmetric motion).

Running ``python -m biped_walker.cad.biped10_design`` writes
  * ``biped_walker/assets/biped10/meshes/*.stl``  part meshes (mm)
  * ``biped_walker/assets/biped10/biped10.json``  bodies, joints, inertias
  * ``cad/biped10/assembly.scad``                 OpenSCAD assembly (two poses)
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

from . import parts as P
from .mesh import mass_properties, write_stl

PKG_DIR = Path(__file__).resolve().parents[1]
ASSET_DIR = PKG_DIR / "assets" / "biped10"
MESH_DIR = ASSET_DIR / "meshes"
MODEL_JSON = ASSET_DIR / "biped10.json"
REPO_CAD_DIR = PKG_DIR.parent / "cad" / "biped10"

HIP_Y = 42.0          # lateral offset of each leg line (and yaw axis) from the centre
FLOOR_TOP = 40.0      # box floor top: the yaw servo's tabs rest on it
YAW_Z = FLOOR_TOP - 17.0          # yaw horn outer face (tab underside is 17 mm above it)
HIP_Z = YAW_Z - 2.0 - 13.0        # hip-roll axis: under the 2 mm yaw plate + bracket side wall
U_HIP, U_THIGH, U_SHIN, U_ANKLE = 24.0, 42.0, 30.0, 30.0   # U bracket lengths
SIDE = 13.0           # bracket side-wall outer face, measured from the axis
SOLE = {"x": (-50.0, 50.0), "y": (-30.0, 30.0)}  # foot plate, relative to sole frame
FOOT_X = -5.0         # foot_plate part offset so the plate is centred under the ankle

M = np.diag([1.0, -1.0, 1.0])
EX, EY, EZ = np.eye(3)
RZ_UP = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], float)    # U along +servo y
RZ_DOWN = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]], float)  # U along -servo y
RZ_180 = np.diag([-1.0, -1.0, 1.0])

U_PARTS = {24.0: "u_bracket_24", 30.0: "u_bracket_30", 42.0: "u_bracket_42"}
SCAD_CALL = {
    "mg996r": "mg996r()", "servo_horn": "servo_horn()", "bracket_multi": "bracket_multi()",
    "u_bracket_24": "u_bracket(24)", "u_bracket_30": "u_bracket(30)", "u_bracket_42": "u_bracket(42)",
    "yaw_plate": "yaw_plate()", "foot_plate": "foot_plate()", "chassis_frame": "chassis_frame()",
    "chassis_plates": "chassis_plates()", "battery": "battery()", "pcb": "pcb()",
}


def _frame(xs, ys, zs) -> np.ndarray:
    """Servo frame from its x (long body), y (side-wall normal) and z (shaft) in robot axes."""
    return np.column_stack([xs, ys, zs]).astype(float)


@dataclass
class Body:
    name: str
    parent: Optional[str]
    R: np.ndarray                     # world rotation at the zero pose
    t: np.ndarray                     # world position (mm) at the zero pose
    joint: Optional[str] = None
    axis_world: Optional[np.ndarray] = None
    parts: List[Tuple[str, np.ndarray, np.ndarray, bool]] = field(default_factory=list)  # (part, R, t, collide)


def _leg_frames(Y: float) -> Dict[str, Tuple[np.ndarray, np.ndarray]]:
    z_pitch = HIP_Z - (U_HIP + 2 + SIDE)
    z_knee = z_pitch - (U_THIGH + 2 + SIDE)
    z_ankle = z_knee - (U_SHIN + 2) - (U_ANKLE + 2)
    return {
        # standing in the body box on its tabs: shaft down, body pointing inwards
        "hip_yaw": (_frame(-EY, -EX, -EZ), np.array([0.0, Y, YAW_Z])),
        # shaft forwards, body pointing outwards (keeps the space between the hips
        # clear so the yaw links can't meet), side wall up against the yaw plate
        "hip_roll": (_frame(EY, EZ, EX), np.array([24.0, Y, HIP_Z])),
        # shaft outwards, body pointing backwards
        "hip_pitch": (_frame(-EX, EZ, EY), np.array([0.0, Y + 24, z_pitch])),
        # shaft inwards, body pointing forwards
        "knee": (_frame(EX, EZ, -EY), np.array([0.0, Y - 24, z_knee])),
        # lying across the foot: shaft (horn) forwards, body pointing outwards,
        # shifted inwards so the bracket sits centred on the foot
        "ankle_roll": (_frame(EY, EZ, EX), np.array([24.0, Y - 11, z_ankle])),
    }


def _leg(side: str) -> List[Body]:
    """Bodies of one leg: built as the left leg, then mirrored as a whole for the right."""
    Y = HIP_Y
    F = _leg_frames(Y)
    z_sole = F["ankle_roll"][1][2] - SIDE - 2
    yaw_axis = np.array([0.0, 0.0, 1.0])     # +yaw = left toe turns outwards
    roll_axis = np.array([1.0, 0.0, 0.0])    # +roll = left leg outwards
    pitch_axis = np.array([0.0, -1.0, 0.0])  # +pitch = forwards (thigh)

    Rw, tw = F["hip_yaw"]
    yaw = Body("hip_yaw_link", "torso", Rw, tw, "hip_yaw", yaw_axis)
    # the plate reaches outwards, under the outward-pointing hip-roll servo
    yaw.parts += [("servo_horn", Rw, tw, True), ("yaw_plate", Rw @ RZ_180, tw, True)]
    R, t = F["hip_roll"]
    yaw.parts += [("mg996r", R, t, True), ("bracket_multi", R, t, True)]

    hip = Body("hip", "hip_yaw_link", R, t, "hip_roll", roll_axis)
    hip.parts += [("servo_horn", R, t, True), (U_PARTS[U_HIP], R @ RZ_DOWN, t, True)]
    Rn, tn = F["hip_pitch"]
    hip.parts += [("mg996r", Rn, tn, True), ("bracket_multi", Rn, tn, True)]

    thigh = Body("thigh", "hip", Rn, tn, "hip_pitch", pitch_axis)
    thigh.parts += [("servo_horn", Rn, tn, True), (U_PARTS[U_THIGH], Rn @ RZ_DOWN, tn, True)]
    Rk, tk = F["knee"]
    thigh.parts += [("mg996r", Rk, tk, True), ("bracket_multi", Rk, tk, True)]

    shin = Body("shin", "thigh", Rk, tk, "knee", pitch_axis)
    Ra, ta = F["ankle_roll"]
    shin.parts += [("servo_horn", Rk, tk, True), (U_PARTS[U_SHIN], Rk @ RZ_DOWN, tk, True),
                   (U_PARTS[U_ANKLE], Ra @ RZ_UP, ta, False), ("servo_horn", Ra, ta, False)]

    foot = Body("foot_link", "shin", Ra, ta, "ankle_roll", roll_axis)
    foot.parts += [("mg996r", Ra, ta, False), ("bracket_multi", Ra, ta, False),
                   ("foot_plate", np.eye(3), np.array([FOOT_X, Y, z_sole]), False)]
    # Sole frame: robot-aligned, at the centre of the plate underside (pads go below it).
    sole = Body("foot", "foot_link", np.eye(3), np.array([0.0, Y, z_sole]))
    leg = [yaw, hip, thigh, shin, foot, sole]

    for b in leg:
        b.name = f"{side}_{b.name}"
        if b.parent != "torso":
            b.parent = f"{side}_{b.parent}"
        if b.joint:
            b.joint = f"{side}_{b.joint}"
        if side == "right":   # mirror every placement through the robot's x-z plane
            b.R, b.t = M @ b.R @ M, M @ b.t
            b.parts = [(p, M @ R_ @ M, M @ t_, c) for p, R_, t_, c in b.parts]
            if b.axis_world is not None:
                b.axis_world = -M @ b.axis_world   # axial vector: mirror-symmetric motion
    return leg


def build_layout() -> List[Body]:
    torso = Body("torso", None, np.eye(3), np.zeros(3))
    for name in ("chassis_frame", "chassis_plates", "battery", "pcb"):
        torso.parts.append((name, np.eye(3), np.zeros(3), True))
    bodies = [torso]
    for side in ("left", "right"):
        R, t = _leg_frames(HIP_Y)["hip_yaw"]
        if side == "right":
            R, t = M @ R @ M, M @ t
        torso.parts.append(("mg996r", R, t, True))   # yaw servo, held by its tabs on the box floor
        bodies += _leg(side)
    return bodies


# ------------------------------------------------------------- geometry ops
def mat_to_quat(R: np.ndarray) -> List[float]:
    """Rotation matrix -> unit quaternion (w, x, y, z), robust for 180-degree turns (Shepperd)."""
    R = np.asarray(R, float)
    tr = np.trace(R)
    if tr > 0:
        S = math.sqrt(tr + 1.0) * 2
        q = [0.25 * S, (R[2, 1] - R[1, 2]) / S, (R[0, 2] - R[2, 0]) / S, (R[1, 0] - R[0, 1]) / S]
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        S = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        q = [(R[2, 1] - R[1, 2]) / S, 0.25 * S, (R[0, 1] + R[1, 0]) / S, (R[0, 2] + R[2, 0]) / S]
    elif R[1, 1] > R[2, 2]:
        S = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        q = [(R[0, 2] - R[2, 0]) / S, (R[0, 1] + R[1, 0]) / S, 0.25 * S, (R[1, 2] + R[2, 1]) / S]
    else:
        S = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        q = [(R[1, 0] - R[0, 1]) / S, (R[0, 2] + R[2, 0]) / S, (R[1, 2] + R[2, 1]) / S, 0.25 * S]
    q = np.array(q)
    q /= np.linalg.norm(q)
    if q[0] < 0:
        q = -q
    return [0.0 if abs(v) < 1e-12 else float(v) for v in q]


def quat_to_mat(q) -> np.ndarray:
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def axis_angle(axis, angle) -> np.ndarray:
    k = np.asarray(axis, float)
    k = k / np.linalg.norm(k)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * K @ K


# -------------------------------------------------------------- generation
def part_meshes() -> Dict[str, np.ndarray]:
    return {name: d.build() for name, d in P.PARTS.items()}


def part_mass_properties(meshes) -> Dict[str, dict]:
    out = {}
    for name, d in P.PARTS.items():
        mp = mass_properties(meshes[name], 1.0)
        density = d.density if d.mass is None else d.mass / mp["volume"]
        out[name] = {"mass": mp["volume"] * density, "com": mp["com"], "inertia": mp["inertia"] * density}
    return out


def _combine(props):
    """Combine (mass, com, inertia about com) in a common frame."""
    m = sum(p[0] for p in props)
    com = sum(p[0] * p[1] for p in props) / m
    I = np.zeros((3, 3))
    for mi, ci, Ii in props:
        d = ci - com
        I += Ii + mi * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    return m, com, I


def build_model_data(bodies: List[Body], meshes, mprops) -> dict:
    by_name = {b.name: b for b in bodies}
    out_bodies = []
    for b in bodies:
        if b.parent is None:
            R_rel, t_rel = b.R, b.t
        else:
            p = by_name[b.parent]
            R_rel, t_rel = p.R.T @ b.R, p.R.T @ (b.t - p.t)
        visuals, inert, pts = [], [], []
        for part, R, t, collide in b.parts:
            Rl, tl = b.R.T @ R, b.R.T @ (t - b.t)          # part pose in body frame
            visuals.append({"part": part, "pos": (tl / 1000).tolist(), "quat": mat_to_quat(Rl)})
            mp = mprops[part]
            inert.append((mp["mass"] / 1000.0, (Rl @ mp["com"] + tl) / 1000.0,
                          Rl @ mp["inertia"] @ Rl.T * 1e-9))  # g -> kg, g mm^2 -> kg m^2
            if collide:
                pts.append(meshes[part].reshape(-1, 3) @ Rl.T + tl)
        axis = (b.R.T @ b.axis_world).round(12).tolist() if b.joint else [0, 0, 1]
        entry = {"name": b.name, "parent": b.parent, "pos": (t_rel / 1000).tolist(),
                 "quat": mat_to_quat(R_rel), "joint": b.joint, "axis": axis, "visuals": visuals}
        if inert:
            m, com, I = _combine(inert)
            entry["inertial"] = {"mass": m, "pos": com.tolist(),
                                 "fullinertia": [I[0, 0], I[1, 1], I[2, 2], I[0, 1], I[0, 2], I[1, 2]]}
        if pts:
            P_ = np.concatenate(pts)
            lo, hi = P_.min(0), P_.max(0)
            entry["collision_box"] = {"pos": ((lo + hi) / 2000).tolist(), "size": ((hi - lo) / 2000).tolist()}
        out_bodies.append(entry)
    return {
        "name": "biped10",
        "description": "10-DoF MG996R servo-bracket biped (generated by biped_walker.cad.biped10_design)",
        "spacing_mm": {"yaw_to_roll": YAW_Z - HIP_Z, "hip_roll_to_pitch": U_HIP + 2 + SIDE,
                       "thigh": U_THIGH + 2 + SIDE, "shin": U_SHIN + U_ANKLE + 4, "ankle_to_sole": SIDE + 2},
        "parts": {n: {"mesh": f"{n}.stl", "rgba": list(d.rgba), "mass_g": round(mprops[n]["mass"], 2),
                      "description": d.description} for n, d in P.PARTS.items()},
        "bodies": out_bodies,
        "joints": [b.joint for b in bodies if b.joint],
        "feet": ["left_foot", "right_foot"],
        "sole": {"x": [v / 1000 for v in SOLE["x"]], "y": [v / 1000 for v in SOLE["y"]]},
        "total_mass": sum(e["inertial"]["mass"] for e in out_bodies if "inertial" in e),
    }


# ----------------------------------------------------------- posing / FK
def forward_kinematics(model: dict, q: Dict[str, float]) -> Dict[str, Tuple[np.ndarray, np.ndarray]]:
    """World (R, t[m]) of each body for joint angles ``q`` (torso at the origin)."""
    world = {}
    for b in model["bodies"]:
        R = quat_to_mat(b["quat"])
        t = np.asarray(b["pos"])
        if b["parent"] is not None:
            Rp, tp = world[b["parent"]]
            R, t = Rp @ R, Rp @ t + tp
        if b["joint"]:
            R = R @ axis_angle(b["axis"], q.get(b["joint"], 0.0))
        world[b["name"]] = (R, t)
    return world


def posed_parts(model: dict, q: Dict[str, float]):
    """List of (part, R, t[mm]) in the world for a pose."""
    world = forward_kinematics(model, q)
    out = []
    for b in model["bodies"]:
        Rb, tb = world[b["name"]]
        for v in b["visuals"]:
            out.append((v["part"], Rb @ quat_to_mat(v["quat"]), (Rb @ np.asarray(v["pos"]) + tb) * 1000))
    return out


def write_assembly_scad(model: dict, poses: Dict[str, Dict[str, float]], path: Path) -> Path:
    lines = ["// GENERATED by biped_walker/cad/biped10_design.py; do not edit by hand.",
             "// Assembly of the 10-DoF biped from biped10_parts.scad (units: mm).",
             "use <biped10_parts.scad>", "",
             f"pose = \"{next(iter(poses))}\";  // one of: {', '.join(poses)}", ""]
    for pname, q in poses.items():
        lines.append(f"if (pose == \"{pname}\") {{")
        for part, R, t in posed_parts(model, q):
            m = [[*R[i], t[i]] for i in range(3)] + [[0, 0, 0, 1]]
            clean = lambda x: 0.0 if abs(x) < 1e-9 else x  # noqa: E731
            mat = "[" + ", ".join("[" + ", ".join(f"{clean(x):.5g}" for x in row) + "]" for row in m) + "]"
            rgba = model["parts"][part]["rgba"]
            lines.append(f"  multmatrix({mat}) color([{', '.join(f'{c:.3g}' for c in rgba)}]) "
                         f"{SCAD_CALL[part]};")
        lines.append("}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n")
    return path


STAND_POSE = {f"{s}_{j}": v for s in ("left", "right")
              for j, v in (("hip_pitch", 0.25), ("knee", -0.25))}


def generate(write_scad: bool = True, verbose: bool = True) -> dict:
    meshes = part_meshes()
    mprops = part_mass_properties(meshes)
    MESH_DIR.mkdir(parents=True, exist_ok=True)
    for name, mesh in meshes.items():
        vol = mass_properties(mesh)["volume"]
        assert vol > 0, f"{name}: mesh is inside out"
        write_stl(mesh, MESH_DIR / f"{name}.stl", name)
        if verbose:
            print(f"{name:16s} {len(mesh):6d} triangles  {mprops[name]['mass']:7.1f} g")
    model = build_model_data(build_layout(), meshes, mprops)
    MODEL_JSON.write_text(json.dumps(model, indent=1))
    if verbose:
        print(f"total mass {model['total_mass']:.3f} kg -> {MODEL_JSON}")
    if write_scad and REPO_CAD_DIR.parent.exists():
        write_assembly_scad(model, {"stand": STAND_POSE, "zero": {}}, REPO_CAD_DIR / "assembly.scad")
    return model


if __name__ == "__main__":
    generate()
