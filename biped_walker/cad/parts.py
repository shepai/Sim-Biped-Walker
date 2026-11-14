"""Python mirror of ``cad/biped10/biped10_parts.scad``.

Each function builds the same solid as the OpenSCAD module of the same name
(units: mm, same frames), so the simulation meshes can be generated without
OpenSCAD installed. If you change a dimension here, change it in the .scad file
too (and vice versa).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Dict, Optional

import numpy as np

from .mesh import (box, circle, cylinder, difference, merge, plate_xy, plate_xz, plate_yz, rect,
                   rounded_rect, union)

T = 2.0
ALUMINIUM = 2.70e-3   # g / mm^3
RES = 0.5             # mm, sampling of plate outlines


def holes(pts, r):
    return union(*[circle(u, v, r) for u, v in pts])


def grid(u0, u1, v0, v1, pitch):
    us = np.arange(u0, u1 + 1e-6, pitch)
    vs = np.arange(v0, v1 + 1e-6, pitch)
    return [(u, v) for u in us for v in vs]


def _ring(r, n=4):
    return [(r * np.cos(a), r * np.sin(a)) for a in np.radians(np.arange(0, 360, 360 / n))]


# ------------------------------------------------------------- MG996R servo
def mg996r():
    return merge(
        box(-10.2, 30.5, -9.85, 9.85, -43.3, -6.5),
        plate_xy(difference(rect(-17.2, 37.5, -9.85, 9.85),
                            holes([(-13.9, 5), (-13.9, -5), (34.2, 5), (34.2, -5)], 2.2)),
                 (-17.2, 37.5, -9.85, 9.85), -19.5, 2.5, RES),
        cylinder(9, -6.5, -4.5),
        cylinder(4, -6.5, -5.5, cx=12),
        cylinder(2.9, -4.5, -2.5, n=24),
        box(30.5, 32, -3.5, 3.5, -40, -37),
    )


def servo_horn():
    shape = difference(circle(0, 0, 11.5), circle(0, 0, 1.6), holes(_ring(8), 1.4))
    return plate_xy(shape, (-11.5, 11.5, -11.5, 11.5), -2.5, 2.5, 0.25)


# --------------------------------------------------- multi-purpose bracket
def bracket_multi():
    flange = difference(rect(-14, 36, -13, 13), rect(-10.7, 31, -10.3, 10.3),
                        holes([(34.2, 5), (34.2, -5)], 1.6))
    wall = difference(rect(-12, 32, -45.3, -21.5), holes([(6, -33.4), (22, -33.4)], 4))
    bottom = difference(rect(-14, 32, -13, 13), circle(0, 0, 3), holes([(18, 6), (18, -6)], 1.6))
    back = difference(rect(-13, 13, -45.3, -19.5), circle(0, -32.4, 4),
                      holes([(-7, -26), (7, -26), (-7, -39), (7, -39)], 1.6))
    return merge(
        plate_xy(flange, (-14, 36, -13, 13), -21.5, T, RES),
        plate_xz(wall, (-12, 32, -45.3, -21.5), 11, T, RES),
        plate_xz(wall, (-12, 32, -45.3, -21.5), -13, T, RES),
        plate_xy(bottom, (-14, 32, -13, 13), -45.3, T, RES),
        plate_yz(back, (-13, 13, -45.3, -19.5), -14, T, RES),
        cylinder(4, -46.5, -45.3),
    )


# --------------------------------------------------------------- U bracket
def _u_arm(L, centre_r):
    return difference(union(circle(0, 0, 12), rect(0, L + 2, -12, 12)), circle(0, 0, centre_r),
                      holes(_ring(8), 1.4), circle(L / 2 + 4, 0, 5))


def u_bracket(L):
    base = difference(rect(-12, 12, -50, 2), circle(0, -24, 5),
                      holes([(-7, -15), (7, -15), (-7, -33), (7, -33)], 1.6))
    return merge(
        plate_xy(_u_arm(L, 3.5), (-12, L + 2, -12, 12), 0, T, RES),
        plate_xy(_u_arm(L, 3.0), (-12, L + 2, -12, 12), -50, T, RES),
        plate_yz(base, (-12, 12, -50, 2), L, T, RES),
        cylinder(3.5, -51.5, -50),
    )


# -------------------------------------------------------------------- foot
def foot_plate():
    shape = difference(rounded_rect(-45, 55, -30, 30, 8), circle(-34, 0, 7), circle(42, 0, 7),
                       holes([(42, 17), (42, -17), (-34, 17), (-34, -17)], 3),
                       holes([(-18, -8), (-18, 8), (18, -8), (18, 8)], 1.6))
    return plate_xy(shape, (-45, 55, -30, 30), 0, T, RES)


# ---------------------------------------------------------------- yaw plate
def yaw_plate():
    """Plate on the hip-yaw horn that carries the hip-roll servo bracket (servo frame)."""
    shape = difference(rounded_rect(-14, 34, -26, 26, 4), circle(0, 0, 1.6), holes(_ring(8), 1.4),
                       holes([(-7, -18), (-7, 18), (28, -18), (28, 18), (10, -6), (10, 6), (22, -6), (22, 6)], 1.6),
                       circle(16, -16, 4), circle(16, 16, 4))
    return plate_xy(shape, (-14, 34, -26, 26), 0, T, RES)


# -------------------------------------------------------------------- body
# Body box (robot frame, mm): x -44..44, y -62..62, z 38..132. The two hip-yaw
# servos stand inside it on their mounting tabs, shafts pointing down through
# cut-outs in the floor.
YAW_CUTOUT_Y = (11.0, 52.7)   # floor cut-out for each yaw servo case, |y| range


def chassis_frame():
    floor = difference(rect(-44, 44, -62, 62), holes(grid(-31.75, 31.75, -6.35, 6.35, 12.7), 2.2),
                       holes(grid(-31.75, 31.75, -57.15, -57.15, 12.7), 2.2),
                       holes(grid(-31.75, 31.75, 57.15, 57.15, 12.7), 2.2),
                       rect(-10.3, 10.3, *YAW_CUTOUT_Y), rect(-10.3, 10.3, -YAW_CUTOUT_Y[1], -YAW_CUTOUT_Y[0]))
    top = difference(rect(-44, 44, -62, 62), holes(grid(-31.75, 31.75, -50.8, 50.8, 12.7), 2.2))
    side = difference(rect(-44, 44, 40, 130), holes(grid(-31.75, 31.75, 47, 123, 12.7), 2.2))
    return merge(
        plate_xy(floor, (-44, 44, -62, 62), 38, T, RES),
        plate_xy(top, (-44, 44, -62, 62), 130, T, RES),
        plate_xz(side, (-44, 44, 40, 130), 60, T, RES),
        plate_xz(side, (-44, 44, 40, 130), -62, T, RES),
    )


def chassis_plates():
    shape = difference(rect(-60, 60, 40, 130), holes(grid(-50.8, 50.8, 46.35, 122.55, 12.7), 2.2))
    return merge(plate_yz(shape, (-60, 60, 40, 130), 44, T, RES),
                 plate_yz(shape, (-60, 60, 40, 130), -46, T, RES))


def battery():
    return box(-35, 30, -38, 38, 70, 113)


def pcb():
    return merge(plate_yz(rect(-28, 28, 85, 125), (-28, 28, 85, 125), 46, 1.6, RES),
                 box(47.6, 49.6, -12, 2, 100, 114),
                 box(47.6, 50.6, 8, 20, 92, 100))


# ------------------------------------------------------------- catalogue
@dataclass(frozen=True)
class PartDef:
    build: Callable[[], np.ndarray]
    rgba: tuple
    density: Optional[float] = None   # g/mm^3, used if mass is None
    mass: Optional[float] = None      # g, fixed total mass (density derived)
    description: str = ""


BLACK_AL = (0.15, 0.15, 0.17, 1.0)
SERVO = (0.09, 0.09, 0.10, 1.0)
SILVER = (0.72, 0.74, 0.77, 1.0)
ORANGE = (0.95, 0.47, 0.13, 1.0)

PARTS: Dict[str, PartDef] = {
    "mg996r": PartDef(mg996r, SERVO, mass=55.0, description="TowerPro MG996R servo, 55 g"),
    "servo_horn": PartDef(servo_horn, SILVER, density=ALUMINIUM, description="25T round aluminium horn"),
    "bracket_multi": PartDef(bracket_multi, BLACK_AL, density=ALUMINIUM, description="multi-purpose servo bracket"),
    "u_bracket_24": PartDef(lambda: u_bracket(24), BLACK_AL, density=ALUMINIUM, description="short U bracket (24 mm)"),
    "u_bracket_30": PartDef(lambda: u_bracket(30), BLACK_AL, density=ALUMINIUM, description="medium U bracket (30 mm)"),
    "u_bracket_42": PartDef(lambda: u_bracket(42), BLACK_AL, density=ALUMINIUM, description="long U bracket (42 mm)"),
    "yaw_plate": PartDef(yaw_plate, BLACK_AL, density=ALUMINIUM, description="hip-yaw output plate"),
    "foot_plate": PartDef(foot_plate, BLACK_AL, density=ALUMINIUM, description="foot sole plate"),
    "chassis_frame": PartDef(chassis_frame, SILVER, density=ALUMINIUM, description="body frame"),
    "chassis_plates": PartDef(chassis_plates, ORANGE, density=ALUMINIUM, description="front/back plates"),
    "battery": PartDef(battery, (0.16, 0.17, 0.2, 1.0), mass=250.0, description="2S LiPo + regulator"),
    "pcb": PartDef(pcb, (0.1, 0.45, 0.2, 1.0), mass=45.0, description="controller board"),
}
