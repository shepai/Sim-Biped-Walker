// =====================================================================
//  8-DoF servo-bracket biped: part library (OpenSCAD)
//
//  Hobby "robot servo bracket" style parts sized for MG996R servos:
//    mg996r()          servo body (for fit checks / visuals)
//    servo_horn()      round aluminium horn
//    bracket_multi()   multi-purpose servo bracket (holds one servo)
//    u_bracket(L)      U bracket straddling a servo; L = axis -> base distance
//    foot_plate()      foot sole plate
//    pelvis(), chassis_frame(), chassis_plates(), battery(), pcb()  body
//
//  Units: mm. Sheet thickness T = 2 mm (aluminium).
//
//  Servo frame convention (used by every servo-related part):
//    origin on the output shaft axis, in the plane of the horn's outer face
//    +z   along the output shaft (out of the horn)
//    +x   along the servo length, towards the end away from the shaft
//    y    across the servo width (all parts are symmetric in y)
//  A U bracket rotates about +z; its top arm sits on the horn (z = 0..2) and
//  its bottom arm pivots on the bracket's bearing (z = -50..-48).
//
//  Export a single part, e.g.:
//    openscad -D 'part="u_bracket_36"' -o u_bracket_36.stl biped8_parts.scad
//  (see export_stl.sh). assembly.scad places all parts on the robot.
//
//  Keep in sync with biped_walker/cad/parts.py, which mirrors these
//  dimensions to generate the simulation meshes.
// =====================================================================

T   = 2;      // sheet thickness
$fn = 48;
part = "";    // set with -D to render one part

// ---------------------------------------------------------------- helpers
module plate_xy(z0, t=T) translate([0, 0, z0]) linear_extrude(t) children();
// (u, v, w) -> (x = u, y = y0 + w, z = v)
module plate_xz(y0, t=T) multmatrix([[1,0,0,0],[0,0,1,y0],[0,1,0,0],[0,0,0,1]]) linear_extrude(t) children();
// (u, v, w) -> (x = x0 + w, y = u, z = v)
module plate_yz(x0, t=T) multmatrix([[0,0,1,x0],[1,0,0,0],[0,1,0,0],[0,0,0,1]]) linear_extrude(t) children();

module rect(u0, u1, v0, v1) translate([u0, v0]) square([u1 - u0, v1 - v0]);
module rrect(u0, u1, v0, v1, r) translate([u0 + r, v0 + r]) offset(r = r) square([u1 - u0 - 2*r, v1 - v0 - 2*r]);
module holes(pts, r) for (p = pts) translate(p) circle(r);
function grid(u0, u1, v0, v1, pitch) =
    [for (u = [u0 : pitch : u1]) for (v = [v0 : pitch : v1]) [u, v]];

// ------------------------------------------------------------ MG996R servo
module mg996r() {
    translate([-10.2, -9.85, -43.3]) cube([40.7, 19.7, 36.8]);          // case
    plate_xy(-19.5, 2.5) difference() {                                 // mounting tabs
        rect(-17.2, 37.5, -9.85, 9.85);
        holes([[-13.9, 5], [-13.9, -5], [34.2, 5], [34.2, -5]], 2.2);
    }
    translate([0, 0, -6.5]) cylinder(r = 9, h = 2);                      // gear boss
    translate([12, 0, -6.5]) cylinder(r = 4, h = 1);                     // second boss
    translate([0, 0, -4.5]) cylinder(r = 2.9, h = 2);                    // output spline
    translate([30.5, -3.5, -40]) cube([1.5, 7, 3]);                      // cable exit
}

module servo_horn() plate_xy(-2.5, 2.5) difference() {
    circle(11.5);
    circle(1.6);
    holes([for (a = [0 : 90 : 270]) [8 * cos(a), 8 * sin(a)]], 1.4);
}

// ------------------------------------------------ multi-purpose bracket
module bracket_multi() {
    plate_xy(-21.5) difference() {                                      // flange plate (servo tabs bolt here)
        rect(-14, 36, -13, 13);
        rect(-10.7, 31, -10.3, 10.3);
        holes([[34.2, 5], [34.2, -5]], 1.6);
    }
    for (y0 = [11, -13]) plate_xz(y0) difference() {                    // side walls
        rect(-12, 32, -45.3, -21.5);
        holes([[6, -33.4], [22, -33.4]], 4);
    }
    plate_xy(-45.3) difference() {                                      // bottom plate, bearing hole on the axis
        rect(-14, 32, -13, 13);
        circle(3);
        holes([[18, 6], [18, -6]], 1.6);
    }
    plate_yz(-14) difference() {                                        // back plate = mounting face
        rect(-13, 13, -45.3, -19.5);
        translate([0, -32.4]) circle(4);
        holes([[-7, -26], [7, -26], [-7, -39], [7, -39]], 1.6);
    }
    translate([0, 0, -46.5]) cylinder(r = 4, h = 1.2);                   // flanged bearing
}

// --------------------------------------------------------------- U bracket
module u_arm(L, centre_r) difference() {
    union() { circle(12); rect(0, L + 2, -12, 12); }
    circle(centre_r);
    holes([for (a = [0 : 90 : 270]) [8 * cos(a), 8 * sin(a)]], 1.4);
    translate([L / 2 + 4, 0]) circle(5);
}

module u_bracket(L) {
    plate_xy(0)   u_arm(L, 3.5);                                         // horn side
    plate_xy(-50) u_arm(L, 3);                                           // bearing side
    plate_yz(L) difference() {                                          // base
        rect(-12, 12, -50, 2);
        translate([0, -24]) circle(5);
        holes([[-7, -15], [7, -15], [-7, -33], [7, -33]], 1.6);
    }
    translate([0, 0, -51.5]) cylinder(r = 3.5, h = 1.5);                 // pivot bolt head
}

// ------------------------------------------------------------------- foot
module foot_plate() plate_xy(0) difference() {
    rrect(-45, 55, -30, 30, 8);
    translate([-34, 0]) circle(7);
    translate([42, 0]) circle(7);
    holes([[42, 17], [42, -17], [-34, 17], [-34, -17]], 3);
    holes([[-18, -8], [-18, 8], [18, -8], [18, 8]], 1.6);
}

// ------------------------------------------------------------------- body
// Body parts are drawn directly in the robot frame: x forward, y left, z up,
// origin midway between the hip-roll axes.
module pelvis() {
    plate_xy(14) difference() {
        rect(-30, 30, -62, 62);
        holes(grid(-20, 20, -50, 50, 20), 2.5);
    }
    for (x0 = [-30, 28]) plate_yz(x0) difference() {
        rect(-62, 62, 16, 40);
        holes(grid(-50.8, 50.8, 30, 30, 12.7), 2.2);
    }
}

module chassis_frame() {
    for (z0 = [38, 130]) plate_xy(z0) difference() {
        rect(-44, 44, -47, 47);
        holes(grid(-31.75, 31.75, -31.75, 31.75, 12.7), 2.2);
    }
    for (y0 = [45, -47]) plate_xz(y0) difference() {
        rect(-44, 44, 40, 130);
        holes(grid(-31.75, 31.75, 47, 123, 12.7), 2.2);
    }
}

module chassis_plates() for (x0 = [44, -46]) plate_yz(x0) difference() {
    rect(-45, 45, 40, 130);
    holes(grid(-38.1, 38.1, 46.35, 122.55, 12.7), 2.2);
}

module battery() translate([-35, -38, 42]) cube([65, 76, 43]);

module pcb() {
    plate_yz(46, 1.6) rect(-28, 28, 85, 125);
    translate([47.6, -12, 100]) cube([2, 14, 14]);
    translate([47.6, 8, 92]) cube([3, 12, 8]);
}

// ----------------------------------------------------------------- export
if (part == "mg996r")          mg996r();
if (part == "servo_horn")      servo_horn();
if (part == "bracket_multi")   bracket_multi();
if (part == "u_bracket_24")    u_bracket(24);
if (part == "u_bracket_30")    u_bracket(30);
if (part == "u_bracket_36")    u_bracket(36);
if (part == "foot_plate")      foot_plate();
if (part == "pelvis")          pelvis();
if (part == "chassis_frame")   chassis_frame();
if (part == "chassis_plates")  chassis_plates();
if (part == "battery")         battery();
if (part == "pcb")             pcb();
