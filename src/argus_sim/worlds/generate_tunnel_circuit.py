#!/usr/bin/env python3
"""ARGUS :: generate_tunnel_circuit.py — emit tunnel_circuit.sdf (Scenario E).

A 202.8 m closed-circuit service TUNNEL for the DP7 long-distance drift gate
("VIO drift < 1.5 % over 200 m"). Stadium plan: two 70 m straights joined by
two r = 10 m semicircular ends; cross-section 6 m wide x 3.5 m tall.

    perimeter = 2*L + 2*pi*r = 140 + 62.83 = 202.83 m   (L = 70, r = 10)

The circuit is designed around what Day 1-6 taught about VINS-Fusion in this
sim, so the geometry IS the experiment design:

  * SEGMENTED wall panels (5 m straights / 15-deg arc chords), alternating two
    PBR tints of the contract `detail.png`. An SDF box maps the albedo texture
    once per face, so one 70 m wall = one hopelessly stretched texture; per-
    panel boxes re-tile it every few metres -> dense, trackable detail
    EVERYWHERE (the 30 m corridor's single-box walls were feature-starved).
  * Arch RIBS (pilaster pair + ceiling beam) every ~10 m: strong vertical
    edges and near-field parallax, the cue KLT + stereo triangulation love.
  * Distinctive colour SIGNAGE every ~12.7 m at camera height, alternating
    walls: locally unique landmarks (loop-closure / DBoW cues).
  * GENTLE ends: at 0.8 m/s the semicircle is wz = v/r = 0.08 rad/s of yaw
    WHILE translating -- always-positive parallax. (In-place U-turns, which
    diverge VINS, never occur; cf. fly_uturn_laps.py post-mortem.)
  * Closed circuit: one lap returns to the spawn -> the pose graph closes the
    loop exactly where drift is measured.

Everything matches the frozen Day-1 contract otherwise: dartsim @ 250 Hz, the
same system plugins, SI/ENU, drone spawn (1.5, 0, 1.0) lies on the first
straight's centreline heading +x. The warehouse_corridor world is untouched;
select this one with `world:=tunnel_circuit`.

Regenerate with:  python3 src/argus_sim/worlds/generate_tunnel_circuit.py
then rebuild argus_sim (the .sdf is symlink-installed).
"""

import math
import os

# ----------------------------------------------------------------------- geometry
L = 70.0          # straight length (m)
R = 10.0          # end-cap centreline radius (m)
W = 3.0           # half-width: walls at lateral +/- 3.0 m
H = 3.5           # interior height (m)
PERIM = 2 * L + 2 * math.pi * R          # 202.83 m
CY = R            # arc centres at (0, R) and (L, R); straights at y=0 / y=2R

PANEL = 5.0       # straight wall panel length (m)
ARC_N = 12        # wall segments per semicircle (15 deg each)
THICK = 0.2       # wall panel thickness


def centerline(s):
    """Point, tangent heading on the CCW stadium centreline at arc-length s."""
    s = s % PERIM
    if s < L:                                   # straight A: (0,0)->(L,0)
        return s, 0.0, 0.0
    s -= L
    if s < math.pi * R:                         # right end-cap, CCW about (L, R)
        phi = -math.pi / 2 + s / R
        return L + R * math.cos(phi), CY + R * math.sin(phi), phi + math.pi / 2
    s -= math.pi * R
    if s < L:                                   # straight B: (L,2R)->(0,2R)
        return L - s, 2 * R, math.pi
    s -= L                                      # left end-cap, CCW about (0, R)
    phi = math.pi / 2 + s / R
    return R * math.cos(phi), CY + R * math.sin(phi), phi + math.pi / 2


# ----------------------------------------------------------------------- helpers
def box(name, x, y, z, sx, sy, sz, mat, *, yaw=0.0, collision=True):
    col = (f'\n        <collision name="col"><geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}'
           f'</size></box></geometry></collision>') if collision else ''
    return f'''    <model name="{name}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.4f}</pose>
      <link name="link">{col}
        <visual name="vis"><geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>
          {mat}</visual>
      </link>
    </model>
'''


def cyl(name, x, y, z, r, l, mat, *, rpy="0 0 0", collision=False):
    col = (f'\n        <collision name="col"><geometry><cylinder><radius>{r}</radius>'
           f'<length>{l}</length></cylinder></geometry></collision>') if collision else ''
    return f'''    <model name="{name}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {z:.3f} {rpy}</pose>
      <link name="link">{col}
        <visual name="vis"><geometry><cylinder><radius>{r}</radius><length>{l:.3f}</length></cylinder></geometry>
          {mat}</visual>
      </link>
    </model>
'''


def mat(ambient, diffuse, specular="0.1 0.1 0.1 1", *, texture=None, rough=0.85, metal=0.0):
    pbr = ''
    if texture:
        pbr = (f'<pbr><metal><albedo_map>{texture}</albedo_map>'
               f'<roughness>{rough}</roughness><metalness>{metal}</metalness></metal></pbr>')
    return (f'<material><ambient>{ambient}</ambient><diffuse>{diffuse}</diffuse>'
            f'<specular>{specular}</specular>{pbr}</material>')


# Palette — concrete panel tints (both carry detail.png), trim + landmarks.
CONC_A = mat("0.50 0.50 0.52 1", "0.80 0.80 0.82 1", texture="detail.png", rough=0.92)
CONC_B = mat("0.42 0.44 0.48 1", "0.68 0.71 0.76 1", texture="detail.png", rough=0.9)
FLOOR_M = mat("0.40 0.40 0.42 1", "0.70 0.70 0.72 1", texture="detail.png", rough=0.95)
CEIL_M = mat("0.55 0.55 0.57 1", "0.66 0.66 0.68 1")
RIB_M = mat("0.10 0.16 0.30 1", "0.16 0.26 0.48 1", "0.3 0.3 0.35 1", rough=0.5, metal=0.7)
PIPE_M = mat("0.35 0.36 0.40 1", "0.55 0.57 0.62 1", "0.4 0.4 0.4 1", rough=0.4, metal=0.6)
TRAY_M = mat("0.45 0.30 0.05 1", "0.75 0.52 0.10 1", rough=0.6, metal=0.3)
DASH_M = mat("0.7 0.7 0.7 1", "0.92 0.92 0.92 1")
HAZARD = mat("0.85 0.75 0.0 1", "1.0 0.85 0.0 1")
SIGN_C = [
    mat("0.0 0.5 0.2 1", "0.0 0.7 0.3 1"),     # green
    mat("0.6 0.45 0.0 1", "0.85 0.65 0.0 1"),  # amber
    mat("0.0 0.3 0.6 1", "0.0 0.4 0.85 1"),    # blue
    mat("0.55 0.08 0.08 1", "0.82 0.12 0.12 1"),  # red
    mat("0.35 0.10 0.45 1", "0.55 0.16 0.70 1"),  # violet
]


# ----------------------------------------------------------------------- walls
def straight_walls(tag, y_wall, x0, x1, z=H / 2):
    """Segmented panel wall along x from x0..x1 at lateral line y_wall."""
    s, n = '', 0
    x = min(x0, x1)
    end = max(x0, x1)
    while x < end - 1e-6:
        seg = min(PANEL, end - x)
        s += box(f"tw_{tag}_{n}", x + seg / 2, y_wall, z,
                 seg + 0.06, THICK, H, CONC_A if n % 2 == 0 else CONC_B)
        x += seg
        n += 1
    return s


def arc_walls(tag, cx, radius, phi0, z=H / 2):
    """Segmented panel ring: ARC_N chord boxes over a semicircle from phi0."""
    s = ''
    dphi = math.pi / ARC_N
    chord = 2 * radius * math.sin(dphi / 2)
    for i in range(ARC_N):
        phi = phi0 + (i + 0.5) * dphi
        x = cx + radius * math.cos(phi)
        y = CY + radius * math.sin(phi)
        # box long axis along the chord = tangent at mid-angle (phi + 90 deg)
        s += box(f"tw_{tag}_{i}", x, y, z, chord * 1.08, THICK, H,
                 CONC_A if i % 2 == 0 else CONC_B, yaw=phi + math.pi / 2)
    return s


# ----------------------------------------------------------------------- assembly
def build():
    parts = []

    # ---- floor + ceiling slabs over the full stadium bounding box ----
    bx, by = 35.0, CY                       # bounding-box centre
    BX, BY = 2 * (L / 2 + R + W) + 8, 2 * (R + W) + 2 * R + 8   # 104 x 34
    parts.append(box("floor", bx, by, -0.1, BX, BY, 0.2, FLOOR_M))
    parts.append(box("ceiling", bx, by, H + 0.1, BX, BY, 0.2, CEIL_M))

    # ---- wall rings (outer at centreline+W, inner at centreline-W) ----
    parts.append(straight_walls("Ao", -W, 0.0, L))            # straight A outer
    parts.append(straight_walls("Ai", +W, 0.0, L))            # straight A inner
    parts.append(straight_walls("Bo", 2 * R + W, 0.0, L))     # straight B outer
    parts.append(straight_walls("Bi", 2 * R - W, 0.0, L))     # straight B inner
    parts.append(arc_walls("Ro", L, R + W, -math.pi / 2))     # right cap outer
    parts.append(arc_walls("Ri", L, R - W, -math.pi / 2))     # right cap inner
    parts.append(arc_walls("Lo", 0.0, R + W, math.pi / 2))    # left cap outer
    parts.append(arc_walls("Li", 0.0, R - W, math.pi / 2))    # left cap inner

    # ---- arch ribs every ~10 m of centreline arc-length ----
    n_ribs = 20
    for k in range(n_ribs):
        s = k * PERIM / n_ribs
        x, y, th = centerline(s)
        lx, ly = -math.sin(th), math.cos(th)          # left unit vector
        for side, sgn in (("p", +1), ("m", -1)):      # pilaster pair
            px, py = x + sgn * (W - 0.18) * lx, y + sgn * (W - 0.18) * ly
            parts.append(box(f"rib{k}_{side}", px, py, H / 2,
                             0.30, 0.25, H, RIB_M, yaw=th))
        parts.append(box(f"rib{k}_c", x, y, H - 0.16,            # ceiling beam
                         0.30, 2 * W - 0.3, 0.28, RIB_M, yaw=th))

    # ---- signage placards at camera height, alternating walls ----
    n_sign = 16
    for k in range(n_sign):
        s = (k + 0.5) * PERIM / n_sign
        x, y, th = centerline(s)
        lx, ly = -math.sin(th), math.cos(th)
        sgn = 1 if k % 2 == 0 else -1
        px, py = x + sgn * (W - 0.16) * lx, y + sgn * (W - 0.16) * ly
        parts.append(box(f"sign{k}", px, py, 1.5, 1.2, 0.05, 0.8,
                         SIGN_C[k % len(SIGN_C)], yaw=th, collision=False))

    # ---- service pipes along the straights (above / below flight band) ----
    for tag, ywall, off in (("A", -W, 0.22), ("B", 2 * R + W, -0.22)):
        parts.append(cyl(f"pipe_{tag}", L / 2, ywall + off, 2.9, 0.06, L - 2,
                         PIPE_M, rpy="0 1.5708 0"))
        parts.append(cyl(f"tray_{tag}", L / 2, ywall + off, 0.9, 0.05, L - 2,
                         TRAY_M, rpy="0 1.5708 0"))

    # ---- floor guidance dashes along the whole centreline ----
    n_dash = 78
    for k in range(n_dash):
        s = k * PERIM / n_dash
        x, y, th = centerline(s)
        parts.append(box(f"dash{k}", x, y, 0.012, 0.8, 0.12, 0.02,
                         DASH_M, yaw=th, collision=False))

    # ---- init garden: near-field 3D structure through the VINS init window ----
    # The corridor world inits at ~1.3 deg tilt with obstacles/racking giving
    # close-range parallax; bare tunnel walls alone initialised at 13-26 deg
    # (day-7 iterations 3-4). These pillars/crates sit at |y| 1.8-2.2 m — clear
    # of the |y|<=1.0 m flight lane — covering the first ~14 m of the lap.
    PILLAR = mat("0.33 0.34 0.37 1", "0.48 0.49 0.53 1", "0.3 0.3 0.3 1", rough=0.4, metal=0.6)
    CRATE_C = [
        mat("0.40 0.28 0.14 1", "0.62 0.44 0.22 1", rough=0.95),
        mat("0.07 0.16 0.42 1", "0.12 0.28 0.70 1", "0.3 0.3 0.4 1", rough=0.3),
        mat("0.45 0.07 0.07 1", "0.78 0.13 0.13 1", rough=0.5),
        mat("0.06 0.35 0.12 1", "0.10 0.55 0.20 1", rough=0.6),
    ]
    for k, (gx, side) in enumerate(((4.0, +1), (6.5, -1), (9.0, +1), (11.5, -1), (14.0, +1))):
        gy = side * 2.0
        if k % 2 == 0:
            parts.append(cyl(f"initpillar{k}", gx, gy, 1.1, 0.18, 2.2,
                             PILLAR, collision=True))
        else:
            parts.append(box(f"initcrate{k}a", gx, gy, 0.35, 0.7, 0.7, 0.7,
                             CRATE_C[k % 4]))
            parts.append(box(f"initcrate{k}b", gx + 0.1, gy - side * 0.15, 0.95,
                             0.5, 0.5, 0.5, CRATE_C[(k + 1) % 4]))

    # ---- hazard stripes across the lane at each end-cap entry/exit ----
    for k, s in enumerate((L - 2.0, L + math.pi * R + 2.0,
                           2 * L + math.pi * R - 2.0, PERIM - 2.0)):
        x, y, th = centerline(s)
        parts.append(box(f"hazard{k}", x, y, 0.013, 0.35, 2 * W - 0.4, 0.02,
                         HAZARD, yaw=th, collision=False))

    body = "\n".join(parts)

    # ---- lights: 16 ceiling points along the centreline + directional fill ----
    def point_light(name, x, y, shadows):
        return f'''    <light type="point" name="{name}">
      <pose>{x:.2f} {y:.2f} {H - 0.25:.2f} 0 0 0</pose>
      <diffuse>0.95 0.93 0.85 1</diffuse><specular>0.2 0.2 0.2 1</specular><intensity>1.0</intensity>
      <attenuation><range>16</range><constant>0.3</constant><linear>0.07</linear><quadratic>0.01</quadratic></attenuation>
      <cast_shadows>{'true' if shadows else 'false'}</cast_shadows>
    </light>
'''
    lights = f'''    <light type="directional" name="fill">
      <pose>{bx} {by} 8 0 0 0</pose><direction>0.2 0.15 -1</direction>
      <diffuse>0.35 0.35 0.35 1</diffuse><specular>0.05 0.05 0.05 1</specular>
      <intensity>0.5</intensity><cast_shadows>false</cast_shadows>
    </light>
'''
    for k in range(16):
        x, y, _ = centerline((k + 0.25) * PERIM / 16)
        lights += point_light(f"light_t{k}", x, y, shadows=(k % 4 == 0))

    return f'''<?xml version="1.0" ?>
<!--
  ARGUS :: tunnel_circuit.sdf  (GENERATED by worlds/generate_tunnel_circuit.py)
  Scenario E: 202.83 m closed-circuit tunnel for the DP7 200 m drift gate.

  FRAME / UNITS (frozen contract): SI (m, kg, s, rad). World = ENU.
    Stadium centreline: straights y=0 (x 0..70) and y=20 (x 70..0), semicircular
    end-caps r=10 about (0,10)/(70,10). Cross-section 6 m wide x 3.5 m tall.
    Drone spawn (1.5, 0, 1.0) on the first straight, heading +x, CCW lap.
  Physics: dartsim @ 250 Hz (identical to warehouse_corridor).

  Hand-edit generate_tunnel_circuit.py, NOT this file.
-->
<sdf version="1.10">
  <world name="tunnel_circuit">

    <physics name="dartsim_250hz" type="dart">
      <max_step_size>0.004</max_step_size>
      <!-- RTF capped at 0.5 ON PURPOSE (recording robustness): the Scenario E
           sensor bag writes ~80 MB/s of stereo rgb8 at RTF 1.0, which stalls
           writeback on a 14 GB host, collapses the sim and OOMs the recorder
           (day-7). Half-pace halves the disk rate; physics step, sensor rates
           and all sim-time data are IDENTICAL — recording just takes 2x wall. -->
      <real_time_factor>0.5</real_time_factor>
      <real_time_update_rate>125</real_time_update_rate>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>

    <scene>
      <ambient>0.32 0.32 0.35 1</ambient>
      <background>0.05 0.05 0.07 1</background>
      <grid>false</grid>
      <shadows>true</shadows>
    </scene>

{lights}
{body}
  </world>
</sdf>
'''


if __name__ == "__main__":
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "tunnel_circuit.sdf")
    with open(out, "w") as f:
        f.write(build())
    print(f"wrote {out}  (perimeter = {PERIM:.2f} m)")
