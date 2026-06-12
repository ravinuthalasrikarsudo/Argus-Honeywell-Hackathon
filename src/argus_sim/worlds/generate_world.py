#!/usr/bin/env python3
"""ARGUS :: generate_world.py — emit the enriched warehouse_corridor.sdf.

The contract values (corridor 30x5x3 m, dartsim 250 Hz, the gz
system plugins, the six SLALOM obstacles, the Zone-B `light_b_flicker` fixture,
the `detail.png` PBR texture) are reproduced EXACTLY -- the reactive avoider is
tuned and verified against those obstacle poses, so they must not move. On top of
that frozen core this script lays in the warehouse "dressing" that makes the scene
read as a real facility on screen:

  * industrial pallet RACKING bays down both side walls, loaded with palletised
    cardboard cartons (steel-blue uprights, safety-orange beams);
  * a parked FORKLIFT silhouette near the dock end;
  * overhead CONDUIT / sprinkler pipe runs along the ceiling;
  * floor DRESSING: a dashed centre guide-lane, yellow aisle safety borders, the
    zone boundary stripes and the A/B/C wall placards.

EVERYTHING added here is kept clear of the drone's flight lane: scenery hugs the
walls (inner face no closer than |y| = 2.0 m, the path only needs |y| <= 1.0 m) or
sits above the obstacle z-band (z > 2.0 m, the avoider gates |dz| < 0.8 m around
its 1.0 m cruise altitude). So the look changes; the verified slalom does not.

Regenerate with:  python3 src/argus_sim/worlds/generate_world.py
then rebuild argus_sim (the .sdf is symlink-installed).
"""

import os

# ----------------------------------------------------------------------------- helpers
def box(name, x, y, z, sx, sy, sz, mat, *, rpy="0 0 0", static=True, collision=True):
    col = (f'\n        <collision name="col"><geometry><box><size>{sx} {sy} {sz}'
           f'</size></box></geometry></collision>') if collision else ''
    return f'''    <model name="{name}">
      <static>{'true' if static else 'false'}</static>
      <pose>{x} {y} {z} {rpy}</pose>
      <link name="link">{col}
        <visual name="vis"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          {mat}</visual>
      </link>
    </model>
'''


def cyl(name, x, y, z, r, l, mat, *, rpy="0 0 0", static=True, collision=True):
    col = (f'\n        <collision name="col"><geometry><cylinder><radius>{r}</radius>'
           f'<length>{l}</length></cylinder></geometry></collision>') if collision else ''
    return f'''    <model name="{name}">
      <static>{'true' if static else 'false'}</static>
      <pose>{x} {y} {z} {rpy}</pose>
      <link name="link">{col}
        <visual name="vis"><geometry><cylinder><radius>{r}</radius><length>{l}</length></cylinder></geometry>
          {mat}</visual>
      </link>
    </model>
'''


def mat(ambient, diffuse, specular="0.1 0.1 0.1 1", *, texture=None, rough=0.8, metal=0.0):
    pbr = ''
    if texture:
        pbr = (f'<pbr><metal><albedo_map>{texture}</albedo_map>'
               f'<roughness>{rough}</roughness><metalness>{metal}</metalness></metal></pbr>')
    return (f'<material><ambient>{ambient}</ambient><diffuse>{diffuse}</diffuse>'
            f'<specular>{specular}</specular>{pbr}</material>')


# Palette
STEEL   = mat("0.10 0.16 0.30 1", "0.16 0.26 0.48 1", "0.3 0.3 0.35 1", rough=0.5, metal=0.7)   # rack uprights
ORANGE  = mat("0.55 0.27 0.0 1",  "0.95 0.45 0.05 1", "0.3 0.2 0.1 1",  rough=0.5, metal=0.4)   # rack beams
CARD_A  = mat("0.42 0.30 0.16 1", "0.72 0.55 0.34 1", rough=0.95)                               # cardboard
CARD_B  = mat("0.38 0.27 0.14 1", "0.66 0.49 0.29 1", rough=0.95)
SHRINK  = mat("0.20 0.22 0.28 1", "0.35 0.40 0.55 1", "0.5 0.5 0.6 1", rough=0.3)               # shrink-wrapped pallet
YELLOW  = mat("0.55 0.48 0.0 1",  "0.95 0.85 0.0 1")
PIPE    = mat("0.35 0.36 0.40 1", "0.55 0.57 0.62 1", "0.4 0.4 0.4 1", rough=0.4, metal=0.6)
REDPIPE = mat("0.45 0.05 0.05 1", "0.80 0.12 0.12 1", rough=0.4, metal=0.5)


# ----------------------------------------------------------------------------- racking
def racking_bay(tag, cx, side):
    """A pallet-racking bay flush to a side wall (inner face at |y|=2.0 m).
    side = +1 (+Y wall) or -1 (-Y wall). Bay is 2.4 m wide, 0.5 m deep, 2.8 m tall."""
    yc = side * 2.25               # bay centre (wall inner face at 2.5; bay 2.0..2.5)
    s = ''
    # two uprights
    for i, dx in enumerate((-1.15, 1.15)):
        s += box(f"rack_{tag}_up{i}", cx + dx, yc, 1.4, 0.1, 0.5, 2.8, STEEL)
    # three load beams
    for i, bz in enumerate((0.85, 1.65, 2.45)):
        s += box(f"rack_{tag}_beam{i}", cx, yc, bz, 2.4, 0.5, 0.08, ORANGE)
    # palletised cartons on the lower two levels
    levels = ((0.95, CARD_A, SHRINK), (1.75, CARD_B, CARD_A))
    for li, (bz, mA, mB) in enumerate(levels):
        for j, (dx, m) in enumerate(((-0.7, mA), (0.05, mB), (0.75, mA))):
            s += box(f"rack_{tag}_box{li}{j}", cx + dx, yc, bz + 0.30,
                     0.55, 0.45, 0.55, m)
    return s


def forklift(tag, x, y, yaw):
    """A blocky parked forklift (counterweight body + cab mast + two forks)."""
    BODY = mat("0.55 0.42 0.0 1", "0.92 0.72 0.05 1", "0.3 0.3 0.2 1", rough=0.5, metal=0.3)
    DARK = mat("0.08 0.08 0.09 1", "0.15 0.15 0.17 1", "0.3 0.3 0.3 1", rough=0.4, metal=0.5)
    s = ''
    s += box(f"{tag}_body",  x, y, 0.45, 1.2, 0.9, 0.7, BODY, rpy=f"0 0 {yaw}")
    s += box(f"{tag}_cab",   x - 0.1, y, 1.25, 0.5, 0.8, 0.8, DARK, rpy=f"0 0 {yaw}")
    s += box(f"{tag}_mast",  x + 0.7, y, 1.0, 0.12, 0.7, 2.0, DARK, rpy=f"0 0 {yaw}")
    for i, dy in enumerate((-0.25, 0.25)):
        s += box(f"{tag}_fork{i}", x + 1.1, y + dy, 0.08, 0.9, 0.12, 0.06, DARK, rpy=f"0 0 {yaw}")
    return s


# ----------------------------------------------------------------------------- assembly
def build():
    parts = []

    # ---- structural shell (PBR detail texture; corridor 30 x 5 x 3) ----
    DETAIL_FLOOR = mat("0.45 0.45 0.47 1", "0.78 0.78 0.80 1", texture="detail.png", rough=0.92)
    DETAIL_WALL  = mat("0.55 0.55 0.55 1", "0.82 0.82 0.80 1", texture="detail.png", rough=0.9)
    CEIL         = mat("0.70 0.70 0.72 1", "0.80 0.80 0.82 1")
    shell = f'''    <model name="warehouse_shell">
      <static>true</static>
      <link name="structure">
        <collision name="floor_col"><pose>15 0 -0.1 0 0 0</pose><geometry><box><size>30 5 0.2</size></box></geometry></collision>
        <visual name="floor_vis"><pose>15 0 -0.1 0 0 0</pose><geometry><box><size>30 5 0.2</size></box></geometry>
          {DETAIL_FLOOR}</visual>
        <collision name="ceiling_col"><pose>15 0 3.1 0 0 0</pose><geometry><box><size>30 5 0.2</size></box></geometry></collision>
        <visual name="ceiling_vis"><pose>15 0 3.1 0 0 0</pose><geometry><box><size>30 5 0.2</size></box></geometry>
          {CEIL}</visual>
        <collision name="wall_left_col"><pose>15 2.6 1.5 0 0 0</pose><geometry><box><size>30 0.2 3</size></box></geometry></collision>
        <visual name="wall_left_vis"><pose>15 2.6 1.5 0 0 0</pose><geometry><box><size>30 0.2 3</size></box></geometry>
          {DETAIL_WALL}</visual>
        <collision name="wall_right_col"><pose>15 -2.6 1.5 0 0 0</pose><geometry><box><size>30 0.2 3</size></box></geometry></collision>
        <visual name="wall_right_vis"><pose>15 -2.6 1.5 0 0 0</pose><geometry><box><size>30 0.2 3</size></box></geometry>
          {DETAIL_WALL}</visual>
        <collision name="wall_x0_col"><pose>-0.1 0 1.5 0 0 0</pose><geometry><box><size>0.2 5 3</size></box></geometry></collision>
        <visual name="wall_x0_vis"><pose>-0.1 0 1.5 0 0 0</pose><geometry><box><size>0.2 5 3</size></box></geometry>
          {DETAIL_WALL}</visual>
        <collision name="wall_x30_col"><pose>30.1 0 1.5 0 0 0</pose><geometry><box><size>0.2 5 3</size></box></geometry></collision>
        <visual name="wall_x30_vis"><pose>30.1 0 1.5 0 0 0</pose><geometry><box><size>0.2 5 3</size></box></geometry>
          {DETAIL_WALL}</visual>
      </link>
    </model>
'''
    parts.append(shell)

    # ---- floor dressing ----
    floor = []
    # zone boundary stripes (kept from contract)
    floor.append(box("zone_stripe_AB", 10, 0, 0.011, 0.3, 5, 0.02,
                     mat("0.85 0.75 0.0 1", "1.0 0.85 0.0 1"), collision=False))
    floor.append(box("zone_stripe_BC", 20, 0, 0.011, 0.3, 5, 0.02,
                     mat("0.85 0.4 0.0 1", "1.0 0.5 0.0 1"), collision=False))
    # dashed centre guide-lane
    xc = 2.0
    i = 0
    while xc <= 28.0:
        floor.append(box(f"lane_{i}", xc, 0, 0.012, 0.8, 0.12, 0.02,
                         mat("0.7 0.7 0.7 1", "0.92 0.92 0.92 1"), collision=False))
        xc += 1.6
        i += 1
    # yellow aisle safety borders along both walls
    for s, tag in ((1, "L"), (-1, "R")):
        floor.append(box(f"safety_border_{tag}", 15, s * 2.0, 0.012, 30, 0.12, 0.02,
                         YELLOW, collision=False))
    parts.extend(floor)

    # ---- wall placards (zone landmarks, kept from contract) ----
    parts.append(box("placard_A", 5, 2.49, 1.9, 1.6, 0.04, 1.0,
                     mat("0.0 0.5 0.2 1", "0.0 0.7 0.3 1"), collision=False))
    parts.append(box("placard_B", 15, 2.49, 1.9, 1.6, 0.04, 1.0,
                     mat("0.6 0.45 0.0 1", "0.85 0.65 0.0 1"), collision=False))
    parts.append(box("placard_C", 25, 2.49, 1.9, 1.6, 0.04, 1.0,
                     mat("0.0 0.3 0.6 1", "0.0 0.4 0.85 1"), collision=False))

    # ---- pallet racking down both walls (clear of the slalom + the path) ----
    for cx in (2.5, 9.5, 20.0, 27.5):          # +Y wall
        parts.append(racking_bay(f"L{int(cx*10)}", cx, +1))
    for cx in (5.5, 12.0, 24.5):               # -Y wall
        parts.append(racking_bay(f"R{int(cx*10)}", cx, -1))

    # ---- parked forklift near the dock end, hard against the -Y wall ----
    parts.append(forklift("forklift", 28.5, -2.0, 1.5708))

    # ---- overhead conduit / sprinkler pipes (above the flight band) ----
    for i, (yy, m) in enumerate(((1.6, PIPE), (0.0, REDPIPE), (-1.6, PIPE))):
        parts.append(cyl(f"pipe_{i}", 15, yy, 2.75, 0.05, 29.0, m,
                         rpy="0 1.5708 0", collision=False))

    # ======================= SLALOM OBSTACLES (frozen poses) =======================
    # Positions/sizes are EXACTLY the verified layout; only materials upgraded.
    PALLET = mat("0.30 0.20 0.10 1", "0.52 0.36 0.18 1", rough=0.95)        # wood
    PILLAR = mat("0.33 0.34 0.37 1", "0.48 0.49 0.53 1", "0.3 0.3 0.3 1", rough=0.4, metal=0.6)
    SHELF  = mat("0.20 0.22 0.26 1", "0.32 0.35 0.42 1", "0.4 0.4 0.45 1", rough=0.4, metal=0.5)
    BARREL = mat("0.07 0.16 0.42 1", "0.12 0.28 0.70 1", "0.3 0.3 0.4 1", rough=0.3)   # blue plastic
    CRATE1 = mat("0.40 0.28 0.14 1", "0.62 0.44 0.22 1", rough=0.95)
    CRATE2 = mat("0.46 0.32 0.16 1", "0.68 0.48 0.24 1", rough=0.95)
    DRUM   = mat("0.45 0.07 0.07 1", "0.78 0.13 0.13 1", "0.3 0.2 0.2 1", rough=0.4, metal=0.3)  # hazard drum

    obstacles = [
        box("obs_a_pallet",     4, -1.2, 0.5,  1.0, 1.2, 1.0, PALLET),
        cyl("obs_a_pillar",     7,  1.5, 1.5,  0.2, 3.0,      PILLAR),
        box("obs_b_shelf",     14,  1.4, 1.25, 0.8, 2.0, 2.5, SHELF),
        cyl("obs_b_barrel",    17, -1.3, 0.5,  0.35, 1.0,     BARREL),
        box("obs_c_crate_low", 23, -1.4, 0.4,  1.2, 1.2, 0.8, CRATE1),
        box("obs_c_crate_high",23, -1.4, 1.1,  0.8, 0.8, 0.6, CRATE2),
        cyl("obs_c_drum",      26,  1.3, 0.75, 0.3, 1.5,      DRUM),
    ]
    parts.extend(obstacles)

    body = "\n".join(parts)

    # ---- lights: directional fill + 6 ceiling point lights (Zone-B flicker target) ----
    def point_light(name, x, shadows):
        return f'''    <light type="point" name="{name}">
      <pose>{x} 0 2.8 0 0 0</pose>
      <diffuse>0.95 0.93 0.85 1</diffuse><specular>0.2 0.2 0.2 1</specular><intensity>1.0</intensity>
      <attenuation><range>14</range><constant>0.3</constant><linear>0.08</linear><quadratic>0.01</quadratic></attenuation>
      <cast_shadows>{'true' if shadows else 'false'}</cast_shadows>
    </light>
'''
    lights = f'''    <light type="directional" name="fill">
      <pose>15 0 6 0 0 0</pose><direction>0.2 0.15 -1</direction>
      <diffuse>0.35 0.35 0.35 1</diffuse><specular>0.05 0.05 0.05 1</specular>
      <intensity>0.5</intensity><cast_shadows>false</cast_shadows>
    </light>
'''
    lights += point_light("light_a1", 2.5, False)
    lights += point_light("light_a2", 7.5, True)
    lights += point_light("light_b1", 12.5, False)
    lights += point_light("light_b_flicker", 17.5, False)   # name preserved for flicker_light.sh
    lights += point_light("light_c1", 22.5, True)
    lights += point_light("light_c2", 27.5, False)

    return f'''<?xml version="1.0" ?>
<!--
  ARGUS :: warehouse_corridor.sdf  (GENERATED by worlds/generate_world.py)
  Gazebo Harmonic (gz-sim8) warehouse corridor for the stereo/IMU/LiDAR VIO drone.

  FRAME / UNITS (frozen contract): SI (m, kg, s, rad). World = ENU.
    +X corridor length (0->30 m), +Y width (-2.5->2.5 m), +Z up (0->3 m).
    Interior 30 x 5 x 3 m. Zones A=[0,10] B=[10,20] C=[20,30]. Drone spawn (1.5,0,1.0).
  Physics: dartsim @ 250 Hz (no ODE engine in gz-harmonic; dart is the default).

  Hand-edit generate_world.py, NOT this file. The six obs_* slalom obstacles keep
  their verified poses; the racking / forklift / pipes / floor markings are
  scenery kept clear of the |y|<=1.0 m flight lane (walls >= |y|=2.0; pipes z>2.0).
-->
<sdf version="1.10">
  <world name="warehouse_corridor">

    <physics name="dartsim_250hz" type="dart">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
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
      <ambient>0.35 0.35 0.38 1</ambient>
      <background>0.70 0.72 0.75 1</background>
      <grid>false</grid>
      <shadows>true</shadows>
    </scene>

{lights}
{body}
  </world>
</sdf>
'''


if __name__ == "__main__":
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "warehouse_corridor.sdf")
    with open(out, "w") as f:
        f.write(build())
    print("wrote", out)
