# argus_nav — dense perception & reactive navigation (Pillar 4)

GPS-free sense-and-avoid: three nodes launched by `argus_nav.launch.py`
(`enable_depth` / `enable_mapper` / `enable_avoider`).

| Node | Function |
|------|----------|
| `stereo_depth.py` | SGBM + WLS dense stereo depth (2× decimated), SOR outlier removal, 0.3–12 m gating → `/argus/depth/{image,points}` |
| `occupancy_mapper.py` | Log-odds voxel fusion with free-space ray carving, neighbour-support speck filter, motion gate, AABB world bounds, decay → `/argus/map/points` |
| `reactive_avoider.py` | Potential-field controller (goal attraction + sector-binned obstacle repulsion + wall repulsion), rangefinder altitude PID, local-minimum escape, 0.8 m/s envelope → `/argus/cmd_vel` |

Sensor fusion: stereo depth + 3D LiDAR + downward rangefinder. Flight control
dead-reckons distance (no absolute pose); see `docker/demo.sh --avoid` for the
live autonomous demo and root README §6.

`rviz/argus_nav.rviz` is the demo view (fused map + trajectory).
