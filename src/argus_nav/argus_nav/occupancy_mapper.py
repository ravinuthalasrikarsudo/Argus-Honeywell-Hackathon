#!/usr/bin/env python3
"""ARGUS occupancy_mapper -- temporal log-odds terrain fusion.

The single-frame ``stereo_depth`` cloud is already WLS-filtered and outlier-gated,
but any *single* frame still carries quantisation noise and the odd stereo blunder
-- which is exactly why the raw per-frame cloud makes a slightly inaccurate map.
This node fuses many frames over time into a **probabilistic log-odds voxel grid**
expressed in the world frame and renders it as a true-colour (camera RGB) cloud:

  * every incoming depth point reinforces its voxel (+l_hit, clamped) and updates
    that voxel's colour from the camera image (running blend),
  * **free-space carving** -- every voxel the camera ray passes *through* on its way
    to a hit is evidence of empty space, so it is decremented (-l_miss). This is the
    single biggest cleanliness lever: a spurious "floating" voxel left by a stereo
    blunder or a transient VIO wobble is seen-through on the very next frame and
    carved away, instead of lingering as a garbage trail the drone never flew to,
  * the whole grid also decays slowly toward "unknown" on a timer (-decay) so a
    moved obstacle eventually fades,
  * only voxels above ``occ_thresh`` *that have enough occupied neighbours*
    (spatial-consistency filter) are published -- isolated specks are dropped.

Three payoffs:
  1. **Accuracy / outlier rejection** -- a real surface is hit frame after frame,
     reinforced, and crosses the occupancy threshold; a transient stereo blunder is
     seen once, never accumulates, is carved by the next ray, and decays away.
  2. **No garbage trails** -- ray carving + the neighbour-support filter remove the
     drifting/floating voxels that made the map look like paths to nowhere.
  3. **Dynamics** -- because occupancy decays and is carved, an obstacle that moves
     (or a stale reading) fades out, so the map tracks a changing scene.

Pose comes from VIO by default (``/argus/vio/odom``, nav_msgs/Odometry) -- i.e.
**no GPS / no ground-truth dependency** -- but a geometry_msgs/PoseStamped source
(e.g. ground truth for A/B testing) is also supported via ``pose_type:=pose``.

ORIENTATION (the bit that used to wreck the map): we take VIO *position* to place
the cloud, but by default we do NOT rotate by the live VIO *orientation*. In this
low-parallax corridor the VINS yaw/attitude is the noisy, init-lottery-prone part
of the estimate (the reactive_avoider deliberately steers in the body frame and
never trusts VIO orientation/z for exactly this reason). The mapper was the lone
component rotating the dense cloud by that wobbly quaternion -- so every attitude
glitch smeared whole walls into "paths the drone never flew to". The holonomic
drone is commanded with zero angular velocity throughout and holds itself level on
the rangefinder, so its true attitude stays at the spawn orientation (identity in
the VINS world frame, which initialises level + facing +X down the corridor) for
the entire flight. Hence ``orientation_mode=identity`` is both the robust and the
physically-correct model. Ground-truth poses (``pose_type:=pose``) ARE attitude-
reliable, so ``orientation_mode=auto`` uses the full quaternion in that case.

Subscribes:
  /argus/depth/points  sensor_msgs/PointCloud2   XYZ(+RGB), cam0_optical_frame
  <pose_topic>         nav_msgs/Odometry | geometry_msgs/PoseStamped (world)

Publishes (additive -- see CONTRACT.md sec 8):
  /argus/map/points    sensor_msgs/PointCloud2   fused occupied voxels (XYZ+RGB), world
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy,
                       qos_profile_sensor_data)

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2, PointField


def parse_xyzrgb(msg: PointCloud2):
    """Extract (xyz Nx3 float32, rgb Nx3 uint8|None) from a PointCloud2.

    Colour is the float32-packed ``rgb`` field that ``stereo_depth`` emits
    (little-endian bytes [b, g, r, 0]); returns ``None`` if absent.
    """
    off = {f.name: f.offset for f in msg.fields}
    n = msg.width * msg.height
    if not {'x', 'y', 'z'} <= off.keys() or n == 0:
        return np.empty((0, 3), np.float32), None
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
    xyz = np.empty((raw.shape[0], 3), np.float32)
    for i, ax in enumerate(('x', 'y', 'z')):
        xyz[:, i] = raw[:, off[ax]:off[ax] + 4].copy().view(np.float32).ravel()

    rgb = None
    if 'rgb' in off:
        o = off['rgb']
        rgb = np.empty((raw.shape[0], 3), np.uint8)
        rgb[:, 0] = raw[:, o + 2]  # r
        rgb[:, 1] = raw[:, o + 1]  # g
        rgb[:, 2] = raw[:, o + 0]  # b

    finite = np.isfinite(xyz).all(axis=1)
    xyz = xyz[finite]
    if rgb is not None:
        rgb = rgb[finite]
    return xyz, rgb


def quat_to_R(q) -> np.ndarray:
    """3x3 rotation matrix from a geometry_msgs quaternion (body->world)."""
    x, y, z, w = q.x, q.y, q.z, q.w
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    return np.array([
        [1 - s * (y * y + z * z), s * (x * y - z * w),     s * (x * z + y * w)],
        [s * (x * y + z * w),     1 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w),     s * (y * z + x * w),     1 - s * (x * x + y * y)],
    ], dtype=np.float64)


def make_xyzrgb_cloud(header, xyz, rgb) -> PointCloud2:
    """Build an XYZ+packed-RGB float32 PointCloud2 from (N,3) arrays.

    Always emits the ``rgb`` field so RViz's RGB8 transformer has colour; if no
    per-point colour is supplied the points fall back to mid-grey.
    """
    n = xyz.shape[0]
    buf = np.zeros((n, 4), dtype=np.float32)
    buf[:, 0:3] = xyz
    if rgb is not None and len(rgb) == n:
        r = rgb[:, 0].astype(np.uint32)
        g = rgb[:, 1].astype(np.uint32)
        b = rgb[:, 2].astype(np.uint32)
    else:
        r = g = b = np.full(n, 180, np.uint32)
    packed = (r << 16) | (g << 8) | b
    buf[:, 3] = packed.view(np.float32)

    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = n
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = 16 * n
    msg.is_dense = True
    msg.data = buf.tobytes()
    return msg


# 26-connected neighbour offsets (used by the spatial-consistency filter).
_NEIGHBORS = [(dx, dy, dz)
              for dx in (-1, 0, 1) for dy in (-1, 0, 1) for dz in (-1, 0, 1)
              if not (dx == 0 and dy == 0 and dz == 0)]


class OccupancyMapper(Node):
    def __init__(self):
        super().__init__('occupancy_mapper')

        self.declare_parameter('voxel_size', 0.15)        # m, map resolution
        self.declare_parameter('l_hit', 0.85)             # log-odds added per hit
        self.declare_parameter('l_miss', 0.4)             # log-odds removed per carve
        self.declare_parameter('l_max', 5.0)              # clamp (confidence ceiling)
        self.declare_parameter('l_min', -2.0)             # clamp / prune floor
        self.declare_parameter('occ_thresh', 1.6)         # publish voxels above this
        # Decay bleeds every voxel toward "unknown" on a timer so a voxel the camera
        # has looked away from eventually drops below occ_thresh and DISAPPEARS. That
        # is wrong for this demo: we want the fused terrain to PERSIST so the finished
        # map can be reviewed at the end of the flight. Default 0 = no fade; the map is
        # cleaned by ray-carving (geometry that is seen-through) + the neighbour filter,
        # not by blindly forgetting. Raise it only if you want a moved obstacle to fade.
        self.declare_parameter('decay', 0.0)              # log-odds removed per pub tick
        self.declare_parameter('publish_rate', 4.0)       # Hz, map publish + decay
        # Stereo gets unreliable fast with the 0.12 m baseline; only fuse the near,
        # trustworthy band. Far points are exactly the ones that smear into trails.
        self.declare_parameter('max_range', 6.0)          # m, ignore far (noisy) points
        self.declare_parameter('min_range', 0.4)          # m, ignore self/too-near
        # WORLD BOUNDS CLIP -- the map must never extend past the warehouse the drone
        # is actually in (corridor is 30 x 5 x 3 m: x[0,30] y[-2.5,2.5] z[0,3]). A VIO
        # position drift/jump otherwise places a whole cloud OUTSIDE the building, which
        # reads as "the map goes out of bounds". Any world point outside this AABB is
        # physically impossible terrain, so we drop it before it is ever fused.
        self.declare_parameter('clip_bounds', True)
        self.declare_parameter('bounds_min', [-0.5, -2.6, -0.1])  # world AABB lo (m)
        self.declare_parameter('bounds_max', [30.5, 2.6, 3.1])    # world AABB hi (m)
        # STATIONARY GATE -- when the drone is not moving there is nothing new to map,
        # so ingesting frames just lets stereo noise accrete fresh voxels forever ("it
        # keeps mapping when stationary"). Only fuse a frame once the drone has moved at
        # least `motion_thresh` metres from the last fused pose. Hovering jitter stays
        # under the threshold -> the map freezes; real flight clears it every few frames.
        self.declare_parameter('motion_gate', True)
        self.declare_parameter('motion_thresh', 0.05)     # m of travel before re-fusing
        self.declare_parameter('carve', True)             # free-space ray carving
        self.declare_parameter('carve_max_rays', 4000)    # subsample rays for carving
        # Spatial-consistency filter: only publish a voxel if it has >= this many
        # occupied 26-neighbours -> kills isolated floating specks (garbage).
        self.declare_parameter('min_neighbors', 3)
        self.declare_parameter('color_alpha', 0.4)        # per-voxel colour blend rate
        self.declare_parameter('max_voxels', 400000)      # hard cap on grid size
        # cam0 optical-frame mount offset in body FLU (from model.sdf).
        self.declare_parameter('cam_offset', [0.10, 0.06, 0.0])
        self.declare_parameter('cloud_topic', '/argus/depth/points')
        self.declare_parameter('map_topic', '/argus/map/points')
        # Trajectory ACTUALLY used to build the map -> RViz draws this instead of the
        # raw /argus/vio/path, so the displayed course always matches the map and never
        # shows the VINS estimate drifting out of the corridor.
        self.declare_parameter('path_topic', '/argus/map/path')
        self.declare_parameter('path_min_step', 0.05)     # m between stored path poses
        self.declare_parameter('map_frame', 'world')
        # GPS-free default: pose from VIO odometry. Use pose_type:=pose for a
        # geometry_msgs/PoseStamped source (e.g. ground truth, for A/B testing).
        self.declare_parameter('pose_topic', '/argus/vio/odom')
        self.declare_parameter('pose_type', 'odometry')   # 'odometry' | 'pose'
        # How to derive the body->world rotation used to place the cloud:
        #   'identity' -- ignore the (noisy VIO) quaternion, use I. Correct for the
        #                 level, never-yawing holonomic drone; kills attitude smear.
        #   'vio'      -- trust the pose's full quaternion (use for ground truth).
        #   'auto'     -- 'vio' when pose_type=='pose' (ground truth, attitude-OK),
        #                 else 'identity' (VIO odometry, attitude-unreliable). DEFAULT.
        self.declare_parameter('orientation_mode', 'auto')
        # Reject obviously-diverged VIO poses (a VINS blow-up to 1e5 m) so one bad
        # estimate can never scatter the whole map to infinity.
        self.declare_parameter('pose_sane_radius', 1000.0)   # m from origin

        self.voxel = float(self.get_parameter('voxel_size').value)
        self.pose = None  # (position xyz np, R body->world)
        self._last_ingest_pos = None  # world pos of the last fused frame (motion gate)

        # log-odds grid: (ix, iy, iz) voxel index -> [log_odds, r, g, b] (floats).
        self.grid: dict[tuple, list] = {}

        # RELIABLE + TRANSIENT_LOCAL: a reliable publisher serves both reliable and
        # best-effort subscribers, and transient-local hands the latest map to RViz
        # the moment it connects (the default sensor-data/best-effort profile was
        # dropping the map for reliable subscribers -> "incompatible QoS" warning).
        map_qos = QoSProfile(depth=1,
                             reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_map = self.create_publisher(
            PointCloud2, self.get_parameter('map_topic').value, map_qos)
        self.pub_path = self.create_publisher(
            Path, self.get_parameter('path_topic').value, map_qos)
        self._path = Path()          # accumulated trajectory used to build the map
        self._path_last = None       # last stored path position (for thinning)

        ptype = str(self.get_parameter('pose_type').value).lower()
        ptopic = self.get_parameter('pose_topic').value

        # Resolve orientation handling once (see the module docstring for the why).
        omode = str(self.get_parameter('orientation_mode').value).lower()
        if omode == 'auto':
            omode = 'vio' if ptype == 'pose' else 'identity'
        self._use_vio_rotation = (omode == 'vio')

        if ptype == 'pose':
            self.create_subscription(PoseStamped, ptopic, self._on_pose, 10)
        else:
            self.create_subscription(Odometry, ptopic, self._on_odom, 10)

        self.create_subscription(PointCloud2, self.get_parameter('cloud_topic').value,
                                 self._on_cloud, qos_profile_sensor_data)

        rate = float(self.get_parameter('publish_rate').value)
        self.timer = self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(
            'occupancy_mapper up: voxel=%.2fm carve=%s decay=%.2f bounds=%s '
            'motion_gate=%s(%.2fm) pose=%s(%s) rotation=%s -> %s'
            % (self.voxel, bool(self.get_parameter('carve').value),
               float(self.get_parameter('decay').value),
               bool(self.get_parameter('clip_bounds').value),
               bool(self.get_parameter('motion_gate').value),
               float(self.get_parameter('motion_thresh').value),
               ptopic, ptype,
               'vio-quat' if self._use_vio_rotation else 'identity(level)',
               self.get_parameter('map_topic').value))

    # -- pose --------------------------------------------------------------
    def _accept(self, p, q):
        """Store the pose unless it is non-finite or outside the world envelope
        (a diverged VIO estimate) -- a bad pose would smear the map to infinity."""
        r = float(self.get_parameter('pose_sane_radius').value)
        if not (np.isfinite([p.x, p.y, p.z]).all()
                and abs(p.x) < r and abs(p.y) < r and abs(p.z) < r):
            self.get_logger().warn('rejecting diverged pose (%.1f,%.1f,%.1f)'
                                    % (p.x, p.y, p.z), throttle_duration_sec=2.0)
            return
        # VIO position places the cloud; orientation only when it's trustworthy
        # (ground truth). For VIO odometry we hold the spawn attitude (identity) --
        # see the module docstring: trusting the wobbly VINS quaternion was what
        # smeared the map into garbage trails.
        R = quat_to_R(q) if self._use_vio_rotation else np.eye(3)
        self.pose = (np.array([p.x, p.y, p.z], np.float64), R)

        # Record the trajectory we are actually mapping from (thinned), so RViz can
        # draw the true course instead of the drifting raw VINS path.
        cur = self.pose[0]
        step = float(self.get_parameter('path_min_step').value)
        if self._path_last is None or np.linalg.norm(cur - self._path_last) >= step:
            ps = PoseStamped()
            ps.header.frame_id = self.get_parameter('map_frame').value
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = \
                float(p.x), float(p.y), float(p.z)
            ps.pose.orientation = q
            self._path.poses.append(ps)
            self._path_last = cur.copy()

    def _on_odom(self, msg: Odometry):
        self._accept(msg.pose.pose.position, msg.pose.pose.orientation)

    def _on_pose(self, msg: PoseStamped):
        self._accept(msg.pose.position, msg.pose.orientation)

    # -- ingest a depth cloud ---------------------------------------------
    def _on_cloud(self, msg: PointCloud2):
        if self.pose is None:
            return  # need a pose before we can place points in the world

        # Stationary gate: skip the frame unless the drone has travelled far enough
        # since the last fused frame. Stops noise from accreting while hovering.
        if bool(self.get_parameter('motion_gate').value):
            cur = self.pose[0]
            if self._last_ingest_pos is not None:
                if np.linalg.norm(cur - self._last_ingest_pos) < \
                        float(self.get_parameter('motion_thresh').value):
                    return
            self._last_ingest_pos = cur.copy()

        xyz, rgb = parse_xyzrgb(msg)
        if xyz.shape[0] == 0:
            return

        # Range gate in the camera (optical) frame: ||xyz|| is the line-of-sight
        # distance. Keep only the near, trustworthy band.
        rng = np.linalg.norm(xyz, axis=1)
        keep = (rng >= float(self.get_parameter('min_range').value)) & \
               (rng <= float(self.get_parameter('max_range').value))
        xyz = xyz[keep]
        rng = rng[keep]
        if rgb is not None:
            rgb = rgb[keep]
        if xyz.shape[0] == 0:
            return

        # optical (x right, y down, z fwd) -> body FLU (x fwd, y left, z up),
        # including the camera mount offset.
        ox, oy, oz = self.get_parameter('cam_offset').value
        body = np.empty_like(xyz, dtype=np.float64)
        body[:, 0] = xyz[:, 2] + ox
        body[:, 1] = -xyz[:, 0] + oy
        body[:, 2] = -xyz[:, 1] + oz

        pos, R = self.pose
        world = body @ R.T + pos                       # body FLU -> world
        cam_world = pos + R @ np.array([ox, oy, oz], np.float64)  # ray origin

        # Drop anything that landed outside the warehouse AABB (VIO drift garbage),
        # so the map can never grow past the building the drone is flying in.
        if bool(self.get_parameter('clip_bounds').value):
            lo = np.asarray(self.get_parameter('bounds_min').value, np.float64)
            hi = np.asarray(self.get_parameter('bounds_max').value, np.float64)
            inside = ((world >= lo) & (world <= hi)).all(axis=1)
            world = world[inside]
            rng = rng[inside]
            if rgb is not None:
                rgb = rgb[inside]
            if world.shape[0] == 0:
                return

        if rgb is None:
            rgb = np.full((world.shape[0], 3), 180, np.uint8)

        # 1) carve free space along each camera ray (before depositing hits).
        if bool(self.get_parameter('carve').value):
            self._carve(cam_world, world, rng)

        # 2) deposit hits: reinforce voxel occupancy + blend in camera colour.
        self._deposit(world, rgb)

    def _deposit(self, world, rgb):
        l_hit = float(self.get_parameter('l_hit').value)
        l_max = float(self.get_parameter('l_max').value)
        alpha = float(self.get_parameter('color_alpha').value)
        grid = self.grid

        idx = np.floor(world / self.voxel).astype(np.int64)
        keys_u, inv = np.unique(idx, axis=0, return_inverse=True)
        # mean camera colour of the points that fell in each voxel this frame.
        counts = np.bincount(inv, minlength=keys_u.shape[0]).astype(np.float64)
        csum = np.zeros((keys_u.shape[0], 3), np.float64)
        np.add.at(csum, inv, rgb.astype(np.float64))
        cmean = csum / counts[:, None]

        for row, col in zip(keys_u.tolist(), cmean):
            key = (row[0], row[1], row[2])
            cell = grid.get(key)
            if cell is None:
                grid[key] = [l_hit, col[0], col[1], col[2]]
            else:
                cell[0] = min(l_max, cell[0] + l_hit)
                cell[1] += alpha * (col[0] - cell[1])
                cell[2] += alpha * (col[1] - cell[2])
                cell[3] += alpha * (col[2] - cell[3])

    def _carve(self, origin, world, rng):
        """Decrement (toward free) every *existing* voxel a ray passes through,
        stopping one voxel short of the hit so we never erase the surface itself."""
        l_miss = float(self.get_parameter('l_miss').value)
        l_min = float(self.get_parameter('l_min').value)
        grid = self.grid
        if not grid:
            return

        # Subsample rays to bound per-frame cost.
        cap = int(self.get_parameter('carve_max_rays').value)
        if world.shape[0] > cap:
            sel = np.random.choice(world.shape[0], cap, replace=False)
            world, rng = world[sel], rng[sel]

        unit = (world - origin) / np.maximum(rng[:, None], 1e-6)
        max_steps = int(np.ceil(float(self.get_parameter('max_range').value) / self.voxel))
        # Half-voxel sampling guarantees we land in every voxel the ray crosses.
        dists = (np.arange(1, max_steps + 1) * (self.voxel * 0.5))  # (S,)

        # (N, S, 3) sample points; mask out samples at/after the surface voxel.
        samples = origin + unit[:, None, :] * dists[None, :, None]
        valid = dists[None, :] < (rng[:, None] - self.voxel)        # (N, S)
        pts = samples[valid]
        if pts.shape[0] == 0:
            return

        miss = np.unique(np.floor(pts / self.voxel).astype(np.int64), axis=0)
        for row in miss.tolist():
            key = (row[0], row[1], row[2])
            cell = grid.get(key)
            if cell is not None:
                cell[0] = max(l_min, cell[0] - l_miss)

    # -- decay + publish ---------------------------------------------------
    def _publish(self):
        # Trajectory first, so the true course shows from the very first pose even
        # before any voxel crosses the occupancy threshold.
        if self._path.poses:
            self._path.header.frame_id = self.get_parameter('map_frame').value
            self._path.header.stamp = self.get_clock().now().to_msg()
            self.pub_path.publish(self._path)

        if not self.grid:
            return
        decay = float(self.get_parameter('decay').value)
        l_min = float(self.get_parameter('l_min').value)
        occ = float(self.get_parameter('occ_thresh').value)

        keys = np.array(list(self.grid.keys()), dtype=np.int64).reshape(-1, 3)
        vals = np.array(list(self.grid.values()), dtype=np.float64).reshape(-1, 4)
        lo = vals[:, 0] - decay        # time decay toward unknown
        col = vals[:, 1:4]

        keep = lo > l_min
        keys, lo, col = keys[keep], lo[keep], col[keep]

        # Enforce a hard cap (keep the most-confident voxels) to bound memory.
        cap = int(self.get_parameter('max_voxels').value)
        if lo.shape[0] > cap:
            sel = np.argpartition(lo, -cap)[-cap:]
            keys, lo, col = keys[sel], lo[sel], col[sel]

        self.grid = {(k[0], k[1], k[2]): [l, c[0], c[1], c[2]]
                     for k, l, c in zip(keys.tolist(), lo.tolist(), col.tolist())}

        # Occupied subset, then spatial-consistency (neighbour-support) filter.
        occ_mask = lo >= occ
        okeys, ocol = keys[occ_mask], col[occ_mask]
        okeys, ocol = self._neighbor_filter(okeys, ocol)

        pts = ((okeys + 0.5) * self.voxel).astype(np.float32) if okeys.shape[0] \
            else np.empty((0, 3), np.float32)
        rgb = ocol.astype(np.uint8) if okeys.shape[0] else None

        hdr = Header()
        hdr.stamp = self.get_clock().now().to_msg()
        hdr.frame_id = self.get_parameter('map_frame').value
        self.pub_map.publish(make_xyzrgb_cloud(hdr, pts, rgb))

    def _neighbor_filter(self, keys, col):
        """Drop occupied voxels with fewer than ``min_neighbors`` occupied
        26-neighbours -- removes isolated specks that read as garbage."""
        need = int(self.get_parameter('min_neighbors').value)
        if need <= 0 or keys.shape[0] == 0:
            return keys, col
        occ_set = set(map(tuple, keys.tolist()))
        mask = np.empty(keys.shape[0], dtype=bool)
        for i, k in enumerate(keys.tolist()):
            c = 0
            for dx, dy, dz in _NEIGHBORS:
                if (k[0] + dx, k[1] + dy, k[2] + dz) in occ_set:
                    c += 1
                    if c >= need:
                        break
            mask[i] = c >= need
        return keys[mask], col[mask]


def main(argv=None):
    rclpy.init(args=argv)
    node = OccupancyMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
