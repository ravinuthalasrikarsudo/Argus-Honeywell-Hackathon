#!/usr/bin/env python3
"""ARGUS reactive_avoider -- reactive obstacle avoidance.

Closes the sense-and-avoid loop: instead of flying a *scripted*
``cmd_vel`` path (drive_drone / fly_shuttle), the drone now **senses obstacles
live and steers around them**. It fuses the dense, outlier-filtered stereo
obstacle cloud from ``stereo_depth`` (``/argus/depth/points``) with the onboard
3D LiDAR cloud (``/argus/lidar/points``) -- robust where stereo is weak (blank
walls) -- and takes its position reference from **VIO odometry** by default
(``/argus/vio/odom``), so it needs no GPS / ground truth. It then runs a
**potential-field planner**:

    v = goal_attraction  +  obstacle_repulsion  +  corridor_walls

The kinematic drone is holonomic (gz VelocityControl twist), so avoidance is
done by *strafing* in body Y/Z while cruising forward in X -- no yaw needed, so
the stereo pair keeps facing down the corridor. Gains are demo-tuned to reliably
clear the enriched warehouse slalom (see warehouse_corridor.sdf) while honoring
the project's 0.8 m/s flight envelope.

Frames: obstacle points arrive in ``cam0_optical_frame`` (optical: +x right,
+y down, +z forward). We rotate them into body FLU (forward/left/up) and offset
by the cam0 mount (+0.10, +0.06, 0). Goal attraction is computed in the world
ENU frame and rotated into body by the drone yaw. Output Twist is body FLU on
``/argus/cmd_vel`` -- the existing frozen ROS->gz input.

Publishes (additive -- see CONTRACT.md sec 8):
  /argus/cmd_vel      geometry_msgs/Twist   (existing frozen input topic)
  /argus/nav/status   std_msgs/String       human-readable state line
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import String


def parse_xyz(msg: PointCloud2) -> np.ndarray:
    """Extract an (N,3) float32 XYZ array from a PointCloud2 by field offsets."""
    off = {f.name: f.offset for f in msg.fields}
    if not {'x', 'y', 'z'} <= off.keys() or msg.width * msg.height == 0:
        return np.empty((0, 3), np.float32)
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
    out = np.empty((raw.shape[0], 3), np.float32)
    for i, ax in enumerate(('x', 'y', 'z')):
        out[:, i] = raw[:, off[ax]:off[ax] + 4].copy().view(np.float32).ravel()
    return out[np.isfinite(out).all(axis=1)]


def yaw_from_quat(q) -> float:
    """Z-yaw from a geometry_msgs quaternion."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class ReactiveAvoider(Node):
    def __init__(self):
        super().__init__('reactive_avoider')

        # --- mission / envelope ---------------------------------------------
        # GPS-free corridor traversal: cruise FORWARD (body +X, the kinematic drone
        # holds spawn yaw), hold altitude on the downward rangefinder, and avoid in
        # the body frame. We deliberately do NOT steer on absolute VIO pose -- VIO's
        # z is unreliable and a bad-pose -> bad-command feedback loop diverges the
        # estimator. Pose is used ONLY to measure forward distance for the stop.
        self.declare_parameter('goal_dist', 24.0)       # m forward before stopping
        self.declare_parameter('max_speed', 0.8)        # m/s, flight envelope (horizontal)
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('ramp', 1.5)             # s, accel/decel ramp
        # Acceleration limit on the published twist. The kinematic VelocityControl
        # applies a commanded velocity in ONE physics step, so any abrupt change in
        # cmd_vel shows up on the IMU as a huge acceleration impulse (a 0.5 m/s step
        # at 250 Hz reads as 125 m/s^2) which destabilises VIO. Rate-limiting the
        # command keeps the motion physical, the IMU clean, and the flight smooth.
        self.declare_parameter('max_accel', 0.6)        # m/s^2, horizontal+vertical slew
        # Reject obviously-diverged VIO poses (e.g. a VINS blow-up to 1e5 m) so a bad
        # estimate can never false-trigger GOAL_REACHED. Steering never uses absolute
        # pose, so a rejected pose only pauses goal-distance accounting.
        self.declare_parameter('pose_sane_radius', 1000.0)  # m from origin

        # --- altitude hold (downward rangefinder, NOT VIO z) ----------------
        self.declare_parameter('target_agl', 1.0)       # m above ground to hold
        self.declare_parameter('k_alt', 0.8)            # P-gain on altitude error
        self.declare_parameter('max_climb', 0.5)        # m/s vertical clamp

        # --- potential-field gains ------------------------------------------
        self.declare_parameter('k_att', 0.6)            # forward cruise = k_att*max_speed
        self.declare_parameter('k_rep', 0.45)
        self.declare_parameter('rep_sectors', 24)       # angular bins for density-free repulsion
        self.declare_parameter('influence_d0', 2.0)     # m, repulsion range
        self.declare_parameter('safety_radius', 0.6)    # m, hard "too close"
        self.declare_parameter('obstacle_z_band', 0.8)  # m, |dz| around drone counted
        self.declare_parameter('forward_min', 0.12)     # m/s creep so it never stalls
        self.declare_parameter('tangential_gain', 0.6)  # local-minimum escape

        # --- io -------------------------------------------------------------
        # GPS-free by default: pose from VIO odometry, NOT ground truth. Set
        # pose_type:=pose to consume a geometry_msgs/PoseStamped instead (e.g.
        # /argus/ground_truth/pose as an A/B baseline). Pose is optional -- without
        # it the drone still flies & avoids, it just won't auto-stop at goal_dist.
        self.declare_parameter('pose_topic', '/argus/vio/odom')
        self.declare_parameter('pose_type', 'odometry')   # 'odometry' | 'pose'
        self.declare_parameter('cloud_topic', '/argus/depth/points')
        self.declare_parameter('lidar_topic', '/argus/lidar/points')
        self.declare_parameter('use_lidar', True)
        self.declare_parameter('rangefinder_topic', '/argus/rangefinder')
        self.declare_parameter('use_rangefinder', True)
        self.declare_parameter('cmd_topic', '/argus/cmd_vel')
        # cam0 optical-frame mount offset in body FLU (from model.sdf).
        self.declare_parameter('cam_offset', [0.10, 0.06, 0.0])
        # lidar_link mount offset in body FLU (from model.sdf): on a +0.12 m mast.
        self.declare_parameter('lidar_offset', [0.0, 0.0, 0.12])

        self.pose = None
        self.agl = None                                 # m above ground (rangefinder)
        self._prev_xy = None                            # last pose sample (path integration)
        self._traveled_acc = 0.0                        # accumulated path length (jump-rejected)
        self.obs_stereo = np.empty((0, 3), np.float32)  # gated, body FLU
        self.obs_lidar = np.empty((0, 3), np.float32)   # gated, body FLU
        self.obs_body = np.empty((0, 3), np.float32)    # fused, body FLU
        self.min_obs = float('inf')
        self.reached = False
        self._t0 = None
        self._cmd = np.zeros(3)        # last published body velocity (accel-limited)
        self._last_ctrl = None         # wall/sim time of last control tick

        self.pub_cmd = self.create_publisher(
            Twist, self.get_parameter('cmd_topic').value, 10)
        self.pub_status = self.create_publisher(String, '/argus/nav/status', 10)

        ptype = str(self.get_parameter('pose_type').value).lower()
        ptopic = self.get_parameter('pose_topic').value
        if ptype == 'pose':
            self.create_subscription(PoseStamped, ptopic, self._on_pose, 10)
        else:
            self.create_subscription(Odometry, ptopic, self._on_odom, 10)

        self.create_subscription(PointCloud2, self.get_parameter('cloud_topic').value,
                                 self._on_cloud, qos_profile_sensor_data)
        self.use_lidar = bool(self.get_parameter('use_lidar').value)
        if self.use_lidar:
            self.create_subscription(PointCloud2, self.get_parameter('lidar_topic').value,
                                     self._on_lidar, qos_profile_sensor_data)
        self.use_rangefinder = bool(self.get_parameter('use_rangefinder').value)
        if self.use_rangefinder:
            self.create_subscription(LaserScan, self.get_parameter('rangefinder_topic').value,
                                     self._on_range, qos_profile_sensor_data)

        rate = float(self.get_parameter('control_rate').value)
        self.timer = self.create_timer(1.0 / rate, self._control)
        self.get_logger().info(
            'reactive_avoider up: goal_dist=%.1fm max_speed=%.2f target_agl=%.1fm '
            'pose=%s(%s) lidar=%s rangefinder=%s -> %s'
            % (self.get_parameter('goal_dist').value, self.get_parameter('max_speed').value,
               self.get_parameter('target_agl').value, ptopic, ptype, self.use_lidar,
               self.use_rangefinder, self.get_parameter('cmd_topic').value))

    # -- subscriptions --------------------------------------------------------
    def _sane_pose(self, pose):
        """True if a pose is finite and within the world envelope (not a VIO blow-up)."""
        p = pose.position
        r = float(self.get_parameter('pose_sane_radius').value)
        return (math.isfinite(p.x) and math.isfinite(p.y) and math.isfinite(p.z)
                and abs(p.x) < r and abs(p.y) < r and abs(p.z) < r)

    def _on_pose(self, msg: PoseStamped):
        if self._sane_pose(msg.pose):
            self.pose = msg.pose

    def _on_odom(self, msg: Odometry):
        if self._sane_pose(msg.pose.pose):
            self.pose = msg.pose.pose

    def _on_range(self, msg: LaserScan):
        # Single down-pointing beam: ranges[0] = height above ground (drone is
        # held level by the kinematic drive, so this is true vertical AGL).
        if msg.ranges and math.isfinite(msg.ranges[0]):
            self.agl = float(msg.ranges[0])

    def _on_cloud(self, msg: PointCloud2):
        xyz = parse_xyz(msg)
        if xyz.shape[0] == 0:
            self.obs_stereo = np.empty((0, 3), np.float32)
        else:
            # optical (x right, y down, z fwd) -> body FLU (x fwd, y left, z up)
            ox, oy, oz = self.get_parameter('cam_offset').value
            body = np.stack([xyz[:, 2] + ox, -xyz[:, 0] + oy, -xyz[:, 1] + oz], axis=1)
            self.obs_stereo = self._gate(body)
        self._fuse()

    def _on_lidar(self, msg: PointCloud2):
        xyz = parse_xyz(msg)
        if xyz.shape[0] == 0:
            self.obs_lidar = np.empty((0, 3), np.float32)
        else:
            # lidar_link is body-aligned FLU on a mast; add the mount offset only.
            lx, ly, lz = self.get_parameter('lidar_offset').value
            body = np.stack([xyz[:, 0] + lx, xyz[:, 1] + ly, xyz[:, 2] + lz], axis=1)
            self.obs_lidar = self._gate(body)
        self._fuse()

    def _gate(self, body: np.ndarray) -> np.ndarray:
        """Keep obstacles near flight altitude and not behind the drone."""
        zband = float(self.get_parameter('obstacle_z_band').value)
        keep = (np.abs(body[:, 2]) < zband) & (body[:, 0] > -0.3)
        return body[keep].astype(np.float32)

    def _fuse(self):
        """Merge the stereo and lidar obstacle sets and refresh the nearest range."""
        if self.obs_stereo.shape[0] or self.obs_lidar.shape[0]:
            self.obs_body = np.vstack([self.obs_stereo, self.obs_lidar])
            self.min_obs = float(np.min(np.hypot(self.obs_body[:, 0], self.obs_body[:, 1])))
        else:
            self.obs_body = np.empty((0, 3), np.float32)
            self.min_obs = float('inf')

    # -- control loop ---------------------------------------------------------
    def _control(self):
        goal_dist = float(self.get_parameter('goal_dist').value)
        traveled = self._traveled()        # None until a (sane) pose arrives
        if self.reached or (traveled is not None and traveled >= goal_dist):
            self.reached = True
            target = np.zeros(3)           # decelerate smoothly to hover at goal
            state, remaining = 'GOAL_REACHED', 0.0
        else:
            target = self._potential_field()   # body-frame forward + avoidance (x, y)
            target[2] = self._altitude_cmd()   # z from rangefinder, independent of x/y
            remaining = (goal_dist - traveled) if traveled is not None else float('nan')
            state = 'AVOIDING' if self.min_obs < self.get_parameter('influence_d0').value else 'CRUISE'

        v = self._accel_limit(target)      # rate-limit -> clean IMU, smooth flight
        t = Twist()
        t.linear.x, t.linear.y, t.linear.z = float(v[0]), float(v[1]), float(v[2])
        self.pub_cmd.publish(t)
        self._status(state, remaining)

    def _traveled(self):
        """Forward distance travelled, by DEAD RECKONING the commanded body-x
        velocity (integrated in _accel_limit). The drone is kinematic, so commanded
        velocity == actual velocity to within the physics step -- this is a far more
        reliable goal-distance estimate than absolute VIO position, which jumps on
        VINS startup / re-localisation and would otherwise stop the drone instantly.
        Steering still uses live sensors; only this stop trigger is dead-reckoned."""
        return self._traveled_acc

    def _altitude_cmd(self):
        """Vertical velocity from the downward rangefinder (GPS-free altitude hold).
        Returns 0 (hold) when no AGL reading is available -- never dives blind."""
        if self.agl is None:
            return 0.0
        target = float(self.get_parameter('target_agl').value)
        k = float(self.get_parameter('k_alt').value)
        max_climb = float(self.get_parameter('max_climb').value)
        return float(np.clip(k * (target - self.agl), -max_climb, max_climb))

    def _potential_field(self):
        max_v = float(self.get_parameter('max_speed').value)
        # --- attraction: cruise FORWARD in the body frame (no absolute pose) -
        # The kinematic drone holds spawn yaw (we never command angular vel), so
        # body +X stays aligned with the corridor heading throughout.
        att = np.array([max_v * float(self.get_parameter('k_att').value), 0.0, 0.0])

        # --- repulsion: horizontal, from the fused obstacle cloud (body FLU) -
        rep = np.zeros(3)
        d0 = float(self.get_parameter('influence_d0').value)
        k_rep = float(self.get_parameter('k_rep').value)
        tang = np.zeros(3)
        if self.obs_body.shape[0]:
            oh = self.obs_body[:, :2]                       # x,y in body
            d = np.hypot(oh[:, 0], oh[:, 1])
            near = (d < d0) & (d > 1e-3)
            if near.any():
                ohn, dn = oh[near], d[near]
                # DENSITY-INDEPENDENT repulsion: bin the near points into angular
                # sectors and keep only the nearest point per sector, so a textured
                # wall (thousands of stereo points) exerts the same force as a clean
                # one (a handful of lidar points). Summing every raw point made the
                # repulsion scale with point density and paralysed forward flight.
                nsec = int(self.get_parameter('rep_sectors').value)
                bear = np.arctan2(ohn[:, 1], ohn[:, 0])
                sec = np.clip(((bear + math.pi) / (2 * math.pi) * nsec).astype(int), 0, nsec - 1)
                r2 = np.zeros(2)
                for s in np.unique(sec):
                    m = sec == s
                    i = int(np.argmin(dn[m]))
                    ds = float(dn[m][i])
                    dirv = -ohn[m][i] / ds                  # push away from obstacle
                    r2 += dirv * (k_rep * (1.0 / ds - 1.0 / d0) / (ds * ds))
                rep[0], rep[1] = r2[0], r2[1]
                # cap repulsion so a close obstacle can't fling the drone
                rmax = 2.0 * max_v
                rn = np.linalg.norm(rep[:2])
                if rn > rmax:
                    rep[:2] *= rmax / rn
                # local-minimum escape: obstacle nearly dead-ahead -> slide to
                # the side that has more free space (use nearest range per side).
                safe = float(self.get_parameter('safety_radius').value)
                ahead = (ohn[:, 0] > 0) & (np.abs(ohn[:, 1]) < safe)
                if ahead.any():
                    left = ohn[ohn[:, 1] > 0]
                    right = ohn[ohn[:, 1] < 0]
                    left_clr = np.min(np.hypot(left[:, 0], left[:, 1])) if left.shape[0] else d0
                    right_clr = np.min(np.hypot(right[:, 0], right[:, 1])) if right.shape[0] else d0
                    side = 1.0 if left_clr > right_clr else -1.0   # toward more clearance
                    tang[1] = side * float(self.get_parameter('tangential_gain').value) * max_v

        v = att + rep + tang

        # Never fully stall while short of the goal: keep a forward creep unless
        # something is inside the hard safety radius dead ahead.
        fmin = float(self.get_parameter('forward_min').value)
        safe = float(self.get_parameter('safety_radius').value)
        blocked = self.min_obs < safe
        if blocked:
            v[0] = min(v[0], 0.0)          # do not push into a close obstacle
        elif v[0] < fmin:
            v[0] = fmin

        # clamp HORIZONTAL speed only (z is the independent altitude controller)
        sp = math.hypot(v[0], v[1])
        if sp > max_v:
            v[0] *= max_v / sp
            v[1] *= max_v / sp
        return v

    def _accel_limit(self, target):
        """Slew the published velocity toward `target` under a max-acceleration cap.
        Replaces the old open-loop ramp: it bounds dv/dt on start, stop AND every
        avoidance correction, so the kinematic body never steps its velocity (which
        the IMU would report as a divergence-inducing acceleration spike)."""
        target = np.asarray(target, dtype=float)
        a_max = float(self.get_parameter('max_accel').value)
        now = self.get_clock().now().nanoseconds / 1e9
        dt = (now - self._last_ctrl) if self._last_ctrl is not None else 1.0 / max(
            1e-3, float(self.get_parameter('control_rate').value))
        self._last_ctrl = now
        dt_eff = max(1e-3, min(dt, 0.5))           # guard against long stalls
        if a_max <= 0.0:
            self._cmd = target
        else:
            dv_max = a_max * dt_eff
            self._cmd = self._cmd + np.clip(target - self._cmd, -dv_max, dv_max)
        # Dead-reckon forward progress for the goal trigger (see _traveled).
        self._traveled_acc += max(0.0, float(self._cmd[0])) * dt_eff
        return self._cmd

    def _status(self, state, remaining):
        m = self.min_obs
        rem = ('%.2fm' % remaining) if math.isfinite(remaining) else 'n/a'
        agl = ('%.2fm' % self.agl) if self.agl is not None else 'n/a'
        msg = String()
        msg.data = ('state=%s remaining=%s agl=%s nearest_obstacle=%s obs_pts=%d'
                    % (state, rem, agl,
                       ('%.2fm' % m) if math.isfinite(m) else 'none',
                       self.obs_body.shape[0]))
        self.pub_status.publish(msg)
        self.get_logger().info(msg.data, throttle_duration_sec=1.0)


def main(argv=None):
    rclpy.init(args=argv)
    node = ReactiveAvoider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub_cmd.publish(Twist())  # stop on exit
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
