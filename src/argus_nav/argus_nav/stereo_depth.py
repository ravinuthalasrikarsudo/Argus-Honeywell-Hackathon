#!/usr/bin/env python3
"""ARGUS stereo_depth -- dense stereo perception.

Turns the stereo pair (``/argus/cam0/image_raw`` left +
``/argus/cam1/image_raw`` right, baseline 0.12 m) into a **dense** depth image
and an **outlier-filtered** 3D point cloud. Recovers a per-pixel disparity and
a metric cloud the planner can treat as a live obstacle field.

Pipeline (one stereo frame):

  1. Pair left/right images by timestamp (message_filters approx sync).
  2. (optional) decimate for speed -- SGBM on 1280x720 is expensive, and under
     the known WSLg iGPU render budget (see CONTRACT.md sec 7) RTF is already
     tight, so default decimation = 2 (-> 640x360).
  3. Semi-Global Block Matching (cv2.StereoSGBM) -> disparity (px).
  4. Reproject to metric XYZ in the LEFT optical frame using the contract
     intrinsics (fx=fy=640, cx=640, cy=360) and the stereo baseline recovered
     from the patched right CameraInfo (P[3] = -fx*baseline = -76.8).
  5. **Outlier removal**: range gate + statistical outlier removal (open3d if
     present, else a scipy-cKDTree statistical filter, else a pure-numpy
     voxel-density fallback) + voxel downsample to cap the cloud size.

Publishes (additive to the frozen contract -- see CONTRACT.md sec 8):
  /argus/depth/image   sensor_msgs/Image   32FC1 metric depth, cam0_optical_frame
  /argus/depth/points  sensor_msgs/PointCloud2  XYZ+RGB, cam0_optical_frame

Frame note: cam0 ("left") *optical* frame follows the optical convention
(+x right, +y down, +z forward-along-the-camera-boresight). The drone body FLU
relation is handled by the consumer (reactive_avoider).
"""

import struct

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import message_filters
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField

try:
    import cv2
except ImportError as exc:  # pragma: no cover - hard dependency
    raise SystemExit('argus_nav.stereo_depth requires OpenCV (python3-opencv): %s' % exc)


# ----------------------------------------------------------------------------
# Image <-> numpy (no cv_bridge dependency; the contract fixes the encodings).
# ----------------------------------------------------------------------------
def image_to_gray(msg: Image) -> np.ndarray:
    """Decode a sensor_msgs/Image to an 8-bit grayscale ndarray (H, W)."""
    h, w = msg.height, msg.width
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    enc = msg.encoding.lower()
    if enc in ('rgb8', 'bgr8'):
        # msg.step is the row stride in BYTES (= w*3 for 3-channel, plus any pad).
        img = buf.reshape(h, msg.step if msg.step else w * 3)[:, :w * 3].reshape(h, w, 3)
        # Luma; channel order does not matter for a grayscale conversion weight
        # symmetric enough for matching. Use Rec.601 with the correct order.
        if enc == 'rgb8':
            return (0.299 * img[..., 0] + 0.587 * img[..., 1] + 0.114 * img[..., 2]).astype(np.uint8)
        return (0.114 * img[..., 0] + 0.587 * img[..., 1] + 0.299 * img[..., 2]).astype(np.uint8)
    if enc in ('mono8', '8uc1'):
        return buf.reshape(h, msg.step if msg.step else w)[:, :w]
    raise ValueError('stereo_depth: unsupported image encoding %r' % msg.encoding)


def image_to_rgb(msg: Image) -> np.ndarray:
    """Decode to an (H, W, 3) uint8 RGB array (for coloring the cloud)."""
    h, w = msg.height, msg.width
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    enc = msg.encoding.lower()
    if enc == 'rgb8':
        return buf.reshape(h, w, 3)
    if enc == 'bgr8':
        return buf.reshape(h, w, 3)[..., ::-1]
    gray = image_to_gray(msg)
    return np.repeat(gray[..., None], 3, axis=2)


# ----------------------------------------------------------------------------
# Statistical outlier removal -- best backend available, graceful fallback.
# ----------------------------------------------------------------------------
def _sor_open3d(xyz, nb_neighbors, std_ratio, voxel):
    import open3d as o3d  # noqa: WPS433 (optional backend)
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(xyz.astype(np.float64))
    if voxel > 0.0:
        pc = pc.voxel_down_sample(voxel)
    pc, idx = pc.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
    return np.asarray(pc.points, dtype=np.float32), None  # color handled by caller via voxel? -> drop


def _sor_scipy(xyz, nb_neighbors, std_ratio):
    from scipy.spatial import cKDTree  # noqa: WPS433 (optional backend)
    tree = cKDTree(xyz)
    k = min(nb_neighbors + 1, len(xyz))
    dists, _ = tree.query(xyz, k=k)
    mean_d = dists[:, 1:].mean(axis=1)  # exclude self (distance 0)
    thresh = mean_d.mean() + std_ratio * mean_d.std()
    return mean_d <= thresh


def _voxel_density_mask(xyz, voxel, min_count):
    """Pure-numpy fallback: drop points whose voxel cell holds < min_count points."""
    keys = np.floor(xyz / voxel).astype(np.int64)
    # Hash the 3D voxel index to 1D, count, and keep dense cells.
    _, inv, counts = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
    return counts[inv] >= min_count


# ----------------------------------------------------------------------------
def make_pointcloud2(header, xyz, rgb):
    """Build a PointCloud2 (XYZ float32 + packed RGB float32) from Nx3 arrays."""
    n = xyz.shape[0]
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    buf = np.zeros((n, 4), dtype=np.float32)
    buf[:, 0:3] = xyz
    if rgb is not None and len(rgb) == n:
        r = rgb[:, 0].astype(np.uint32)
        g = rgb[:, 1].astype(np.uint32)
        b = rgb[:, 2].astype(np.uint32)
        packed = (r << 16) | (g << 8) | b
        buf[:, 3] = packed.view(np.float32)
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = n
    msg.fields = fields
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = 16 * n
    msg.is_dense = True
    msg.data = buf.tobytes()
    return msg


class StereoDepth(Node):
    def __init__(self):
        super().__init__('stereo_depth')

        # --- parameters -----------------------------------------------------
        self.declare_parameter('decimation', 2)          # downscale factor before SGBM
        self.declare_parameter('min_disparity', 1)
        self.declare_parameter('num_disparities', 96)    # multiple of 16
        self.declare_parameter('block_size', 7)
        self.declare_parameter('uniqueness_ratio', 10)
        self.declare_parameter('speckle_window_size', 100)
        self.declare_parameter('speckle_range', 2)
        self.declare_parameter('disp12_max_diff', 1)     # left-right consistency (px)
        self.declare_parameter('min_range', 0.3)         # m, drop too-near (drone body)
        self.declare_parameter('max_range', 12.0)        # m, drop far/unreliable
        # WLS disparity post-filter (cv2.ximgproc): edge-aware smoothing + hole fill
        # + a per-pixel confidence map. The single biggest depth-quality lever here.
        self.declare_parameter('use_wls', True)
        self.declare_parameter('wls_lambda', 8000.0)
        self.declare_parameter('wls_sigma', 1.5)
        self.declare_parameter('wls_conf_thresh', 0.5)   # 0..1, drop low-confidence px
        # Range-dependent depth-uncertainty gate. With the 0.12 m baseline, depth
        # noise grows as Z^2: sigma_Z = Z^2 * sigma_disp / (fx * baseline). We drop
        # any point whose modelled sigma_Z exceeds max_depth_std -> kills the noisy
        # far tail analytically (smarter than a flat max_range cut).
        self.declare_parameter('disp_sigma_px', 0.5)     # assumed disparity stddev (px)
        self.declare_parameter('max_depth_std', 0.5)     # m, max tolerated sigma_Z
        self.declare_parameter('voxel_size', 0.08)       # m, downsample + density cell
        self.declare_parameter('sor_neighbors', 12)
        self.declare_parameter('sor_std_ratio', 1.5)
        self.declare_parameter('sor_min_count', 3)       # numpy fallback density floor
        self.declare_parameter('max_points', 60000)      # hard cap on published cloud
        self.declare_parameter('fx', 640.0)              # contract fallback
        self.declare_parameter('cx', 640.0)
        self.declare_parameter('cy', 360.0)
        self.declare_parameter('baseline', 0.12)         # contract fallback
        self.declare_parameter('publish_color', True)

        gp = self.get_parameter
        self.dec = max(1, int(gp('decimation').value))
        nd = int(gp('num_disparities').value)
        nd = max(16, (nd // 16) * 16)
        bs = int(gp('block_size').value)
        self.matcher = cv2.StereoSGBM_create(
            minDisparity=int(gp('min_disparity').value),
            numDisparities=nd,
            blockSize=bs,
            P1=8 * 3 * bs ** 2,
            P2=32 * 3 * bs ** 2,
            disp12MaxDiff=int(gp('disp12_max_diff').value),
            uniquenessRatio=int(gp('uniqueness_ratio').value),
            speckleWindowSize=int(gp('speckle_window_size').value),
            speckleRange=int(gp('speckle_range').value),
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
        )

        # WLS needs a right-view matcher of the same config; gracefully degrade to
        # plain SGBM if the contrib module is missing in this OpenCV build.
        self.use_wls = bool(gp('use_wls').value) and hasattr(cv2, 'ximgproc')
        if self.use_wls:
            self.right_matcher = cv2.ximgproc.createRightMatcher(self.matcher)
            self.wls = cv2.ximgproc.createDisparityWLSFilter(self.matcher)
            self.wls.setLambda(float(gp('wls_lambda').value))
            self.wls.setSigmaColor(float(gp('wls_sigma').value))
        elif bool(gp('use_wls').value):
            self.get_logger().warn('use_wls set but cv2.ximgproc unavailable; '
                                   'falling back to plain SGBM disparity.')

        # Intrinsics/baseline: seeded from the contract, refined from CameraInfo.
        self.fx = float(gp('fx').value)
        self.cx = float(gp('cx').value)
        self.cy = float(gp('cy').value)
        self.baseline = float(gp('baseline').value)
        self._have_info = False

        # --- pubs/subs ------------------------------------------------------
        self.pub_depth = self.create_publisher(Image, '/argus/depth/image', qos_profile_sensor_data)
        self.pub_cloud = self.create_publisher(PointCloud2, '/argus/depth/points', qos_profile_sensor_data)

        # CameraInfo: latch fx/cx/cy from cam0, baseline from cam1.P[3].
        self.create_subscription(CameraInfo, '/argus/cam0/camera_info',
                                 self._cam0_info, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/argus/cam1/camera_info',
                                 self._cam1_info, qos_profile_sensor_data)

        sub_l = message_filters.Subscriber(self, Image, '/argus/cam0/image_raw',
                                           qos_profile=qos_profile_sensor_data)
        sub_r = message_filters.Subscriber(self, Image, '/argus/cam1/image_raw',
                                           qos_profile=qos_profile_sensor_data)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [sub_l, sub_r], queue_size=5, slop=0.02)
        self.sync.registerCallback(self._on_stereo)

        self._sor_backend = self._pick_backend()
        self.get_logger().info(
            'stereo_depth up: decimation=%d num_disp=%d WLS=%s SOR=%s -> /argus/depth/{image,points}'
            % (self.dec, nd, self.use_wls, self._sor_backend))

    # -- CameraInfo latches ---------------------------------------------------
    def _cam0_info(self, msg: CameraInfo):
        if msg.k[0] > 0:
            self.fx = float(msg.k[0])
            self.cx = float(msg.k[2])
            self.cy = float(msg.k[5])
            self._have_info = True

    def _cam1_info(self, msg: CameraInfo):
        # P[3] = -fx * baseline (deviation #3). Recover baseline if present.
        if msg.p[0] > 0 and msg.p[3] != 0.0:
            self.baseline = abs(float(msg.p[3]) / float(msg.p[0]))

    def _pick_backend(self):
        # Prefer scipy statistical outlier removal: it preserves per-point color
        # and is present in this image. open3d's path voxel-samples before SOR and
        # loses the color correspondence, so it is only a last resort if scipy is
        # absent; the pure-numpy voxel-density filter is the final fallback.
        try:
            import scipy  # noqa: F401
            return 'scipy'
        except ImportError:
            pass
        try:
            import open3d  # noqa: F401
            return 'open3d'
        except ImportError:
            return 'numpy'

    # -- main stereo callback -------------------------------------------------
    def _on_stereo(self, left: Image, right: Image):
        try:
            gl = image_to_gray(left)
            gr = image_to_gray(right)
        except ValueError as exc:
            self.get_logger().warn(str(exc), throttle_duration_sec=5.0)
            return

        fx, cx, cy = self.fx, self.cx, self.cy
        if self.dec > 1:
            gl = gl[::self.dec, ::self.dec]
            gr = gr[::self.dec, ::self.dec]
            fx = fx / self.dec
            cx = cx / self.dec
            cy = cy / self.dec

        # Disparity is fixed-point (1/16 px) signed 16-bit in OpenCV. With WLS we
        # also matche the right view and let the edge-aware filter fuse them; its
        # confidence map then gates unreliable (occluded / low-texture) pixels.
        disp_l = self.matcher.compute(gl, gr)
        if self.use_wls:
            disp_r = self.right_matcher.compute(gr, gl)
            disp_fp = self.wls.filter(disp_l, gl, disparity_map_right=disp_r)
            conf = self.wls.getConfidenceMap()  # float32 in [0, 255]
        else:
            disp_fp = disp_l
            conf = None
        disp = disp_fp.astype(np.float32) / 16.0
        h, w = disp.shape

        valid = disp > float(self.get_parameter('min_disparity').value)
        if conf is not None:
            valid &= conf >= 255.0 * float(self.get_parameter('wls_conf_thresh').value)
        # Metric depth, left optical frame: Z forward, X right, Y down.
        with np.errstate(divide='ignore', invalid='ignore'):
            z = (fx * self.baseline) / disp
        min_r = float(self.get_parameter('min_range').value)
        max_r = float(self.get_parameter('max_range').value)
        valid &= np.isfinite(z) & (z > min_r) & (z < max_r)

        # Range-dependent uncertainty gate: sigma_Z = Z^2 * sigma_disp/(fx*b).
        # (fx here is already decimation-scaled, matching the disparity grid.)
        sigma_d = float(self.get_parameter('disp_sigma_px').value)
        max_std = float(self.get_parameter('max_depth_std').value)
        if max_std > 0.0:
            denom = fx * self.baseline
            with np.errstate(divide='ignore', invalid='ignore'):
                sigma_z = (z * z) * sigma_d / denom if denom > 0 else np.full_like(z, np.inf)
            valid &= sigma_z <= max_std

        # --- publish a dense depth image (NaN where invalid) ----------------
        depth = np.where(valid, z, np.nan).astype(np.float32)
        self.pub_depth.publish(self._depth_msg(depth, left.header))

        if not valid.any():
            return

        us, vs = np.meshgrid(np.arange(w), np.arange(h))
        zv = z[valid]
        xv = (us[valid] - cx) * zv / fx
        yv = (vs[valid] - cy) * zv / fx
        xyz = np.stack([xv, yv, zv], axis=1).astype(np.float32)

        rgb = None
        if bool(self.get_parameter('publish_color').value):
            color = image_to_rgb(left)
            if self.dec > 1:
                color = color[::self.dec, ::self.dec]
            rgb = color[valid].astype(np.uint8)

        xyz, rgb = self._filter_outliers(xyz, rgb)
        if xyz.shape[0] == 0:
            return

        cap = int(self.get_parameter('max_points').value)
        if xyz.shape[0] > cap:
            sel = np.random.choice(xyz.shape[0], cap, replace=False)
            xyz = xyz[sel]
            rgb = rgb[sel] if rgb is not None else None

        hdr = left.header
        hdr.frame_id = 'cam0_optical_frame'
        self.pub_cloud.publish(make_pointcloud2(hdr, xyz, rgb))

    # -- outlier removal dispatch --------------------------------------------
    def _filter_outliers(self, xyz, rgb):
        voxel = float(self.get_parameter('voxel_size').value)
        nb = int(self.get_parameter('sor_neighbors').value)
        std_ratio = float(self.get_parameter('sor_std_ratio').value)

        if self._sor_backend == 'scipy' and xyz.shape[0] > nb + 1:
            mask = _sor_scipy(xyz, nb, std_ratio)
            xyz, rgb = xyz[mask], (rgb[mask] if rgb is not None else None)
            xyz, rgb = self._voxel_downsample(xyz, rgb, voxel)
            return xyz, rgb

        if self._sor_backend == 'open3d':
            # open3d path also downsamples; colors dropped for simplicity/speed.
            try:
                xyz2, _ = _sor_open3d(xyz, nb, std_ratio, voxel)
                return xyz2, None
            except Exception as exc:  # noqa: BLE001 - fall back on any o3d hiccup
                self.get_logger().warn('open3d SOR failed (%s); using numpy fallback' % exc,
                                       throttle_duration_sec=10.0)
                self._sor_backend = 'numpy'

        # numpy fallback: voxel density filter (also downsamples).
        if voxel > 0.0 and xyz.shape[0] > 0:
            mask = _voxel_density_mask(xyz, voxel,
                                       int(self.get_parameter('sor_min_count').value))
            xyz, rgb = xyz[mask], (rgb[mask] if rgb is not None else None)
            xyz, rgb = self._voxel_downsample(xyz, rgb, voxel)
        return xyz, rgb

    @staticmethod
    def _voxel_downsample(xyz, rgb, voxel):
        """Keep one representative point per voxel (first occurrence)."""
        if voxel <= 0.0 or xyz.shape[0] == 0:
            return xyz, rgb
        keys = np.floor(xyz / voxel).astype(np.int64)
        _, idx = np.unique(keys, axis=0, return_index=True)
        return xyz[idx], (rgb[idx] if rgb is not None else None)

    def _depth_msg(self, depth, src_header):
        msg = Image()
        msg.header = src_header
        msg.header.frame_id = 'cam0_optical_frame'
        msg.height, msg.width = depth.shape
        msg.encoding = '32FC1'
        msg.is_bigendian = 0
        msg.step = msg.width * 4
        msg.data = depth.tobytes()
        return msg


def main(argv=None):
    rclpy.init(args=argv)
    node = StereoDepth()
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
