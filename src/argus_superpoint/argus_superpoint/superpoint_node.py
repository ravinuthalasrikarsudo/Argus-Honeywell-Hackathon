#!/usr/bin/env python3
"""ARGUS superpoint_node.py

Standalone learned-feature front-end. Runs the SuperPoint ONNX extractor on the
RTX 4050 (ONNX Runtime CUDA EP), subscribes to a camera image and publishes:

  * /argus/vio/keypoints      sensor_msgs/PointCloud2  -- detected keypoints as
                              (u, v, score) points in the cam0 optical frame.
  * /argus/superpoint/overlay sensor_msgs/Image        -- the input frame with
                              keypoints drawn (score-coloured), for RViz / rqt
                              visual confirmation that learned features survive
                              the low-texture Zone-B walls where KLT starves.

Validates the extractor at >= 15 Hz with 1024 max keypoints on the dGPU.

Run with the SuperPoint venv interpreter and ROS sourced (see run_superpoint.sh):
    source /opt/ros/humble/setup.bash && source ~/argus/install/setup.bash
    ~/.venvs/argus-sp/bin/python -m argus_superpoint.superpoint_node --ros-args ...
"""

import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header

try:
    import onnxruntime as ort
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        '[superpoint] onnxruntime not importable. Run with the SuperPoint venv:\n'
        '  bash ~/argus/scripts/run_superpoint.sh\n'
        f'  (import error: {exc})')


def _imgmsg_to_gray(msg: Image) -> np.ndarray:
    """Convert a sensor_msgs/Image to a HxW uint8 grayscale array (no cv_bridge
    dependency -- decode the buffer directly, robust to encoding)."""
    h, w = msg.height, msg.width
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    enc = msg.encoding.lower()
    if enc in ('mono8', '8uc1'):
        return buf.reshape(h, w)
    if enc in ('rgb8', 'bgr8'):
        img = buf.reshape(h, w, 3)
        # luminosity grayscale (channel order does not matter for luma weights here)
        r, g, b = img[..., 0], img[..., 1], img[..., 2]
        if enc == 'bgr8':
            r, b = b, r
        return (0.299 * r + 0.587 * g + 0.114 * b).astype(np.uint8)
    if enc in ('rgba8', 'bgra8'):
        img = buf.reshape(h, w, 4)[..., :3]
        return (0.299 * img[..., 0] + 0.587 * img[..., 1]
                + 0.114 * img[..., 2]).astype(np.uint8)
    # fallback: assume single channel padded to step
    return buf[: h * w].reshape(h, w)


class SuperPointNode(Node):
    def __init__(self):
        super().__init__('argus_superpoint')

        self.declare_parameter('model_path',
                               '/home/vittal/argus/models/superpoint/superpoint_1024.onnx')
        self.declare_parameter('image_topic', '/argus/cam0/image_raw')
        self.declare_parameter('keypoints_topic', '/argus/vio/keypoints')
        self.declare_parameter('overlay_topic', '/argus/superpoint/overlay')
        self.declare_parameter('infer_width', 0)    # 0 = native; else resize width
        self.declare_parameter('infer_height', 0)   # 0 = native; else resize height
        self.declare_parameter('score_threshold', 0.0005)
        self.declare_parameter('publish_overlay', True)
        self.declare_parameter('force_cpu', False)

        gp = self.get_parameter
        self.model_path = gp('model_path').value
        self.infer_w = int(gp('infer_width').value)
        self.infer_h = int(gp('infer_height').value)
        self.score_thr = float(gp('score_threshold').value)
        self.publish_overlay = bool(gp('publish_overlay').value)
        force_cpu = bool(gp('force_cpu').value)

        # ---- ONNX Runtime session (CUDA EP, CPU fallback) ----
        # ORT 1.21+ ships CUDA/cuDNN under the venv's nvidia/* wheels; preload_dlls
        # loads them so the CUDA EP initialises (otherwise it silently falls back to
        # CPU -> ~1 Hz instead of >15 Hz).
        if hasattr(ort, 'preload_dlls'):
            try:
                ort.preload_dlls()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f'ort.preload_dlls() failed: {exc}')
        avail = ort.get_available_providers()
        providers = ['CPUExecutionProvider'] if force_cpu else (
            ['CUDAExecutionProvider', 'CPUExecutionProvider']
            if 'CUDAExecutionProvider' in avail else ['CPUExecutionProvider'])
        so = ort.SessionOptions()
        so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        self.sess = ort.InferenceSession(self.model_path, sess_options=so, providers=providers)
        self.in_name = self.sess.get_inputs()[0].name
        self.out_names = [o.name for o in self.sess.get_outputs()]
        self.using = self.sess.get_providers()[0]
        self.get_logger().info(
            f'SuperPoint ONNX loaded: {self.model_path}\n'
            f'  provider={self.using}  in={self.in_name} {self.sess.get_inputs()[0].shape}\n'
            f'  outputs={[(o.name, o.shape) for o in self.sess.get_outputs()]}')

        try:
            import cv2  # noqa: F401
            self._cv2 = cv2
        except ImportError:
            self._cv2 = None
            self.publish_overlay = False
            self.get_logger().warn('cv2 unavailable -> overlay disabled')

        # RELIABLE image sub: the ARGUS gz bridge and `ros2 bag play` both publish
        # RELIABLE; a BEST_EFFORT sub receives NOTHING from them over Cyclone
        # (deviation #7 -- same trap that stalled VINS). Byte-match it.
        img_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST)
        self.pub_kp = self.create_publisher(PointCloud2, gp('keypoints_topic').value, 10)
        # Overlay is viz-only: publish BEST_EFFORT (sensor-data QoS) so rqt_image_view
        # and RViz Image displays -- which subscribe BEST_EFFORT by default -- actually
        # RECEIVE it. A RELIABLE pub + BEST_EFFORT sub "matches" but delivers nothing
        # over Cyclone on loopback (multicast disabled); see contract deviation #7.
        self.pub_ov = (self.create_publisher(Image, gp('overlay_topic').value,
                                             qos_profile_sensor_data)
                       if self.publish_overlay else None)
        self.sub = self.create_subscription(
            Image, gp('image_topic').value, self._on_image, img_qos)

        # rate bookkeeping
        self._n = 0
        self._t_win = time.monotonic()
        self._infer_ms = 0.0
        self._kp_last = 0
        self.create_timer(2.0, self._report)
        self.get_logger().info('SuperPoint node up; waiting for images...')

    # ---- inference ----
    def _infer(self, gray: np.ndarray):
        """Return (keypoints Nx2 float [u,v] at native res, scores N)."""
        h0, w0 = gray.shape
        img = gray
        sx = sy = 1.0
        if self.infer_w > 0 and self.infer_h > 0 and self._cv2 is not None:
            img = self._cv2.resize(gray, (self.infer_w, self.infer_h))
            sx = w0 / float(self.infer_w)
            sy = h0 / float(self.infer_h)
        inp = (img.astype(np.float32) / 255.0)[None, None]  # [1,1,H,W]
        outs = self.sess.run(self.out_names, {self.in_name: inp})

        kpts, scores = None, None
        for arr in outs:
            a = np.asarray(arr)
            sq = np.squeeze(a)
            if sq.ndim == 2 and sq.shape[-1] == 2:
                kpts = sq.astype(np.float32)
            elif sq.ndim == 2 and sq.shape[0] == 2 and sq.shape[1] != 2:
                kpts = sq.T.astype(np.float32)
            elif sq.ndim == 1:
                scores = sq.astype(np.float32)
        if kpts is None:
            return np.empty((0, 2), np.float32), np.empty((0,), np.float32)
        if scores is None or len(scores) != len(kpts):
            scores = np.ones((len(kpts),), np.float32)
        if sx != 1.0 or sy != 1.0:
            kpts = kpts * np.array([sx, sy], np.float32)
        if self.score_thr > 0:
            m = scores >= self.score_thr
            kpts, scores = kpts[m], scores[m]
        return kpts, scores

    def _on_image(self, msg: Image):
        gray = _imgmsg_to_gray(msg)
        t0 = time.monotonic()
        kpts, scores = self._infer(gray)
        self._infer_ms += (time.monotonic() - t0) * 1e3
        self._n += 1
        self._kp_last = len(kpts)

        self.pub_kp.publish(self._cloud(msg.header, kpts, scores))
        if self.pub_ov is not None:
            self.pub_ov.publish(self._overlay(msg, gray, kpts, scores))

    # ---- message builders ----
    def _cloud(self, header: Header, kpts, scores) -> PointCloud2:
        n = len(kpts)
        data = np.zeros((n, 3), np.float32)
        if n:
            data[:, 0] = kpts[:, 0]
            data[:, 1] = kpts[:, 1]
            data[:, 2] = scores
        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = n
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * n
        msg.is_dense = True
        msg.data = data.tobytes()
        return msg

    def _overlay(self, msg: Image, gray, kpts, scores) -> Image:
        cv2 = self._cv2
        vis = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        if len(scores):
            smax = float(scores.max()) or 1.0
            for (u, v), s in zip(kpts, scores):
                c = s / smax
                col = (int(255 * (1 - c)), int(60 + 195 * c), int(255 * c))  # blue->green/red
                cv2.circle(vis, (int(round(u)), int(round(v))), 2, col, -1)
        cv2.putText(vis, f'SuperPoint kpts={len(kpts)} [{self.using}]', (10, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        out = Image()
        out.header = msg.header
        out.height, out.width = vis.shape[:2]
        out.encoding = 'bgr8'
        out.is_bigendian = 0
        out.step = 3 * out.width
        out.data = vis.tobytes()
        return out

    def _report(self):
        now = time.monotonic()
        dt = now - self._t_win
        if self._n > 0 and dt > 0:
            hz = self._n / dt
            ms = self._infer_ms / self._n
            self.get_logger().info(
                f'rate={hz:.1f} Hz  infer={ms:.1f} ms  kpts={self._kp_last}  ({self.using})')
        self._n = 0
        self._infer_ms = 0.0
        self._t_win = now


def main(argv=None):
    rclpy.init(args=argv)
    node = SuperPointNode()
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
