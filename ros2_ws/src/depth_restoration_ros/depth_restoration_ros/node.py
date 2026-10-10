"""Restore a captured EE RGB-D scan once its matching SAM mask arrives."""

from copy import deepcopy
import json
import queue
import threading
import time

from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from .client import MODEL_PORTS, RestorationError, RestoreClient, depth_to_mm, merge_depth, target_mask
from .synchronization import ScanCache, parse_detections

LATCHED = QoSProfile(depth=8, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
# Depth two: a late subscriber sees the latest start and end line, not a history of earlier runs.
BRAIN_STATUS = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE,
                          durability=DurabilityPolicy.TRANSIENT_LOCAL)


def stamp_ns(msg):
    return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)


class DepthRestorationNode(Node):
    def __init__(self):
        super().__init__('depth_restoration_node')
        defaults = {
            'model': 'swindrnet', 'server_host': '127.0.0.1',
            'swindrnet_url': '', 'remake_url': '', 'frame_url': '',
            'http_timeout_s': 30.0, 'pair_slop_s': 0.05, 'input_timeout_s': 120.0,
            'cache_size': 8, 'preserve_background': True,
            'source_image_topic': '/qwen/source_image',
            'source_depth_topic': '/qwen/source_depth',
            'source_camera_info_topic': '/qwen/source_camera_info',
            'source_mask_topic': '/sam/mask_image',
            'source_detections_topic': '/qwen/labeled_detections',
            'output_depth_topic': '/depth_restoration/depth_image',
            'output_image_topic': '/depth_restoration/source_image',
            'output_camera_info_topic': '/depth_restoration/camera_info',
            'output_mask_topic': '/depth_restoration/mask_image',
            'output_detections_topic': '/depth_restoration/labeled_detections',
            'status_topic': '/depth_restoration/status',
            'slow_brain_status_topic': '/slow_brain/status',
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        param = lambda name: self.get_parameter(name).value
        self._model = str(param('model')).strip().lower()
        if self._model not in (*MODEL_PORTS, 'raw'):
            raise ValueError(f'Unknown model {self._model!r}; choose {[*MODEL_PORTS, "raw"]}')
        self._client = (None if self._model == 'raw' else RestoreClient(
            self._model, str(param('server_host')), str(param(self._model + '_url')),
            float(param('http_timeout_s'))))
        self._preserve_background = bool(param('preserve_background'))
        self._cache = ScanCache(float(param('pair_slop_s')), int(param('cache_size')),
                                float(param('input_timeout_s')))
        self._bridge = CvBridge()
        self._busy = False
        self._results = queue.Queue(maxsize=1)
        self._worker = None
        self._closing = threading.Event()
        self._input_subscriptions = []
        for kind, msg_type, parameter in (
                ('rgb', Image, 'source_image_topic'),
                ('depth', Image, 'source_depth_topic'),
                ('info', CameraInfo, 'source_camera_info_topic'),
                ('mask', Image, 'source_mask_topic')):
            self._input_subscriptions.append(self.create_subscription(
                msg_type, str(param(parameter)),
                lambda msg, kind=kind: self._input(kind, msg), LATCHED))
        self._input_subscriptions.append(self.create_subscription(
            String, str(param('source_detections_topic')), self._detections, LATCHED))
        self._pubs = {}
        for kind, msg_type, parameter in (
                ('depth', Image, 'output_depth_topic'),
                ('rgb', Image, 'output_image_topic'),
                ('info', CameraInfo, 'output_camera_info_topic'),
                ('mask', Image, 'output_mask_topic'),
                ('detections', String, 'output_detections_topic')):
            self._pubs[kind] = self.create_publisher(msg_type, str(param(parameter)), LATCHED)
        self._status_pub = self.create_publisher(String, str(param('status_topic')), LATCHED)
        self._brain_pub = self.create_publisher(
            String, str(param('slow_brain_status_topic')), BRAIN_STATUS)
        self._timer = self.create_timer(0.05, self._poll)
        endpoint = self._client.url if self._client else 'explicit raw-depth mode'
        self.get_logger().info(f'Depth restoration ready: {self._model}, {endpoint}')
        self._status('idle')

    def _status(self, state, **details):
        self._status_pub.publish(String(data=json.dumps({
            'state': state, 'model': self._model, **details,
        })))

    def _brain_status(self, state, inference_time_s=None, reason=''):
        """One line on the shared slow-brain status topic; called only by the restore worker.

        Diagnostic only, like qwen_bridge's: a failed publish must not fail a restoration.
        """
        payload = {'stage': 'depth_restoration', 'state': state, 'model': self._model,
                   'port': self._client.port, 'inference_time_s': inference_time_s}
        if reason:
            payload['reason'] = reason
        try:
            self._brain_pub.publish(String(data=json.dumps(payload)))
        except Exception as exc:
            self.get_logger().warning(f'slow_brain status not published: {exc}')

    def _infer(self, rgb, raw_mm, binary):
        """Run the model once and announce its start and its end (ok or failed), once each."""
        self._brain_status('running')
        started = time.monotonic()
        try:
            restored = self._client.restore(rgb, raw_mm, binary)
        except Exception as exc:
            self._brain_status('failed', round(time.monotonic() - started, 3), str(exc)[:200])
            raise
        self._brain_status('ok', round(time.monotonic() - started, 3))
        return restored

    def _input(self, kind, msg):
        try:
            self._cache.add(kind, stamp_ns(msg), msg)
            self._schedule()
        except ValueError as exc:
            self.get_logger().error(str(exc))
            self._status('error', error=str(exc))

    def _detections(self, msg):
        try:
            stamp, detections = parse_detections(msg.data)
            self._cache.add('detections', stamp, detections)
            self._schedule()
        except (ValueError, TypeError) as exc:
            self.get_logger().error(f'Invalid detection metadata: {exc}')
            self._status('error', error=str(exc))

    def _schedule(self):
        if self._busy or self._closing.is_set():
            return
        scan = self._cache.ready()
        if scan is None:
            return
        self._busy = True
        # A failed scan is not retried forever on the same retained mask. A new
        # command/replan captures and publishes a fresh timestamped scan.
        self._cache.mark_complete(scan.stamp_ns)
        self._status('running', source_stamp_ns=scan.stamp_ns,
                     depth_stamp_ns=scan.depth_stamp_ns)
        self._worker = threading.Thread(target=self._restore, args=(scan,), daemon=True)
        self._worker.start()

    def _restore(self, scan):
        started = time.monotonic()
        try:
            rgb = self._bridge.imgmsg_to_cv2(scan.rgb, desired_encoding='rgb8')
            raw = self._bridge.imgmsg_to_cv2(scan.depth, desired_encoding='passthrough')
            raw_mm = depth_to_mm(raw, scan.depth.encoding)
            labels = self._bridge.imgmsg_to_cv2(scan.mask, desired_encoding='passthrough')
            shape = raw_mm.shape
            if rgb.shape != (*shape, 3) or labels.shape != shape:
                raise RestorationError('RGB, aligned depth, and SAM mask dimensions differ')
            if (scan.info.height, scan.info.width) != shape:
                raise RestorationError('CameraInfo dimensions differ from aligned depth')
            frames = {msg.header.frame_id for msg in
                      (scan.rgb, scan.depth, scan.info, scan.mask)}
            if len(frames) != 1 or not scan.depth.header.frame_id:
                raise RestorationError('RGB/depth/CameraInfo/mask require the same optical frame_id')
            if (len(scan.info.k) != 9 or not np.all(np.isfinite(scan.info.k))
                    or not scan.info.k[0] > 0 or not scan.info.k[4] > 0):
                raise RestorationError('CameraInfo has invalid intrinsics')
            binary = target_mask(labels, scan.detections)
            restored = raw_mm if self._client is None else self._infer(rgb, raw_mm, binary)
            depth_m = merge_depth(raw_mm, restored, binary, self._preserve_background)
            if not np.any(depth_m[binary > 0] > 0):
                raise RestorationError('Scan has no valid TARGET depth')
            output = self._bridge.cv2_to_imgmsg(depth_m, encoding='32FC1')
            output.header = deepcopy(scan.depth.header)
            source = deepcopy(scan.rgb)
            source.header = deepcopy(scan.depth.header)
            info = deepcopy(scan.info)
            info.header = deepcopy(scan.depth.header)
            mask = deepcopy(scan.mask)
            mask.header = deepcopy(scan.depth.header)
            detections = deepcopy(scan.detections)
            for det in detections:
                det['source_stamp_ns'] = scan.depth_stamp_ns
                det['source_frame_id'] = scan.depth.header.frame_id
            envelope = String(data=json.dumps({
                'source_stamp_ns': scan.depth_stamp_ns,
                'frame_id': scan.depth.header.frame_id, 'detections': detections,
            }))
            self._results.put((scan, {
                'depth': output, 'rgb': source, 'info': info,
                'mask': mask, 'detections': envelope,
            }, None, time.monotonic() - started))
        except Exception as exc:  # one bad service response must not terminate ROS
            self._results.put((scan, None, str(exc), time.monotonic() - started))

    def _poll(self):
        for stamp in self._cache.expire():
            self.get_logger().error(f'Timed out waiting for paired inputs: scan {stamp}')
            self._status('error', source_stamp_ns=stamp, error='Input pairing timeout')
        try:
            scan, outputs, error, elapsed = self._results.get_nowait()
        except queue.Empty:
            return
        if error:
            self.get_logger().error(f'Restoration failed after {elapsed:.2f}s: {error}')
            self._status('error', source_stamp_ns=scan.stamp_ns,
                         depth_stamp_ns=scan.depth_stamp_ns, latency_s=elapsed, error=error)
        else:
            # Mask is a convenient completion signal, but consumers must still
            # pair all messages by timestamp: DDS has no cross-topic ordering.
            for kind in ('rgb', 'info', 'depth', 'detections', 'mask'):
                self._pubs[kind].publish(outputs[kind])
            self._status('ready', source_stamp_ns=scan.depth_stamp_ns, latency_s=elapsed)
            self.get_logger().info(
                f'Published {self._model} scan {scan.depth_stamp_ns} in {elapsed:.2f}s')
        self._busy = False
        self._schedule()

    def destroy_node(self):
        self._closing.set()
        if self._worker is not None:
            self._worker.join(timeout=1.0)
        if self._client is not None:
            self._client.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DepthRestorationNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
