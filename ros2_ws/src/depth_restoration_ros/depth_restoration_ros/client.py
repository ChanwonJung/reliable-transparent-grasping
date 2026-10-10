"""HTTP contract shared by the three services, independent of ROS and training code."""

import base64
import http.client
import json
import socket
import uuid
from urllib.parse import urlsplit

import cv2
import numpy as np

MODEL_PORTS = {'swindrnet': 30000, 'remake': 31000, 'frame': 32000}


class RestorationError(RuntimeError):
    """Invalid service response or input; no implicit raw-depth fallback."""


def depth_to_mm(depth, encoding):
    """Convert ROS depth to uint16 millimetres; invalid values become zero."""
    if encoding == '16UC1' and depth.dtype == np.uint16:
        return np.ascontiguousarray(depth)
    if encoding == '32FC1' and np.issubdtype(depth.dtype, np.floating):
        valid = np.isfinite(depth) & (depth > 0)
        safe = np.where(valid, depth, 0)
        return np.clip(np.rint(safe * 1000.0), 0, 65535).astype(np.uint16)
    raise RestorationError(f'Unsupported depth encoding/dtype: {encoding}/{depth.dtype}')


def target_mask(label_map, detections):
    """SAM label i+1 corresponds to detection i in the original Qwen list."""
    if label_map.ndim != 2 or not np.issubdtype(label_map.dtype, np.integer):
        raise RestorationError('SAM label map must be a single-channel integer image')
    ids = [i + 1 for i, det in enumerate(detections)
           if str(det.get('category', '')).upper() == 'TARGET']
    if not ids:
        raise RestorationError('No TARGET detection in scan')
    binary = np.where(np.isin(label_map, ids), 255, 0).astype(np.uint8)
    if not np.any(binary):
        raise RestorationError('TARGET mask is empty')
    return binary


def _png(array):
    ok, encoded = cv2.imencode('.png', array, [cv2.IMWRITE_PNG_COMPRESSION, 1])
    if not ok:
        raise RestorationError('Failed to encode input PNG')
    return encoded.tobytes()


def _multipart(fields):
    boundary = uuid.uuid4().hex
    parts = []
    for name, payload in fields.items():
        header = (f'--{boundary}\r\nContent-Disposition: form-data; '
                  f'name="{name}"; filename="{name}.png"\r\n'
                  'Content-Type: image/png\r\n\r\n')
        parts.append(header.encode() + payload + b'\r\n')
    parts.append(f'--{boundary}--\r\n'.encode())
    return b''.join(parts), f'multipart/form-data; boundary={boundary}'


class RestoreClient:
    """One bounded HTTP request at a time, with a reusable connection."""

    def __init__(self, model='swindrnet', host='127.0.0.1', url='', timeout_s=30.0):
        model = model.strip().lower()
        if model not in MODEL_PORTS:
            raise ValueError(f'Unknown restoration model: {model}')
        self.model = model
        self.url = url or f'http://{host}:{MODEL_PORTS[model]}/restore_depth'
        parsed = urlsplit(self.url)
        if parsed.scheme not in ('http', 'https') or not parsed.hostname:
            raise ValueError('Restoration URL must be an HTTP(S) endpoint')
        self._parsed = parsed
        self._timeout_s = float(timeout_s)
        if self._timeout_s <= 0:
            raise ValueError('HTTP timeout must be positive')
        self._conn = None

    @property
    def port(self):
        """TCP port of the endpoint this client posts to."""
        return self._parsed.port or (443 if self._parsed.scheme == 'https' else 80)

    def close(self):
        if self._conn is not None:
            self._conn.close()
            self._conn = None

    def _post(self, body, content_type):
        for attempt in range(2):
            fresh = self._conn is None
            if fresh:
                cls = (http.client.HTTPSConnection if self._parsed.scheme == 'https'
                       else http.client.HTTPConnection)
                self._conn = cls(self._parsed.hostname, self._parsed.port,
                                 timeout=self._timeout_s)
            try:
                path = self._parsed.path or '/restore_depth'
                if self._parsed.query:
                    path += '?' + self._parsed.query
                self._conn.request('POST', path, body, {
                    'Content-Type': content_type, 'Content-Length': str(len(body)),
                })
                response = self._conn.getresponse()
                return response.status, response.read()
            except (http.client.HTTPException, OSError) as exc:
                self.close()
                if fresh or attempt == 1 or isinstance(exc, socket.timeout):
                    raise RestorationError(f'Restoration request failed: {exc}') from exc

    def restore(self, rgb, depth_mm, mask):
        """Return full-frame uint16 mm depth after strict validation."""
        if depth_mm.ndim != 2 or depth_mm.dtype != np.uint16:
            raise RestorationError('Input depth must be uint16 millimetres')
        if rgb.shape != (*depth_mm.shape, 3) or rgb.dtype != np.uint8:
            raise RestorationError('RGB must be aligned uint8 HxWx3')
        if mask.shape != depth_mm.shape or mask.dtype != np.uint8:
            raise RestorationError('Mask must be aligned uint8 HxW')
        if not np.any(mask):
            raise RestorationError('TARGET mask is empty')
        body, content_type = _multipart({
            'rgb': _png(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)),
            'depth': _png(depth_mm), 'mask': _png(mask),
        })
        status, response = self._post(body, content_type)
        try:
            reply = json.loads(response)
        except (ValueError, UnicodeDecodeError) as exc:
            raise RestorationError(f'HTTP {status}: response is not JSON') from exc
        if status != 200:
            detail = reply.get('detail', reply) if isinstance(reply, dict) else reply
            raise RestorationError(f'HTTP {status}: {str(detail)[:300]}')
        if not isinstance(reply, dict) or not isinstance(reply.get('depth_png_base64'), str):
            raise RestorationError('Response lacks depth_png_base64')
        try:
            payload = base64.b64decode(reply['depth_png_base64'], validate=True)
            restored = cv2.imdecode(np.frombuffer(payload, np.uint8), cv2.IMREAD_UNCHANGED)
        except (ValueError, cv2.error) as exc:
            raise RestorationError('Response depth is not a valid PNG') from exc
        if restored is None or restored.dtype != np.uint16 or restored.ndim != 2:
            raise RestorationError('Response depth must be a uint16 single-channel PNG')
        if restored.shape != depth_mm.shape:
            raise RestorationError(f'Response shape {restored.shape} != input {depth_mm.shape}')
        if not np.any(restored[mask > 0] > 0):
            raise RestorationError('Response has no valid TARGET depth')
        return restored


def merge_depth(raw_mm, restored_mm, mask, preserve_background=True):
    """Produce 32FC1 metres, optionally retaining measured non-target pixels."""
    depth_mm = np.where(mask > 0, restored_mm, raw_mm) if preserve_background else restored_mm
    return depth_mm.astype(np.float32) * np.float32(0.001)
