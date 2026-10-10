"""Bounded scan cache that never relies on cross-topic delivery order."""

from collections import OrderedDict
from dataclasses import dataclass
import json
import time


@dataclass(frozen=True)
class Scan:
    stamp_ns: int
    depth_stamp_ns: int
    rgb: object
    depth: object
    info: object
    mask: object
    detections: list


def parse_detections(payload):
    """Accept a stamped legacy list or a stamped envelope, never an unpaired list."""
    data = json.loads(payload)
    if isinstance(data, dict):
        detections = data.get('detections')
        stamp = int(data.get('source_stamp_ns', data.get('scan_stamp_ns', 0)))
    elif isinstance(data, list):
        detections = data
        stamps = {int(det.get('source_stamp_ns', det.get('scan_stamp_ns', 0)))
                  for det in detections if isinstance(det, dict)}
        stamp = stamps.pop() if len(stamps) == 1 else 0
    else:
        raise ValueError('Detections must be a list or stamped envelope')
    if not isinstance(detections, list) or not detections or not all(
            isinstance(det, dict) for det in detections):
        raise ValueError('Detections must contain a nonempty list of objects')
    if stamp <= 0:
        raise ValueError('Detections need consistent nonzero source_stamp_ns metadata')
    for det in detections:
        item_stamp = det.get('source_stamp_ns', det.get('scan_stamp_ns', stamp))
        if int(item_stamp) != stamp:
            raise ValueError('Detection metadata disagrees with envelope timestamp')
    return stamp, detections


class ScanCache:
    def __init__(self, slop_s=0.05, max_size=8, timeout_s=120.0):
        self.slop_ns = int(slop_s * 1e9)
        self.max_size = int(max_size)
        self.timeout_s = float(timeout_s)
        if self.slop_ns < 0 or self.max_size < 1 or self.timeout_s <= 0:
            raise ValueError('Invalid synchronization limits')
        self._buffers = {key: OrderedDict() for key in
                         ('rgb', 'depth', 'info', 'mask', 'detections')}
        self._completed = OrderedDict()

    def add(self, kind, stamp_ns, value):
        if stamp_ns <= 0:
            raise ValueError('Scan messages require a nonzero source timestamp')
        buffer = self._buffers[kind]
        buffer[stamp_ns] = (time.monotonic(), value)
        buffer.move_to_end(stamp_ns)
        while len(buffer) > self.max_size:
            buffer.popitem(last=False)

    def expire(self):
        now = time.monotonic()
        expired_masks = []
        for kind, buffer in self._buffers.items():
            for stamp, (arrival, _) in list(buffer.items()):
                if now - arrival > self.timeout_s:
                    del buffer[stamp]
                    if kind == 'mask' and stamp not in self._completed:
                        expired_masks.append(stamp)
        return expired_masks

    def _nearest(self, kind, stamp):
        buffer = self._buffers[kind]
        if not buffer:
            return None
        candidate = min(buffer, key=lambda key: abs(key - stamp))
        return candidate if abs(candidate - stamp) <= self.slop_ns else None

    def ready(self):
        for stamp, (_, mask) in list(self._buffers['mask'].items()):
            if stamp in self._completed:
                continue
            if stamp not in self._buffers['rgb'] or stamp not in self._buffers['detections']:
                continue
            depth_stamp = self._nearest('depth', stamp)
            if depth_stamp is None:
                continue
            info_stamp = self._nearest('info', depth_stamp)
            if info_stamp is None:
                continue
            return Scan(stamp, depth_stamp,
                        self._buffers['rgb'][stamp][1],
                        self._buffers['depth'][depth_stamp][1],
                        self._buffers['info'][info_stamp][1], mask,
                        self._buffers['detections'][stamp][1])
        return None

    def mark_complete(self, stamp_ns):
        self._completed[stamp_ns] = True
        while len(self._completed) > self.max_size * 2:
            self._completed.popitem(last=False)
