#!/usr/bin/env python3
"""Live side-by-side of the raw RealSense depth and a Seraph depth-restoration model's
output. Not a project ROS package -- a standalone script, like the other files in demos/.

Each frame is cropped to --crop (default 640x360, centred), POSTed to one of the three
HTTP services on Seraph and shown next to the raw depth it came from. Both panels use the
same colour scale; black = no depth.

  python3 demos/restore_depth_viewer.py --model 32000     # 30000 | 31000 | 32000
  python3 demos/restore_depth_viewer.py --probe           # what does each service say, how fast is it?

  30000  swindrnet     rgb + depth only
  31000  remake        } also need to know WHERE the transparent object is: SAM2.1 runs on the
  32000  FRAME        } server from your click prompts (below)

Needs the Seraph tunnels (environments/ros2/seraph_env.sh) and a camera publishing colour +
aligned depth (realsense2_camera with align_depth.enable:=true).

Click prompts (31000 / 32000), in either depth panel:
  left click   = this is the object     right click = this is background
  c            = clear all points       u           = undo the last point
The points are sent with EVERY frame and the server re-runs SAM2.1 on each one -- that is
what costs latency; --probe measures it. Until the first click nothing is sent (the
service answers 400 without a mask or prompt). The services do not return the SAM mask,
only its score, so the mask itself can't be drawn.

How it stays close to real time -- the model, not the plumbing, sets the pace:
  * Only the NEWEST frame is ever sent. While a request is in flight, camera frames
    overwrite each other in a one-slot mailbox; nothing queues up, so the restored image is
    never older than one round trip plus the wait for the next frame.
  * One request in flight, over one kept-alive HTTP connection.
  * Display, camera callbacks and the network call run on separate threads, so a slow or
    dead server never freezes the window.

The left panel shows the raw depth of the exact frame that produced the right panel, so the
two stay comparable while the scene moves. --live-left shows the live camera instead.

Confidence row (C and 1 - C), second row, on by default (--no-confidence hides it): C is the
confidence in the RAW depth. The viewer checks every reply for an extra image
`<name>_png_base64` with a confidence-like name (or --conf-key NAME); 8/16-bit images are
scaled to 0..1 by their full range.
  30000  sends `confidence`: shown as C.
  32000  sends `pred_weight`, the weight on the PREDICTED depth: shown inverted, C = 1 - it.
  31000  sends none: the two panels stay empty and the reason is printed once to the terminal.
An image named `reliability` is deliberately NOT read as C (it is a different quantity); point
--conf-key at it to view it the same way.

Scoring (space / g) -- needs a ground-truth depth, which a live camera does not give you:
  g       capture the ground truth: put an opaque stand-in where the transparent object will
          be, keep the camera FIXED, press g. It is the per-pixel median of 15 raw frames.
          (Or pass --gt depth.png: 16-bit millimetres, exactly the --crop size.)
  space   with the real transparent object in the same place, capture the next
          --capture-frames restored frames. Raw and restored depth are scored against the
          ground truth (RMSE, MAE, AbsRel, delta<1.05/1.10/1.25) and saved with the frames as
          metrics.json + report.html under data/captures/. Without a ground truth the frames
          are still saved, just not scored. restore_eval.py defines the metrics.
"""
import argparse
import base64
import http.client
import json
import socket
import sys
import textwrap
import threading
import time
import uuid
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))    # restore_eval.py sits next to this file
import restore_eval  # noqa: E402

REPO = Path(__file__).resolve().parent.parent
MODEL_PORTS = (30000, 31000, 32000)
MODEL_NAMES = {30000: "SwinDRNet", 31000: "ReMake", 32000: "FRAME"}
COLOR_TOPIC = "/camera/camera/color/image_raw"
DEPTH_TOPIC = "/camera/camera/aligned_depth_to_color/image_raw"
CONF_NAMES = ("confidence", "conf", "confidence_map", "conf_map", "c")   # reply images read as C
# Reply images holding the weight on the PREDICTED depth, so C = 1 - image. Inferred from :32000's
# `pred_weight`, which is ~0.98 at raw-depth holes and ~0.17 elsewhere; the server code isn't in this repo.
WEIGHT_NAMES = ("pred_weight",)

HEADER, FOOTER, DIVIDER = 46, 40, 2
WHITE, GREY, RED, GREEN = (255, 255, 255), (170, 170, 170), (70, 70, 255), (80, 220, 80)
AMBER = (0, 190, 255)


# ── ROS side ────────────────────────────────────────────────────────────────

@dataclass(frozen=True)
class Frame:
    seq: int
    t_rx: float          # time.monotonic() when the pair reached this process
    rgb: np.ndarray      # (H, W, 3) uint8, RGB order
    depth: np.ndarray    # (H, W) uint16 millimetres, 0 = invalid (what the services take)


class RateMeter:
    """Smoothed events-per-second; reads 0 once the events stop."""

    def __init__(self, alpha=0.15):
        self._hz, self._alpha, self._last = 0.0, alpha, None

    def tick(self):
        now = time.monotonic()
        if self._last is None or now - self._last > 1.5:
            self._hz = 0.0          # first event, or restarting after a stall
        elif now > self._last:
            inst = 1.0 / (now - self._last)
            self._hz = inst if self._hz == 0.0 else self._hz + self._alpha * (inst - self._hz)
        self._last = now

    @property
    def hz(self):
        return self._hz if self._last and time.monotonic() - self._last < 1.5 else 0.0


def _rgb(msg, y0, y1, x0, x1):
    if msg.encoding not in ("rgb8", "bgr8"):
        raise ValueError(f"colour encoding {msg.encoding!r}: need rgb8 or bgr8")
    buf = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
    img = buf[y0:y1, x0 * 3:x1 * 3].reshape(y1 - y0, x1 - x0, 3)
    return np.ascontiguousarray(img[:, :, ::-1] if msg.encoding == "bgr8" else img)


def _depth_mm(msg, y0, y1, x0, x1):
    if msg.encoding == "16UC1":     # RealSense: millimetres, already what the services want
        buf = np.frombuffer(msg.data, np.uint16).reshape(msg.height, msg.step // 2)
        return np.ascontiguousarray(buf[y0:y1, x0:x1])
    if msg.encoding == "32FC1":     # metres
        buf = np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.step // 4)
        mm = np.nan_to_num(buf[y0:y1, x0:x1]) * 1000.0
        return np.clip(np.round(mm), 0, 65535).astype(np.uint16)
    raise ValueError(f"depth encoding {msg.encoding!r}: need 16UC1 or 32FC1")


class FrameGrabber:
    """Subscribes colour + aligned depth, crops, and keeps only the newest pair."""

    def __init__(self, color_topic, depth_topic, crop, origin, slop):
        import rclpy
        from message_filters import ApproximateTimeSynchronizer, Subscriber
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image

        self._rclpy = rclpy
        self._crop, self._origin = crop, origin
        self._cond = threading.Condition()
        self._frame = None
        self._seq = 0
        self.error = None
        self.rate = RateMeter()

        rclpy.init()
        self._node = rclpy.create_node("restore_depth_viewer")
        sync = ApproximateTimeSynchronizer(
            [Subscriber(self._node, Image, color_topic, qos_profile=qos_profile_sensor_data),
             Subscriber(self._node, Image, depth_topic, qos_profile=qos_profile_sensor_data)],
            queue_size=5, slop=slop)
        sync.registerCallback(self._on_pair)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()

    def _spin(self):
        from rclpy.executors import ExternalShutdownException
        try:
            self._executor.spin()
        except ExternalShutdownException:   # rclpy.shutdown() while spinning: normal on exit
            pass

    def _on_pair(self, color, depth):
        try:
            frame = self._build(color, depth)
        except ValueError as e:
            self.error = str(e)
            return
        self.error = None
        self.rate.tick()
        with self._cond:
            self._frame = frame
            self._seq = frame.seq
            self._cond.notify_all()

    def _build(self, color, depth):
        h, w = color.height, color.width
        if (depth.height, depth.width) != (h, w):
            raise ValueError(f"depth is {depth.width}x{depth.height}, colour {w}x{h}: "
                             "use the aligned_depth_to_color topic")
        cw, ch = self._crop
        if cw > w or ch > h:
            raise ValueError(f"frame {w}x{h} is smaller than the {cw}x{ch} crop")
        x0, y0 = self._origin if self._origin else ((w - cw) // 2, (h - ch) // 2)
        if not (0 <= x0 <= w - cw and 0 <= y0 <= h - ch):
            raise ValueError(f"crop origin ({x0},{y0}) puts {cw}x{ch} outside the {w}x{h} frame")
        return Frame(self._seq + 1, time.monotonic(),
                     _rgb(color, y0, y0 + ch, x0, x0 + cw),
                     _depth_mm(depth, y0, y0 + ch, x0, x0 + cw))

    def latest(self):
        with self._cond:
            return self._frame

    def wait_newer(self, seq, timeout):
        """The newest frame if it is newer than `seq`, else None after `timeout` s."""
        with self._cond:
            self._cond.wait_for(lambda: self._seq > seq, timeout)
            return self._frame if self._seq > seq else None

    def close(self):
        self._executor.shutdown()
        self._spin_thread.join(timeout=2)
        self._node.destroy_node()
        if self._rclpy.ok():
            self._rclpy.shutdown()


# ── Server side ─────────────────────────────────────────────────────────────

def _png(a):
    return cv2.imencode(".png", a, [cv2.IMWRITE_PNG_COMPRESSION, 1])[1].tobytes()   # fast, lossless


def _multipart(fields):
    """fields: name -> (filename or None, bytes, content type or None) -> (body, header value)."""
    boundary = uuid.uuid4().hex
    parts = []
    for name, (fname, data, ctype) in fields.items():
        head = f'--{boundary}\r\nContent-Disposition: form-data; name="{name}"'
        head += f'; filename="{fname}"' if fname else ""
        head += f"\r\nContent-Type: {ctype}" if ctype else ""
        parts.append(head.encode() + b"\r\n\r\n" + data + b"\r\n")
    parts.append(f"--{boundary}--\r\n".encode())
    return b"".join(parts), f"multipart/form-data; boundary={boundary}"


class RestoreClient:
    """HTTP client for a Seraph depth-restoration service. Keeps the connection open."""

    def __init__(self, host, port, timeout_s):
        self.host, self.port, self.timeout_s = host, port, timeout_s
        self._conn = None

    def _request(self, method, path, body=None, headers=None):
        for attempt in (1, 2):
            fresh = self._conn is None
            if fresh:
                self._conn = http.client.HTTPConnection(self.host, self.port, timeout=self.timeout_s)
            try:
                self._conn.request(method, path, body=body, headers=headers or {})
                resp = self._conn.getresponse()
                return resp.status, resp.read()
            except (http.client.HTTPException, OSError) as e:    # incl. timeouts and resets
                self.close()
                # A kept-alive connection the server already closed: retry once on a fresh one.
                if fresh or attempt == 2 or isinstance(e, socket.timeout):
                    raise

    def close(self):
        if self._conn:
            self._conn.close()
            self._conn = None

    def _get_json(self, path):
        status, data = self._request("GET", path)
        if status != 200:
            raise RuntimeError(f"GET {path}: HTTP {status}")
        return json.loads(data)

    def capabilities(self):
        """(/health dict, whether POST /restore_depth takes SAM2 prompts)."""
        health = self._get_json("/health")
        spec = self._get_json("/openapi.json")
        fields = {}
        for schema in spec.get("components", {}).get("schemas", {}).values():
            fields.update(schema.get("properties", {}))
        return health, "points" in fields

    def restore(self, rgb, depth_mm, points=None, mask=None):
        """POST one frame. `points` = [(x, y, label)] SAM2 prompts, `mask` = uint8 image.

        Returns (images, info): every `<name>_png_base64` of the reply decoded to an array
        (always has "depth", uint16 mm), and the rest of the reply's JSON fields.
        """
        fields = {"rgb": ("rgb.png", _png(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)), "image/png"),
                  "depth": ("depth.png", _png(depth_mm), "image/png")}
        if mask is not None:
            fields["mask"] = ("mask.png", _png(mask), "image/png")
        if points:
            fields["points"] = (None, json.dumps([[int(x), int(y)] for x, y, _ in points]).encode(), None)
            fields["labels"] = (None, json.dumps([int(l) for _, _, l in points]).encode(), None)
        body, ctype = _multipart(fields)
        status, data = self._request("POST", "/restore_depth", body,
                                     {"Content-Type": ctype, "Content-Length": str(len(body))})
        try:
            reply = json.loads(data)
        except ValueError:
            raise RuntimeError(f"HTTP {status}: reply is not JSON ({data[:80]!r})")
        if status != 200:
            raise RuntimeError(f"HTTP {status}: {reply.get('detail', reply) if isinstance(reply, dict) else reply}")
        images, info = {}, {}
        for k, v in reply.items():
            if k.endswith("_png_base64") and isinstance(v, str):
                img = cv2.imdecode(np.frombuffer(base64.b64decode(v), np.uint8), cv2.IMREAD_UNCHANGED)
                if img is None:
                    raise RuntimeError(f"reply[{k!r}] is not a decodable PNG")
                images[k[:-len("_png_base64")]] = img
            else:
                info[k] = v
        if "depth" not in images:
            raise RuntimeError(f"reply has no depth_png_base64 (keys: {', '.join(reply)})")
        if images["depth"].dtype != np.uint16 or images["depth"].ndim != 2:
            raise RuntimeError(f"reply depth is {images['depth'].dtype} {images['depth'].shape}, "
                               "expected a 16-bit single-channel PNG")
        return images, info


def describe_error(e, port, timeout_s):
    if isinstance(e, socket.timeout):
        return f"no answer within {timeout_s:g} s -- is the service up and the tunnel to :{port} open?"
    if isinstance(e, ConnectionRefusedError):
        return f"connection refused -- is the tunnel to :{port} open?"
    if isinstance(e, (http.client.RemoteDisconnected, ConnectionResetError)):
        return f"connection closed from the Seraph side -- is the service on :{port} running?"
    return f"{type(e).__name__}: {e}"


class Prompts:
    """SAM2 click prompts in crop pixel coordinates: (x, y, label), 1 = object, 0 = background."""

    def __init__(self):
        self._lock = threading.Lock()
        self._pts = []
        self.version = 0

    def add(self, x, y, label):
        with self._lock:
            self._pts.append((int(x), int(y), int(label)))
            self.version += 1

    def undo(self):
        with self._lock:
            if self._pts:
                self._pts.pop()
                self.version += 1

    def clear(self):
        with self._lock:
            if self._pts:
                self._pts = []
                self.version += 1

    def snapshot(self):
        with self._lock:
            return list(self._pts)


@dataclass(frozen=True)
class Result:
    seq: int
    frame: Frame
    depth: np.ndarray    # restored, uint16 mm, same shape as frame.depth
    info: dict           # the reply's non-image fields (mask_source, sam2_score, ...)
    rtt: float           # s, request sent -> reply decoded
    t_done: float        # time.monotonic()
    conf: np.ndarray = None   # C in [0, 1], same shape as depth; None = reply has none
    conf_key: str = None      # reply image it came from
    conf_note: str = ""       # why conf is None, or what was off with it
    points: tuple = ()        # the SAM2 prompts (x, y, label) this reply was made with


def is_weight(key):
    """Is this reply image the weight on the predicted depth (C is then 1 - image)?"""
    return key is not None and key.lower() in WEIGHT_NAMES


def pick_confidence(images, shape, key=None):
    """(C clipped to [0, 1] and shaped `shape`, image name used, note) from a reply's images.

    C is the confidence in the RAW depth. `key` None = look for a confidence-named image or a
    predicted-depth weight (inverted). The note says why there is no map, or what had to be
    adjusted to show it ("" when nothing).
    """
    names = ", ".join(images)
    if key is None:
        key = next((k for k in images if k.lower() in CONF_NAMES + WEIGHT_NAMES), None)
        if key is None:
            return None, None, (f"reply has no confidence image (images: {names}); "
                                "if it is under another name, use --conf-key NAME")
    if key not in images:
        return None, key, f"reply has no image {key!r} (images: {names})"
    arr = images[key]
    if arr.ndim != 2:
        return None, key, f"reply image {key!r} has shape {arr.shape}, expected one H x W channel"
    notes = []
    if arr.dtype.kind in "ui":
        notes.append(f"{arr.dtype} scaled by its full range")
        arr = arr.astype(np.float32) / float(np.iinfo(arr.dtype).max)
    arr = arr.astype(np.float32)
    if arr.shape != shape:
        notes.append(f"resized from {arr.shape[1]}x{arr.shape[0]}")
        arr = cv2.resize(arr, shape[::-1], interpolation=cv2.INTER_LINEAR)
    arr = np.clip(arr, 0.0, 1.0)
    if is_weight(key):
        notes.append("weight on predicted depth, C = 1 - it")
        arr = 1.0 - arr
    return arr, key, "; ".join(notes)


class RestoreWorker(threading.Thread):
    """Always restores the newest frame the grabber has; frames in between are dropped."""

    def __init__(self, client, grabber, prompts, want_conf=False, conf_key=None):
        super().__init__(daemon=True)
        self._client, self._grabber, self._prompts = client, grabber, prompts
        self._want_conf, self._conf_key = want_conf, conf_key
        self._stop_evt = threading.Event()
        self.result = None
        self.error = None
        self.note = None
        self.health = None            # /health dict once the service has answered
        self.takes_prompt = None      # does POST /restore_depth take SAM2 prompts?
        self.waiting_for_prompt = False
        self.rate = RateMeter()
        self._conf_missing = None     # last "no confidence" note printed, to print it once

    def _fail(self, e):
        msg = describe_error(e, self._client.port, self._client.timeout_s)
        if msg != self.error:
            print(f"[:{self._client.port}] {msg}", flush=True)
        self.error = msg
        # A service that rejects every frame must not be hit at camera rate.
        self._stop_evt.wait(0.5)

    def run(self):
        seen = 0
        while not self._stop_evt.is_set():
            if self.health is None:
                try:
                    self.health, self.takes_prompt = self._client.capabilities()
                    self.error = None
                except Exception as e:
                    self._fail(e)
                continue
            points = self._prompts.snapshot() if self.takes_prompt else []
            self.waiting_for_prompt = bool(self.takes_prompt) and not points
            if self.waiting_for_prompt:
                self._stop_evt.wait(0.1)
                continue
            frame = self._grabber.wait_newer(seen, 0.2)
            if frame is None:
                continue
            seen = frame.seq
            t0 = time.monotonic()
            try:
                images, info = self._client.restore(frame.rgb, frame.depth, points)
            except Exception as e:
                self._fail(e)
                continue
            t1 = time.monotonic()
            depth = images["depth"]
            if depth.shape != frame.depth.shape:
                self.note = (f"server returned {depth.shape[1]}x{depth.shape[0]}, "
                             f"resized to {frame.depth.shape[1]}x{frame.depth.shape[0]}")
                depth = cv2.resize(depth, frame.depth.shape[::-1], interpolation=cv2.INTER_NEAREST)
            conf, conf_key, conf_note = (pick_confidence(images, frame.depth.shape, self._conf_key)
                                         if self._want_conf else (None, None, ""))
            if self._want_conf:
                missing = None if conf is not None else conf_note
                if missing and missing != self._conf_missing:
                    print(f"[:{self._client.port}] no confidence map: {missing}", flush=True)
                self._conf_missing = missing
            self.error = None
            self.rate.tick()
            self.result = Result(frame.seq, frame, depth, info, t1 - t0, t1, conf, conf_key, conf_note,
                                 tuple(points))
        self._client.close()

    def stop(self):
        self._stop_evt.set()


# ── Scoring captures ────────────────────────────────────────────────────────

REF_FRAMES = 15             # raw frames medianed into a ground-truth depth (g)
CAPTURE_TIMEOUT_S = 30.0


def load_gt(path, crop):
    """Ground-truth depth from a 16-bit millimetre PNG that is exactly the crop's size."""
    img = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if img is None:
        raise ValueError(f"cannot read {path}")
    if img.dtype != np.uint16 or img.ndim != 2:
        raise ValueError(f"{path} is {img.dtype} {img.shape}: need a single-channel 16-bit PNG in mm")
    if img.shape != (crop[1], crop[0]):
        raise ValueError(f"{path} is {img.shape[1]}x{img.shape[0]} but --crop is {crop[0]}x{crop[1]}: "
                         "the ground truth must be the same view, cut the same way")
    return img


class _Job:
    def __init__(self, kind, need, last_seq):
        self.kind, self.need, self.last_seq = kind, need, last_seq
        self.items, self.t0 = [], time.monotonic()


class CaptureSession:
    """Space scores the next restored frames against a ground truth and saves them; g captures
    that ground truth from the raw depth. Driven from the GUI loop via poll(), never blocks."""

    def __init__(self, args):
        self.args = args
        self.gt, self.gt_source = None, None
        if args.gt:
            self.gt = load_gt(args.gt, args.crop)
            self.gt_source = f"file {Path(args.gt).resolve()}"
        self.job = None
        self._msg = ("", GREY, 0.0)

    def say(self, text, color=GREY, secs=12.0):
        self._msg = (text, color, time.monotonic() + secs)
        print(f"[capture] {text}", flush=True)

    def message(self):
        """(text, colour) to show in the footer, or ("", GREY) when there is nothing to say."""
        if self.job:
            what = "reference depth" if self.job.kind == "reference" else "restored frames"
            return f"capturing {what} {len(self.job.items)}/{self.job.need} ...", WHITE
        text, color, until = self._msg
        return (text, color) if time.monotonic() < until else ("", GREY)

    def start_frames(self, worker):
        if self.job:
            return self.say("already capturing")
        res = worker.result
        if res is None:
            return self.say("nothing restored yet" + (" -- click the object first" if worker.waiting_for_prompt else ""), RED)
        self.job = _Job("frames", self.args.capture_frames, res.seq)     # only frames after the press

    def start_reference(self, grabber):
        if self.job:
            return self.say("already capturing")
        live = grabber.latest()
        if live is None:
            return self.say("no camera frame yet", RED)
        self.job = _Job("reference", REF_FRAMES, live.seq)

    def poll(self, grabber, worker):
        job = self.job
        if job is None:
            return
        if job.kind == "frames":
            res = worker.result
            if res is not None and res.seq > job.last_seq:
                job.items.append(res)
                job.last_seq = res.seq
        else:
            live = grabber.latest()
            if live is not None and live.seq > job.last_seq:
                job.items.append(live.depth)
                job.last_seq = live.seq
        if len(job.items) >= job.need:
            self.job = None
            try:
                self._finish(job, worker)
            except Exception as e:      # a full disk or odd reply must not kill the viewer
                self.say(f"capture failed: {type(e).__name__}: {e}", RED)
        elif time.monotonic() - job.t0 > CAPTURE_TIMEOUT_S:
            self.job = None
            self.say(f"capture timed out: {len(job.items)} of {job.need} after {CAPTURE_TIMEOUT_S:g} s "
                     "(server stalled?)", RED)

    def _finish(self, job, worker):
        a = self.args
        if job.kind == "reference":
            self.gt = restore_eval.reference_depth(job.items)
            self.gt_source = f"captured: per-pixel median of {job.need} raw frames of a static scene"
            valid = float((self.gt > 0).mean())
            return self.say(f"ground truth captured from {job.need} raw frames ({100 * valid:.0f}% of pixels "
                            "have depth). Camera must not move now.", GREEN if valid > 0.5 else AMBER)
        trials = [dict(seq=r.seq, rgb=r.frame.rgb, raw=r.frame.depth, restored=r.depth,
                       server_ms=r.rtt * 1e3, latency_ms=(r.t_done - r.frame.t_rx) * 1e3,
                       info=r.info, points=r.points) for r in job.items]
        meta = {"model": {"name": MODEL_NAMES[a.model], "port": a.model,
                          "health": worker.health or {}},
                "camera": {"crop": list(a.crop), "color_topic": a.color_topic, "depth_topic": a.depth_topic},
                "prompts": [list(p) for p in job.items[-1].points]}
        out, rec = restore_eval.write_capture(a.out_dir, f"restore_{a.model}", meta, trials, self.gt,
                                              self.gt_source, a.fail_mm, a.range)
        try:
            shown = out.relative_to(REPO)
        except ValueError:                  # --out-dir outside the repo: the last two parts are what you need
            shown = Path("...") / out.parent.name / out.name
        if rec["summary"] is None:
            return self.say(f"saved {shown} WITHOUT metrics: no ground truth (press g, or pass --gt)", AMBER)
        obj = rec["summary"]["object"]
        rm, raw = obj["restored"]["rmse_m"], obj["raw"]["rmse_m"]
        line = f"object RMSE {rm['mean']:.4f} m" if rm else "object region empty"
        line += f" (raw sensor {raw['mean']:.4f})" if raw else " (raw sensor: no depth there)"
        self.say(f"saved {shown}  |  {line}  |  open report.html", GREEN)


def handle_key(k, prompts, session, grabber, worker):
    """Apply one cv2.waitKey code. Returns True to quit."""
    if k in (ord("q"), 27):
        return True
    if k == ord("c"):
        prompts.clear()
    elif k == ord("u"):
        prompts.undo()
    elif k == 32:
        session.start_frames(worker)
    elif k == ord("g"):
        session.start_reference(grabber)
    return False


# ── Drawing ─────────────────────────────────────────────────────────────────

colorize = restore_eval.colorize_depth      # one colour scale for the window and the saved views


def put_text(img, text, org, scale=0.5, color=WHITE, thick=1):
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), thick + 2, cv2.LINE_AA)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick, cv2.LINE_AA)


def placeholder(size, lines, color=GREY):
    img = np.zeros((size[1], size[0], 3), np.uint8)
    for i, line in enumerate(lines):
        put_text(img, line, (12, size[1] // 2 - 12 + 22 * i), 0.5, color)
    return img


def draw_prompts(img, points):
    for x, y, label in points:
        color = GREEN if label else RED
        cv2.circle(img, (x, y), 7, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.circle(img, (x, y), 7, color, -1, cv2.LINE_AA)
        cv2.circle(img, (x, y), 7, WHITE, 1, cv2.LINE_AA)


def compose(rows, lo, hi, conf_bar=False, hint="", hint_color=GREY):
    """`rows` stacked top to bottom, each a pair of (panel, title, stats, warn); every row has
    a title/stats header, and a two-line footer carries the colour scales and key hints."""
    ch, cw = rows[0][0][0].shape[:2]
    band = HEADER + ch
    canvas = np.zeros((band * len(rows) + FOOTER, 2 * cw + DIVIDER, 3), np.uint8)
    for r, row in enumerate(rows):
        for i, (panel, title, stat, warn) in enumerate(row):
            x, y = i * (cw + DIVIDER), r * band
            canvas[y + HEADER:y + band, x:x + cw] = panel
            put_text(canvas, title, (x + 8, y + 20), 0.6, WHITE, 1)
            put_text(canvas, stat, (x + 8, y + 39), 0.45, RED if warn else GREY)
    y = band * len(rows) + 5
    bar = cv2.applyColorMap(np.linspace(0, 255, 240, dtype=np.uint8)[None, :], cv2.COLORMAP_TURBO)
    canvas[y:y + 10, 8:248] = bar
    both = "both depth panels" if conf_bar else "both panels"
    put_text(canvas, f"{lo:.2f} to {hi:.2f} m on {both}; black = no depth", (258, y + 9), 0.4, GREY)
    if conf_bar:
        bar = cv2.applyColorMap(np.linspace(0, 255, 120, dtype=np.uint8)[None, :], cv2.COLORMAP_VIRIDIS)
        canvas[y:y + 10, 650:770] = bar
        put_text(canvas, "0 to 1 on C panels", (778, y + 9), 0.4, GREY)
    max_w = canvas.shape[1] - 130           # keep clear of "q / Esc quits"
    if cv2.getTextSize(hint, cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1)[0][0] > max_w:
        n = len(hint)
        while n > 1 and cv2.getTextSize(hint[:n] + "...", cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1)[0][0] > max_w:
            n -= 1
        hint = hint[:n] + "..."
    put_text(canvas, hint, (8, y + 28), 0.4, hint_color)
    put_text(canvas, "q / Esc quits", (canvas.shape[1] - 105, y + 28), 0.4, GREY)
    return canvas


def wrapped(text, width=68, lines=3):
    return textwrap.wrap(text, width)[:lines]


def left_frame(args, live, res):
    """Frame behind the left panel: the one the right panel came from, so the two stay
    comparable. Until there is a result to pair with (service still starting, waiting for a
    click, tunnel down) or with --live-left, the live camera."""
    return res.frame if res and not args.live_left else live


def confidence_row(args, res):
    """Second row: C and 1 - C from the reply's confidence image. Both panels stay empty
    (black) while there is no reply or the reply carries none (:31000 today)."""
    if res is None or res.conf is None:
        empty = np.zeros((args.crop[1], args.crop[0], 3), np.uint8)
        stat, warn = "", False
        if res is not None:
            # Nothing named like a confidence image is normal; a --conf-key that isn't there is not.
            warn = res.conf_key is not None
            stat = res.conf_note[:75] if warn else "no confidence image in reply"
        return [(empty, "C", stat, warn), (empty, "1 - C", "", False)]
    holes = res.frame.depth == 0
    row = []
    source = f"1 - reply image '{res.conf_key}'" if is_weight(res.conf_key) else f"reply image '{res.conf_key}'"
    for title, m in ((f"C   ({source})", res.conf), ("1 - C", 1.0 - res.conf)):
        img = cv2.applyColorMap((m * 255).astype(np.uint8), cv2.COLORMAP_VIRIDIS)
        stat = f"mean {m.mean():.2f}"
        if holes.any():
            stat += f"   at raw-depth holes {m[holes].mean():.2f}"
        note = wrapped(res.conf_note)
        for i, line in enumerate(note):
            put_text(img, line, (10, args.crop[1] - 10 - 20 * (len(note) - 1 - i)), 0.5, GREY)
        row.append((img, title, stat, False))
    return row


def render(args, grabber, worker, prompts, left_src, res, waited, session=None):
    """The window's contents for the current state of the camera and server threads."""
    lo, hi = args.range
    size = args.crop
    err = worker.error
    points = prompts.snapshot() if worker.takes_prompt else []

    if left_src:
        left = colorize(left_src.depth, lo, hi)
    elif grabber.error:
        left = placeholder(size, ["camera frames rejected:"] + wrapped(grabber.error), RED)
    elif waited > 3:
        left = placeholder(size, [f"no colour + depth pair on {args.color_topic}",
                                  f"and {args.depth_topic}",
                                  "(driver up? align_depth.enable:=true?)"])
    else:
        left = placeholder(size, ["waiting for the first camera frame ..."])
    draw_prompts(left, points)
    left_stat = f"camera {grabber.rate.hz:4.1f} Hz" + ("   (live)" if args.live_left else "")
    if session is not None:
        left_stat += f"   ground truth: {'yes' if session.gt is not None else 'no'}"
    if grabber.error:
        left_stat += f"   {grabber.error[:40]}"

    model = MODEL_NAMES[args.model]
    right_title = f"restored depth  (:{args.model}" + (f" {model})" if model else ")")
    if res:
        age = time.monotonic() - res.t_done
        right = colorize(res.depth, lo, hi)
        draw_prompts(right, points)
        note = wrapped(err) if err else ([worker.note] if worker.note else [])
        for i, line in enumerate(note):
            put_text(right, line, (10, size[1] - 10 - 20 * (len(note) - 1 - i)), 0.5,
                     RED if err else GREY)
        right_stat = (f"{worker.rate.hz:4.1f} Hz   server {res.rtt * 1e3:4.0f} ms   "
                      f"frame to restored {(res.t_done - res.frame.t_rx) * 1e3:4.0f} ms")
        if err:
            right_stat = "server error -- showing the last good result"
        elif age > 1.0:
            right_stat += f"   NO UPDATE {age:.1f} s"
        elif res.info.get("sam2_score") is not None:
            right_stat += f"   SAM2 {res.info['sam2_score']:.2f}"
        right_warn = bool(err) or age > 1.0
    else:
        if worker.waiting_for_prompt:
            lines = ["click the transparent object in the left image",
                     "(left click = object, right click = background)"]
            color = GREY
        else:
            lines = [f"waiting for the first reply from :{args.model} ..."] + (wrapped(err) if err else [])
            color = RED if err else GREY
        right = placeholder(size, lines, color)
        right_stat, right_warn = "", False

    rows = [[(left, "original depth", left_stat, bool(grabber.error)),
             (right, right_title, right_stat, right_warn)]]
    if args.confidence:
        rows.append(confidence_row(args, res))
    hint = ("left click: object   right click: background   c: clear   u: undo"
            if worker.takes_prompt else "")
    hint_color = GREY
    if session is not None:
        hint = (hint + "   " if hint else "") + "space: capture & score   g: ground-truth depth"
        text, color = session.message()
        if text:
            hint, hint_color = text, color
    conf_shown = args.confidence and res is not None and res.conf is not None
    return compose(rows, lo, hi, conf_bar=conf_shown, hint=hint, hint_color=hint_color)


def mouse_handler(prompts, size):
    cw, ch = size

    def on_mouse(event, x, y, flags, param):
        if event not in (cv2.EVENT_LBUTTONDOWN, cv2.EVENT_RBUTTONDOWN) or not HEADER <= y < HEADER + ch:
            return
        if x < cw:
            px = x
        elif x >= cw + DIVIDER:
            px = x - (cw + DIVIDER)
        else:
            return
        if 0 <= px < cw:        # a click on either depth panel means the same pixel
            prompts.add(px, y - HEADER, 1 if event == cv2.EVENT_LBUTTONDOWN else 0)
    return on_mouse


# ── Probe ───────────────────────────────────────────────────────────────────

def _timed(fn, n=3):
    times = []
    for _ in range(n):
        t0 = time.monotonic()
        out = fn()
        times.append((time.monotonic() - t0) * 1e3)
    return times, out


def probe(host, size, timeout_s):
    """Send synthetic frames to every service and print what it is, what it returns, and what
    SAM2 prompting costs per frame."""
    w, h = size
    yy, xx = np.mgrid[0:h, 0:w]
    rgb = np.stack([xx * 255 // w, yy * 255 // h, np.full_like(xx, 128)], -1).astype(np.uint8)
    depth = np.round((0.55 + 0.0002 * xx + 0.0001 * yy) * 1000).astype(np.uint16)
    hole = np.zeros((h, w), np.uint8)
    hole[h // 3:2 * h // 3, w // 3:2 * w // 3] = 255
    depth[hole > 0] = 0                               # the "transparent" hole
    centre = [(w // 2, h // 2, 1)]
    print(f"probe: {w}x{h} synthetic frame (a gradient scene, so SAM2's mask is meaningless -- "
          f"only the timings are), timeout {timeout_s:g} s\n")

    for port in MODEL_PORTS:
        print(f"── {MODEL_NAMES[port]} ({host}:{port})")
        with socket.socket() as s:      # a closed tunnel refuses at once
            s.settimeout(1.0)
            if s.connect_ex((host, port)) != 0:
                print("   nothing is listening -- the Seraph tunnel is not open "
                      "(source environments/ros2/seraph_env.sh)\n")
                continue
        client = RestoreClient(host, port, timeout_s)
        try:
            health, takes_prompt = client.capabilities()
            print(f"   /health: {json.dumps(health)}")
            print(f"   takes SAM2 prompts: {'yes (needs a mask or a click)' if takes_prompt else 'no'}")
            base = (lambda: client.restore(rgb, depth, None, hole)) if takes_prompt \
                else (lambda: client.restore(rgb, depth))
            base()                                                  # warm-up, not timed
            times, (images, info) = _timed(base)
            label = "supplied mask" if takes_prompt else "rgb + depth"
            print(f"   {label:<14} best of 3: {min(times):4.0f} ms   (all: {', '.join(f'{t:.0f}' for t in times)})")
            if takes_prompt:
                times2, (images2, info2) = _timed(lambda: client.restore(rgb, depth, centre))
                print(f"   {'SAM2 click':<14} best of 3: {min(times2):4.0f} ms   "
                      f"-> prompting costs about {min(times2) - min(times):+.0f} ms per frame")
                info = info2
            d = images["depth"]
            print(f"   reply: depth {d.shape} {d.dtype} [{d.min()}..{d.max()}] mm"
                  + "".join(f" | {k}={v!r}" for k, v in info.items()))
            conf, key, note = pick_confidence(images, depth.shape)
            print("   -> compatible; " + (f"confidence image {key!r}" + (f" ({note})" if note else "")
                                          if conf is not None else f"no confidence map ({note})"))
        except Exception as e:
            print(f"   {describe_error(e, port, timeout_s)}")
        finally:
            client.close()
        print()


# ── Main ────────────────────────────────────────────────────────────────────

def model_port(value):
    """Accept a model name (including FRAME) or its existing service port."""
    for port, name in MODEL_NAMES.items():
        if value.strip().lower() in (name.lower(), str(port)):
            return port
    raise argparse.ArgumentTypeError("choose SwinDRNet/30000, ReMake/31000, or FRAME/32000")


def parse_args():
    ap = argparse.ArgumentParser(
        description="Raw vs restored depth, live.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--model", type=model_port, choices=MODEL_PORTS, default=30000,
                    help="restoration model name (SwinDRNet, ReMake, FRAME) or its localhost port")
    ap.add_argument("--host", default="127.0.0.1", help="where the Seraph tunnels listen")
    ap.add_argument("--probe", action="store_true",
                    help="send synthetic frames to all three ports, print what they say and how "
                         "fast they are, exit")
    ap.add_argument("--crop", default="640x360", metavar="WxH",
                    help="window cut from each frame and sent to the model")
    ap.add_argument("--crop-origin", type=int, nargs=2, metavar=("X", "Y"),
                    help="crop's top-left corner in the full frame (default: centred)")
    ap.add_argument("--range", type=float, nargs=2, default=(0.25, 1.0), metavar=("MIN", "MAX"),
                    help="depth range of the colour scale, metres")
    ap.add_argument("--live-left", action="store_true",
                    help="left panel = live camera instead of the frame the right panel came from")
    ap.add_argument("--confidence", action=argparse.BooleanOptionalAction, default=True,
                    help="show the C / 1 - C row (its panels stay empty if the reply has no "
                         "confidence image)")
    ap.add_argument("--conf-key", metavar="NAME",
                    help="reply image to show as C (default: the first named confidence / conf / ..., "
                         "or pred_weight, which is shown as 1 - pred_weight)")
    ap.add_argument("--gt", metavar="PNG",
                    help="ground-truth depth for scoring: 16-bit PNG in millimetres, exactly the --crop "
                         "size (alternative to pressing g)")
    ap.add_argument("--capture-frames", type=int, default=5,
                    help="space captures and scores this many restored frames")
    ap.add_argument("--fail-mm", type=float, default=20.0,
                    help="the 'object' region is where the raw depth is missing or differs from the "
                         "ground truth by more than this")
    ap.add_argument("--out-dir", default=str(REPO / "data" / "captures"),
                    help="where captures are saved (git ignores data/)")
    ap.add_argument("--timeout-ms", type=int, default=10000,
                    help="per-request timeout (the first request after idle takes ~2.5 s)")
    ap.add_argument("--slop", type=float, default=0.05,
                    help="max colour/depth timestamp gap to pair, s")
    ap.add_argument("--color-topic", default=COLOR_TOPIC)
    ap.add_argument("--depth-topic", default=DEPTH_TOPIC, help="must be aligned to colour")
    args = ap.parse_args()
    try:
        args.crop = tuple(int(v) for v in args.crop.lower().split("x"))
        assert len(args.crop) == 2 and min(args.crop) > 0
    except (ValueError, AssertionError):
        ap.error(f"--crop expects WxH like 640x360, got {args.crop!r}")
    if args.range[0] >= args.range[1]:
        ap.error("--range MIN must be below MAX")
    if args.capture_frames < 1:
        ap.error("--capture-frames must be at least 1")
    if args.fail_mm <= 0:
        ap.error("--fail-mm must be positive")
    return args


def main():
    args = parse_args()
    timeout_s = args.timeout_ms / 1000
    if args.probe:
        probe(args.host, args.crop, timeout_s)
        return

    try:
        session = CaptureSession(args)
    except ValueError as e:
        raise SystemExit(f"--gt: {e}")
    prompts = Prompts()
    grabber = FrameGrabber(args.color_topic, args.depth_topic, args.crop,
                           tuple(args.crop_origin) if args.crop_origin else None, args.slop)
    worker = RestoreWorker(RestoreClient(args.host, args.model, timeout_s), grabber, prompts,
                           args.confidence, args.conf_key)
    worker.start()

    win = f"raw vs restored depth  (model :{args.model})"
    cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(win, mouse_handler(prompts, args.crop))
    t_start = time.monotonic()
    drawn_key, drawn_at = None, 0.0
    try:
        while True:
            now = time.monotonic()
            res, live = worker.result, grabber.latest()
            left_src = left_frame(args, live, res)
            key = (res.seq if res else 0, left_src.seq if left_src else 0, prompts.version,
                   session.message()[0])
            # Redraw on new data, and twice a second so rates, ages and errors stay current.
            if key != drawn_key or now - drawn_at > 0.5:
                drawn_key, drawn_at = key, now
                cv2.imshow(win, render(args, grabber, worker, prompts, left_src, res, now - t_start,
                                       session))
            k = cv2.waitKey(5) & 0xFF
            session.poll(grabber, worker)
            # VISIBLE reads -1 ("unknown") on some OpenCV builds, even for a shown window,
            # so only 0 means closed.
            if handle_key(k, prompts, session, grabber, worker) or \
                    cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) == 0:
                break
    except KeyboardInterrupt:
        pass
    finally:
        worker.stop()
        worker.join(timeout=timeout_s + 1)
        grabber.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
