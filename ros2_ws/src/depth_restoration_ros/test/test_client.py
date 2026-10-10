"""Exercise real multipart HTTP requests against a local service double."""

import base64
from email import policy
from email.parser import BytesParser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import threading
import time

import cv2
import numpy as np
import pytest

from depth_restoration_ros.client import (
    MODEL_PORTS, RestorationError, RestoreClient, depth_to_mm, merge_depth, target_mask,
)


@pytest.fixture
def service():
    class Handler(BaseHTTPRequestHandler):
        def do_POST(self):
            raw = self.rfile.read(int(self.headers['Content-Length']))
            message = BytesParser(policy=policy.default).parsebytes(
                b'Content-Type: ' + self.headers['Content-Type'].encode() + b'\r\n\r\n' + raw)
            images = {}
            for part in message.iter_parts():
                name = part.get_param('name', header='content-disposition')
                images[name] = cv2.imdecode(
                    np.frombuffer(part.get_payload(decode=True), np.uint8), cv2.IMREAD_UNCHANGED)
            self.server.received.append((self.path, images))
            if self.server.mode == 'timeout':
                time.sleep(0.1)
            depth = self.server.depth
            if self.server.mode == 'uint8':
                depth = depth.astype(np.uint8)
            if self.server.mode == 'shape':
                depth = depth[:1, :1]
            if self.server.mode == 'zeros':
                depth = np.zeros_like(depth)
            reply = {'depth_png_base64': base64.b64encode(cv2.imencode('.png', depth)[1]).decode()}
            if self.server.mode == 'missing':
                reply = {'detail': 'model failed'}
            if self.server.mode == 'bad_base64':
                reply['depth_png_base64'] = 'not base64!!!'
            if self.server.mode == 'bad_png':
                reply['depth_png_base64'] = base64.b64encode(b'not a PNG').decode()
            payload = b'not JSON' if self.server.mode == 'bad_json' else json.dumps(reply).encode()
            self.send_response(503 if self.server.mode == 'http_error' else 200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(payload)))
            self.end_headers()
            try:
                self.wfile.write(payload)
            except BrokenPipeError:
                pass

        def log_message(self, *_args):
            pass

    server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
    server.depth = np.full((3, 4), 1200, dtype=np.uint16)
    server.received = []
    server.mode = 'ok'
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    yield server
    server.shutdown()
    server.server_close()
    thread.join(timeout=2)


def inputs():
    rgb = np.zeros((3, 4, 3), dtype=np.uint8)
    rgb[..., 0] = 200
    raw = np.full((3, 4), 800, dtype=np.uint16)
    mask = np.zeros((3, 4), dtype=np.uint8)
    mask[1:, 1:3] = 255
    return rgb, raw, mask


@pytest.mark.parametrize('model', list(MODEL_PORTS))
def test_all_models_use_shared_full_frame_contract(service, model):
    client = RestoreClient(model, url=f'http://127.0.0.1:{service.server_port}/restore_depth')
    rgb, raw, mask = inputs()
    restored = client.restore(rgb, raw, mask)
    client.close()
    assert np.array_equal(restored, service.depth)
    path, sent = service.received[0]
    assert path == '/restore_depth'
    assert set(sent) == {'rgb', 'depth', 'mask'}
    assert np.array_equal(sent['rgb'], rgb[..., ::-1])
    assert np.array_equal(sent['depth'], raw)
    assert np.array_equal(sent['mask'], mask)
    output = merge_depth(raw, restored, mask)
    assert output.dtype == np.float32
    assert np.allclose(output[mask > 0], 1.2)
    assert np.allclose(output[mask == 0], 0.8)


@pytest.mark.parametrize('mode,match', [
    ('missing', 'depth_png_base64'), ('bad_base64', 'valid PNG'),
    ('bad_png', 'uint16'), ('bad_json', 'not JSON'),
    ('uint8', 'uint16'), ('shape', 'shape'), ('zeros', 'valid TARGET'),
    ('http_error', 'HTTP 503'),
])
def test_invalid_service_response_never_returns_raw(service, mode, match):
    service.mode = mode
    client = RestoreClient(url=f'http://127.0.0.1:{service.server_port}/restore_depth')
    with pytest.raises(RestorationError, match=match):
        client.restore(*inputs())
    client.close()


def test_timeout_is_bounded(service):
    service.mode = 'timeout'
    client = RestoreClient(url=f'http://127.0.0.1:{service.server_port}/restore_depth', timeout_s=0.02)
    with pytest.raises(RestorationError, match='request failed'):
        client.restore(*inputs())
    client.close()
    assert len(service.received) == 1


def test_units_invalid_values_and_defaults():
    depth = np.array([[1.234, 0, np.nan, np.inf, -1, 70]], np.float32)
    assert depth_to_mm(depth, '32FC1').tolist() == [[1234, 0, 0, 0, 0, 65535]]
    with pytest.raises(RestorationError):
        depth_to_mm(depth, 'mono8')
    for model, port in MODEL_PORTS.items():
        assert RestoreClient(model).url == f'http://127.0.0.1:{port}/restore_depth'
    with pytest.raises(ValueError):
        RestoreClient('invented')


@pytest.mark.parametrize('model', ['frame', 'FRAME', ' Frame '])
def test_frame_name_selects_the_third_service(model):
    client = RestoreClient(model)
    assert client.model == 'frame'
    assert client.url == 'http://127.0.0.1:32000/restore_depth'


def test_port_is_the_endpoints_own_port():
    for model, port in MODEL_PORTS.items():
        assert RestoreClient(model).port == port
    assert RestoreClient('remake', url='http://10.0.0.5:8123/restore_depth').port == 8123
    assert RestoreClient('remake', url='https://gpu.example/restore_depth').port == 443
    assert RestoreClient('remake', url='http://gpu.example/restore_depth').port == 80


def test_target_uses_original_detection_indices():
    detections = [{'category': 'OBSTACLE'}, {'category': 'TARGET'},
                  {'category': 'DESTINATION'}, {'category': 'target'}]
    labels = np.array([[0, 1, 2, 3, 4]], np.uint8)
    assert target_mask(labels, detections).tolist() == [[0, 0, 255, 0, 255]]
    with pytest.raises(RestorationError, match='No TARGET'):
        target_mask(labels, [{'category': 'DESTINATION'}])
    with pytest.raises(RestorationError, match='empty'):
        target_mask(np.zeros_like(labels), detections)


def test_input_alignment_is_required(service):
    client = RestoreClient(url=f'http://127.0.0.1:{service.server_port}/restore_depth')
    rgb, depth, mask = inputs()
    with pytest.raises(RestorationError, match='aligned'):
        client.restore(rgb[:1], depth, mask)
    assert not service.received
