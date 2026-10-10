"""Check FRAME selection and capture naming without ROS or a model server."""

import argparse

import numpy as np
import pytest

import restore_depth_viewer as viewer
import restore_eval


@pytest.mark.parametrize('selection', ['FRAME', 'frame', ' Frame ', '32000'])
def test_viewer_selects_frame(selection, monkeypatch):
    monkeypatch.setattr('sys.argv', ['viewer', '--model', selection])
    assert viewer.parse_args().model == 32000


def test_viewer_rejects_unknown_model():
    with pytest.raises(argparse.ArgumentTypeError, match='FRAME/32000'):
        viewer.model_port('unknown')


def test_frame_capture_uses_decided_name_and_preserves_service_metadata(tmp_path):
    raw = np.full((4, 6), 1000, np.uint16)
    raw[1:3, 2:4] = 0
    restored = np.full_like(raw, 1000)
    rgb = np.zeros((*raw.shape, 3), np.uint8)
    meta = {
        'model': {'name': 'FRAME', 'port': 32000, 'health': {'model': 'server-internal-name'}},
        'camera': {'crop': [6, 4]}, 'prompts': [[2, 1, 1]],
    }
    trial = dict(seq=1, rgb=rgb, raw=raw, restored=restored, server_ms=10,
                 latency_ms=12, info={}, points=[(2, 1, 1)])
    out, record = restore_eval.write_capture(
        tmp_path, 'frame', meta, [trial], restored, 'test reference', 20, (0.25, 1.5))
    assert record['model']['name'] == 'FRAME'
    assert record['model']['health']['model'] == 'server-internal-name'
    assert '32000 FRAME' in (out / 'report.html').read_text()
    assert record['summary']['object']['restored']['rmse_m']['mean'] == 0
