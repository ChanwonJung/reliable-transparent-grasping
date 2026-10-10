import json

import pytest

from depth_restoration_ros.synchronization import ScanCache, parse_detections


def test_out_of_order_messages_pair_rgb_exactly_and_depth_within_slop():
    cache = ScanCache()
    rgb_stamp = 1_000_000_000
    depth_stamp = rgb_stamp + 20_000_000
    cache.add('mask', rgb_stamp, 'mask')
    cache.add('info', depth_stamp, 'info')
    cache.add('depth', depth_stamp, 'depth')
    cache.add('rgb', rgb_stamp, 'rgb')
    assert cache.ready() is None
    cache.add('detections', rgb_stamp, [{'category': 'TARGET'}])
    scan = cache.ready()
    assert scan.stamp_ns == rgb_stamp
    assert scan.depth_stamp_ns == depth_stamp
    assert (scan.rgb, scan.depth, scan.info, scan.mask) == ('rgb', 'depth', 'info', 'mask')
    cache.mark_complete(rgb_stamp)
    assert cache.ready() is None


def test_stale_depth_and_wrong_scan_detections_do_not_advance():
    cache = ScanCache()
    for kind in ('rgb', 'mask'):
        cache.add(kind, 1_000_000_000, kind)
    cache.add('detections', 2_000_000_000, [])
    cache.add('depth', 1_100_000_000, 'stale_depth')
    cache.add('info', 1_100_000_000, 'stale_info')
    assert cache.ready() is None
    cache.add('detections', 1_000_000_000, [])
    assert cache.ready() is None
    cache.add('depth', 1_030_000_000, 'correct_depth')
    cache.add('info', 1_030_000_000, 'correct_info')
    assert cache.ready().depth == 'correct_depth'


def test_cache_size_and_input_expiry(monkeypatch):
    clock = [0.0]
    monkeypatch.setattr('depth_restoration_ros.synchronization.time.monotonic', lambda: clock[0])
    cache = ScanCache(max_size=2, timeout_s=1)
    for stamp in (1, 2, 3):
        cache.add('mask', stamp, object())
    assert len(cache._buffers['mask']) == 2
    clock[0] = 2
    assert cache.expire() == [2, 3]
    assert cache.expire() == []


def test_stamped_list_and_envelope():
    detections = [{'category': 'TARGET', 'source_stamp_ns': 123}]
    assert parse_detections(json.dumps(detections)) == (123, detections)
    assert parse_detections(json.dumps({'source_stamp_ns': 123, 'detections': detections})) == (
        123, detections)
    for bad in ([{'category': 'TARGET'}],
                [{'source_stamp_ns': 123}, {'source_stamp_ns': 456}],
                {'source_stamp_ns': 123, 'detections': [{'source_stamp_ns': 456}]},
                {'source_stamp_ns': 123, 'detections': []}):
        with pytest.raises(ValueError):
            parse_detections(json.dumps(bad))
