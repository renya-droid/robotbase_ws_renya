import io
from types import SimpleNamespace

import robotbase_waypoint.waypoint_record as waypoint_record


class _Logger:
    def info(self, _message):
        pass


def _recorder(latest_pose, last_position):
    recorder = waypoint_record.WaypointRecorder.__new__(
        waypoint_record.WaypointRecorder)
    recorder.continuous = True
    recorder.latest_pose = latest_pose
    recorder.last_position = last_position
    recorder.output_file = '/tmp/route.yaml'
    recorder.enter_to_record = False
    recorder.pause_waypoint_requested = False
    recorder.hokuyo_waypoint_requested = False
    recorder.pause_waypoint_topic = '/waypoint_record/pause'
    recorder._stdin_fd = None
    recorder._stdin_attributes = None
    recorder.get_logger = lambda: _Logger()
    return recorder


def test_save_final_waypoint_appends_latest_pose(monkeypatch):
    calls = []

    def fake_append(file_path, x, y, yaw):
        calls.append((file_path, x, y, yaw))
        return SimpleNamespace(
            number=3, x=x, y=y, angle_radians=yaw)

    monkeypatch.setattr(waypoint_record, 'append_waypoint', fake_append)
    recorder = _recorder((3.0, 4.0, 0.5), (0.0, 0.0))

    recorder.save_final_waypoint()

    assert calls == [('/tmp/route.yaml', 3.0, 4.0, 0.5)]
    assert recorder.last_position == (3.0, 4.0)


def test_save_final_waypoint_skips_duplicate(monkeypatch):
    calls = []
    monkeypatch.setattr(
        waypoint_record, 'append_waypoint',
        lambda *args: calls.append(args))
    recorder = _recorder((3.0, 4.0, 0.5), (3.0, 4.0))

    recorder.save_final_waypoint()

    assert calls == []


def test_enter_mode_records_each_press_and_not_automatically(monkeypatch):
    calls = []
    recorder = _recorder(None, None)
    recorder.continuous = False
    recorder.enter_to_record = True
    recorder.map_frame = 'map'
    recorder.base_frame = 'robot/base_footprint'
    recorder.tf_buffer = SimpleNamespace(lookup_transform=lambda *_: SimpleNamespace(
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=1.0, y=2.0),
            rotation=SimpleNamespace(w=1.0, x=0.0, y=0.0, z=0.0))))
    monkeypatch.setattr(waypoint_record.sys, 'stdin', io.StringIO('\n\n'))
    ready = iter([False, True, True])
    monkeypatch.setattr(
        waypoint_record.select, 'select',
        lambda *_: ([waypoint_record.sys.stdin] if next(ready) else [], [], []))
    monkeypatch.setattr(
        waypoint_record, 'append_waypoint',
        lambda file_path, x, y, yaw: (
            calls.append((file_path, x, y, yaw)) or
            SimpleNamespace(number=len(calls), x=x, y=y, angle_radians=yaw)))
    monkeypatch.setattr(waypoint_record.rclpy, 'shutdown', lambda: None)

    recorder._timer_callback()
    assert calls == []
    recorder._timer_callback()
    recorder._timer_callback()

    assert calls == [('/tmp/route.yaml', 1.0, 2.0, 0.0)] * 2
    assert recorder.last_position == (1.0, 2.0)
    recorder.save_final_waypoint()
    assert len(calls) == 2


def test_pause_key_is_read_without_enter_on_a_terminal(monkeypatch):
    recorder = _recorder(None, None)
    recorder._stdin_fd = 7
    monkeypatch.setattr(
        waypoint_record.select, 'select',
        lambda *_: ([waypoint_record.sys.stdin], [], []))
    monkeypatch.setattr(waypoint_record.os, 'read', lambda *_: b'p')

    assert recorder._read_record_command() == 'pause'


def test_pause_key_records_ten_second_stop_while_continuous(monkeypatch):
    calls = []
    recorder = _recorder(None, (1.0, 2.0))
    recorder.map_frame = 'map'
    recorder.base_frame = 'robot/base_footprint'
    recorder.distance_threshold = 2.0
    recorder.tf_buffer = SimpleNamespace(lookup_transform=lambda *_: SimpleNamespace(
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=1.0, y=2.0),
            rotation=SimpleNamespace(w=1.0, x=0.0, y=0.0, z=0.0))))
    monkeypatch.setattr(waypoint_record.sys, 'stdin', io.StringIO(''))
    monkeypatch.setattr(
        waypoint_record.select, 'select', lambda *_: ([], [], []))

    def fake_append(file_path, x, y, yaw, **options):
        calls.append((file_path, x, y, yaw, options))
        return SimpleNamespace(number=3, x=x, y=y, angle_radians=yaw)

    monkeypatch.setattr(waypoint_record, 'append_waypoint', fake_append)

    recorder._pause_waypoint_callback(SimpleNamespace())
    recorder._timer_callback()

    assert calls == [(
        '/tmp/route.yaml', 1.0, 2.0, 0.0,
        {'stop': True, 'wait_time': 10.0},
    )]


def test_hokuyo_request_records_immediately_and_survives_tf_failure(monkeypatch, tmp_path):
    recorder = _recorder(None, (1.0, 2.0))
    recorder.output_file = str(tmp_path / 'route.yaml')
    recorder.map_frame = 'map'
    recorder.base_frame = 'robot/base_footprint'
    recorder.distance_threshold = 2.0
    recorder._read_record_command = lambda: None
    recorder.get_logger = lambda: SimpleNamespace(
        info=lambda *_: None, warning=lambda *a, **k: None)

    def unavailable(*_):
        raise waypoint_record.TransformException('no TF')

    recorder.tf_buffer = SimpleNamespace(lookup_transform=unavailable)
    recorder._hokuyo_waypoint_callback(SimpleNamespace())
    recorder._timer_callback()
    assert recorder.hokuyo_waypoint_requested
    recorder.tf_buffer.lookup_transform = lambda *_: SimpleNamespace(transform=SimpleNamespace(
        translation=SimpleNamespace(x=1.0, y=2.0),
        rotation=SimpleNamespace(w=1.0, x=0.0, y=0.0, z=0.0)))
    recorder._timer_callback()
    assert not recorder.hokuyo_waypoint_requested
    from robotbase_waypoint.waypoint_io import load_waypoints
    waypoints = load_waypoints(recorder.output_file)
    assert len(waypoints) == 1
    assert waypoints[0].hokuyo_disable_for == 2
    assert not waypoints[0].stop


def test_hokuyo_key_is_read_without_enter(monkeypatch):
    recorder = _recorder(None, None)
    recorder._stdin_fd = 7
    monkeypatch.setattr(waypoint_record.select, 'select', lambda *_: ([None], [], []))
    monkeypatch.setattr(waypoint_record.os, 'read', lambda *_: b'h')
    assert recorder._read_record_command() == 'hokuyo'
