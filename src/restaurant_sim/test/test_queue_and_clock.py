"""Service FIFO, seated clearance, fixed-step clock and failure policy."""
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest
from builtin_interfaces.msg import Time
from restaurant_sim.controller import Router
from restaurant_sim.service_queue import ServiceQueues

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    'queue_clock_reproducer', ROOT.parents[1] / 'tools/restaurant_ralph/reproduce_issues.py')
repro = importlib.util.module_from_spec(spec)
spec.loader.exec_module(repro)


def set_clock(node, ns):
    stamp = Time(sec=ns // 10**9, nanosec=ns % 10**9)
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=ns, to_msg=lambda: stamp))


def customer(name, service='cashier'):
    return dict(name=name, steps=[(service, (5.25, 9.7), 12)], index=0, done=False)


def test_service_queue_is_fifo_with_one_server():
    queues = ServiceQueues.from_file(ROOT / 'config/service_queues.json', Router({'obstacles': []}))
    slots = queues.facilities['cashier']['slots']
    a, b, c = customer('a'), customer('b'), customer('c')
    assert queues.target(a, (5.25, 9.7), 0) == ((5.25, 9.7), False)
    assert queues.target(b, (5.25, 9.7), 1) == (slots[0], True)
    assert queues.target(c, (5.25, 9.7), 2) == (slots[1], True)
    a['index'] = 1  # service finished
    assert queues.update([a, b, c], 20) == []
    assert queues.target(b, (5.25, 9.7), 20) == ((5.25, 9.7), False)
    assert queues.target(c, (5.25, 9.7), 20) == (slots[0], True)
    assert [(e['event'], e['agent']) for e in queues.events] == [
        ('join', 'a'), ('serve', 'a'), ('join', 'b'), ('join', 'c'), ('leave', 'a'), ('serve', 'b')]


def test_queue_wait_beyond_limit_is_reported():
    queues = ServiceQueues.from_file(ROOT / 'config/service_queues.json', Router({'obstacles': []}))
    a, b = customer('a'), customer('b')
    queues.target(a, (5.25, 9.7), 0)
    queues.target(b, (5.25, 9.7), 0)
    assert queues.update([a, b], 120.0) == []
    late = queues.update([a, b], 120.5)
    assert [(f, n) for f, n, _ in late] == [('cashier', 'b')]


def test_every_queue_slot_is_reachable_from_every_seat():
    layout = json.loads((ROOT / 'config/layout.json').read_text())
    router = Router(layout)
    queues = ServiceQueues.from_file(ROOT / 'config/service_queues.json', router)
    bindings = json.loads((ROOT / 'config/seat_bindings.json').read_text())['bindings']
    for f in queues.facilities.values():
        for slot in f['slots']:
            for b in bindings:
                assert router.route(b['approach'], slot)


def test_seated_person_adds_no_walking_buffer_but_standing_does():
    node = repro.make_node('low')
    try:
        a, b = node.people[0], node.people[1]
        a.update(active=True, posture='SEATED', pos=a['binding']['center'][:])
        assert a['pos'] not in node.obstacles_for(b)
        a['posture'] = 'STANDING_UP'
        assert a['pos'] in node.obstacles_for(b)
    finally:
        node.worker.shutdown()


def test_late_timer_keeps_simulation_time():
    node = repro.make_node('low')
    try:
        set_clock(node, 0); node.tick()
        set_clock(node, 500_000_000); node.tick()   # one late 0.5 s period
        assert node.elapsed == pytest.approx(0.5)   # the old tick clipped this to 0.2
        set_clock(node, 3_500_000_000); node.tick()  # 3 s gap: at most 10 steps now
        assert node.elapsed == pytest.approx(1.5) and node.lag_seconds == pytest.approx(2.0)
        node.tick(); node.tick()                     # backlog is caught up, never dropped
        assert node.elapsed == pytest.approx(3.5) and node.lag_seconds == pytest.approx(0.0)
    finally:
        node.worker.shutdown()


def test_trajectories_do_not_depend_on_timer_jitter():
    def run(tick_times_ns):
        node = repro.make_node('low')
        try:
            for ns in tick_times_ns:
                set_clock(node, ns); node.tick()
            return node.elapsed, [(a['name'], a['pos'][:], a['z'], a['index'], a['posture']) for a in node.people]
        finally:
            node.worker.shutdown()
    regular = [i * 100_000_000 for i in range(1201)]
    jitter, t = [0], 0
    for k in range(10_000):
        t = min(t + (50_000_000, 130_000_000, 270_000_000, 90_000_000)[k % 4], 120_000_000_000)
        jitter.append(t)
        if t == 120_000_000_000:
            break
    jitter += [t] * 5  # drain any backlog
    assert run(regular) == run(jitter)


def test_failure_policy_fail_raises_and_log_records():
    node = repro.make_node('low')
    try:
        with pytest.raises(RuntimeError, match='MOTION_STALL'):
            node.fail('MOTION_STALL test')
        node.failure_policy = 'log'
        node.fail('MOTION_STALL test', node.people[0])
        assert node.failures[-1]['agent'] == node.people[0]['name']
    finally:
        node.worker.shutdown()


def test_clock_backwards_is_a_failure():
    node = repro.make_node('low')
    try:
        set_clock(node, 5_000_000_000); node.tick()
        set_clock(node, 6_000_000_000); node.tick()
        set_clock(node, 1_000_000_000)
        with pytest.raises(RuntimeError, match='CLOCK_BACKWARDS'):
            node.tick()
    finally:
        node.worker.shutdown()


def test_flush_sends_only_changed_poses_and_retries_failures():
    from concurrent.futures import Future
    from restaurant_sim.controller import Controller
    sent, replies = [], []

    class Worker:
        def submit(self, fn, request):
            sent.append(request)
            f = Future(); f.set_result(SimpleNamespace(returncode=0, stdout=replies.pop(0))); return f

    node = Controller.__new__(Controller)
    node.worker = Worker(); node.pending = None; node.dirty = {}; node.inflight = {}
    node.get_logger = lambda: SimpleNamespace(warning=lambda *a, **k: None)
    replies += ['data: true']
    node.dirty = {'a': 'A1', 'b': 'B1'}; node.flush()
    assert sent == ['A1 B1'] and node.dirty == {}
    node.flush()                       # success collected, nothing changed: no resend
    assert sent == ['A1 B1']
    replies += ['data: false']
    node.dirty = {'a': 'A2'}; node.flush()
    node.dirty['a'] = 'A3'             # newer pose arrives while A2 is in flight
    replies += ['data: true']
    node.flush()                       # A2 failed: retry, but never over the newer A3
    assert sent == ['A1 B1', 'A2', 'A3']


def test_person_does_not_appear_on_the_robot():
    node = repro.make_node('high')
    try:
        a = next(p for p in node.people if p['initial_seated'])
        node.robot = tuple(a['binding']['center'])
        set_clock(node, 0); node.tick()
        for i in range(1, int((a['arrival'] + 5) * 10)):
            set_clock(node, i * 100_000_000); node.tick()
        assert not a['active']
        node.robot = (0.5, 0.5)
        set_clock(node, (i + 1) * 100_000_000); node.tick()
        assert a['active']
    finally:
        node.worker.shutdown()


def test_ros_pose_transport_sends_moved_models_and_retries_failures():
    from concurrent.futures import Future
    from restaurant_sim.controller import Controller
    from ros_gz_interfaces.srv import SetEntityPose
    calls = []

    class Client:
        def service_is_ready(self): return True
        def call_async(self, req):
            calls.append((req.entity.name, round(req.pose.position.x, 2)))
            f = Future(); f.set_result(SimpleNamespace(success=req.entity.name != 'bad')); return f

    node = Controller.__new__(Controller)
    node.pose_transport = 'ros'; node.SetEntityPose = SetEntityPose; node.pose_client = Client()
    node.dirty = {}; node.dirty_pose = {}; node.ros_pending = {}; node.ros_inflight = {}
    node.get_logger = lambda: SimpleNamespace(warning=lambda *a, **k: None)
    node.dirty_pose = {'a': (1, 2, .95, 0), 'bad': (3, 4, .95, 0)}; node.flush()
    assert sorted(calls) == [('a', 1), ('bad', 3)]
    node.flush()                     # 'bad' failed: resent, 'a' not
    assert sorted(calls[2:]) == [('bad', 3)]


def test_robot_is_three_points_along_its_heading():
    from restaurant_sim.controller import Controller
    node = Controller.__new__(Controller)
    node.robot = None
    assert node.robot_points() == []
    node.robot, node.robot_yaw = (5.0, 5.0), 0.0
    xs = [round(p[0], 3) for p in node.robot_points()]
    assert xs == [4.75, 5.0, 5.2] and all(p[1] == 5.0 for p in node.robot_points())
