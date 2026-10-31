"""Historical evidence must survive plan mutation and terminal tick failures."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest
from builtin_interfaces.msg import Time

spec = importlib.util.spec_from_file_location(
    'issue_diagnostics_reproducer', Path(__file__).resolve().parents[3] /
    'tools/restaurant_ralph/reproduce_issues.py')
repro = importlib.util.module_from_spec(spec)
spec.loader.exec_module(repro)


def test_historical_plan_and_route_survive_later_mutation():
    node = repro.make_node('low')
    try:
        a = node.people[0]
        a['path'] = [[8, 7]]
        node.passage.plans[a['name']] = {'phase': 'retreat', 'refuge': [8, 6]}
        records = repro.capture_tick(node, 1)
        node.passage.plans[a['name']]['phase'] = 'hold'
        node.passage.plans[a['name']]['refuge'][0] = 9
        a['path'][0][0] = 10
        assert records[0]['passage'] == {'phase': 'retreat', 'refuge': [8, 6]}
        assert records[0]['path'] == [[8, 7]]
        assert records[0]['pos'] == a['pos']
    finally:
        node.worker.shutdown()


def test_failure_tick_is_preserved_between_one_second_samples(monkeypatch):
    node = repro.make_node('low')
    calls = []
    def failing_tick():
        calls.append(True)
        if len(calls) == 3:
            node.people[0]['route_error'] = 'injected closed passage'
            raise RuntimeError('MOTION_STALL injected')
    node.tick = failing_tick
    monkeypatch.setattr(repro, 'make_node', lambda scenario: node)
    with pytest.raises(repro.LogicFailure) as failure:
        repro.run('low', 1)
    records = failure.value.records
    assert records[-1]['sim_time'] == .2
    assert len(records) == 2 * len(node.people)
    assert records[len(node.people)]['route_error'] == 'injected closed passage'
    assert 'MOTION_STALL' in str(failure.value)


def test_snapshot_reports_dwell_and_step_without_aliasing_or_pose_change():
    node = repro.make_node('low')
    try:
        a = node.people[0]
        a.update(index=1, wait=4.5, blocked=1.2, route_error='occupied cashier')
        node.elapsed = 208.7
        snapshot = node.snapshot(Time(sec=210, nanosec=100000000))
        a['wait'] = 0
        assert snapshot['stamp_ns'] == 210100000000
        assert snapshot['elapsed_sim_seconds'] == 208.7
        agent = snapshot['agents'][0]
        assert agent['diagnostics'] == dict(step_index=1, wait_remaining_sim_seconds=4.5,
                                            blocked_sim_seconds=1.2, route_error='occupied cashier')
        assert [agent['position'][k] for k in ('x', 'y', 'z')] == [*a['pos'], a['z']]
        assert agent['id'] == a['name'] and agent['seat_id'] == a['seat_id']
    finally:
        node.worker.shutdown()
