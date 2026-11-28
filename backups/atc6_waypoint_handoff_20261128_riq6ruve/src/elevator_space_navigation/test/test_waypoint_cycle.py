from types import SimpleNamespace as NS

import pytest
from std_srvs.srv import Trigger
from std_msgs.msg import String

from elevator_space_navigation.waypoint_cycle import WaypointCycleLink
from elevator_space_navigation.elevator_entry_map_diff import ElevatorEntryMapDiff
from test_auto_start_on_closed import automatic_node, closed


def ready(monkeypatch):
    h = automatic_node(monkeypatch)
    h.auto_start_on_closed = False
    h.waypoint_cycle_id = None
    h.waypoint_cycle_result = ''
    h.waypoint_cycle_armed = False
    h.arm_waypoint_cycle = WaypointCycleLink.arm_waypoint_cycle.__get__(h)
    return h


def test_waits_for_waypoint_before_closed_to_open_cycle(monkeypatch):
    h = ready(monkeypatch)
    closed(h)
    h.maybe_start_on_closed_door()
    assert not h.entry_enabled
    response = h.arm_waypoint_cycle(Trigger.Request(), Trigger.Response())
    assert response.success and response.message == h.waypoint_cycle_id
    h.maybe_start_on_closed_door()
    assert h.entry_enabled and h.auto_start_consumed and not h.waypoint_cycle_armed
    assert not h.submit_navigation(0., 1., 'entry')
    h.fresh()
    assert h.submit_navigation(0., 1., 'entry')


@pytest.mark.parametrize('fault', ['standalone', 'busy', 'inside', 'stale', 'partial', 'map_switch'])
def test_refused_request_never_arms(monkeypatch, fault):
    h = ready(monkeypatch)
    if fault == 'standalone': h.auto_start_on_closed = True
    elif fault == 'busy': h.entry_enabled = True; h.operation_finished = False
    elif fault == 'inside': h.robot_y = 1.
    elif fault == 'stale': h.now += 3.
    elif fault == 'partial': h.use_full_door_cycle = False
    else: h.switch_map_after_exit = True
    response = h.arm_waypoint_cycle(Trigger.Request(), Trigger.Response())
    assert not response.success and not h.waypoint_cycle_armed and h.waypoint_cycle_id is None


def test_cancel_disarms_closed_wait_and_requires_new_request(monkeypatch):
    h = ready(monkeypatch)
    response = h.arm_waypoint_cycle(Trigger.Request(), Trigger.Response())
    old_id = response.message
    h.request_motion_stop()
    closed(h)
    h.maybe_start_on_closed_door()
    assert not h.entry_enabled and not h.waypoint_cycle_armed
    response = h.arm_waypoint_cycle(Trigger.Request(), Trigger.Response())
    assert response.success and response.message != old_id


def test_completed_cycle_can_receive_new_correlated_request(monkeypatch):
    h = ready(monkeypatch)
    first = h.arm_waypoint_cycle(Trigger.Request(), Trigger.Response())
    closed(h); h.maybe_start_on_closed_door()
    h.operation_finished = True
    second = h.arm_waypoint_cycle(Trigger.Request(), Trigger.Response())
    assert second.success and second.message != first.message
    assert h.waypoint_cycle_result == '' and not h.entry_enabled


def test_state_reports_current_request_and_result(monkeypatch):
    h = ready(monkeypatch)
    h.arm_waypoint_cycle(Trigger.Request(), Trigger.Response())
    h.dynamic_costmap_profile = False
    h.result_pub = NS(publish=lambda _: None)
    ElevatorEntryMapDiff.publish_experiment_result(h, 'NG_DOOR_TIMEOUT,0,0,1')
    h.publish_entry_state()
    import json
    state = json.loads(h.state_pub.messages[-1].data)
    assert state['waypoint_cycle_id'] == h.waypoint_cycle_id
    assert state['waypoint_cycle_result'] == 'NG_DOOR_TIMEOUT'


def test_post_exit_map_done_preserves_launcher_completion_protocol(monkeypatch):
    import json
    h = ready(monkeypatch)
    h.arm_waypoint_cycle(Trigger.Request(), Trigger.Response())
    h.dynamic_costmap_profile = False
    h.operation_phase = 'FINISHED'; h.operation_finished = True
    h.exit_map_switched = True
    results = []
    h.result_pub = NS(publish=results.append)
    ElevatorEntryMapDiff.publish_experiment_result(h, 'MAP_SWITCH_DONE,0,0,0')
    h.publish_entry_state()
    state = json.loads(h.state_pub.messages[-1].data)
    assert state['waypoint_cycle_result'] == 'GOAL_REACHED'
    assert state['finished'] and state['phase'] == 'FINISHED' and state['map_switched']
    assert state['normal_profile_ready'] and not state['nav_unresolved']
    assert results[-1].data.startswith('MAP_SWITCH_DONE')


def test_lift_waypoint_requires_closed_then_open_before_start(monkeypatch):
    h = ready(monkeypatch)
    h.lift_cycle_enabled = True
    h.lift_start_ready = lambda: True
    h.reset_lift_cycle = lambda: None
    h.arm_waypoint_cycle(Trigger.Request(), Trigger.Response())
    h.get_door_state = lambda: ('OPEN', True, False, {})
    h.maybe_start_on_closed_door()
    assert not h.entry_enabled
    h.get_door_state = lambda: ('CLOSED', False, True, {})
    h.maybe_start_on_closed_door()
    assert h.waypoint_cycle_closed_seen and not h.entry_enabled
    h.get_door_state = lambda: ('OPEN', True, False, {})
    h.maybe_start_on_closed_door()
    assert h.entry_enabled


def test_lift_waypoint_waits_for_startup_stability_then_starts(monkeypatch):
    h = ready(monkeypatch)
    h.lift_cycle_enabled = True
    stable = [False]
    h.lift_start_ready = lambda: stable[0]
    h.reset_lift_cycle = lambda: None
    assert h.arm_waypoint_cycle(Trigger.Request(), Trigger.Response()).success
    closed(h)
    h.maybe_start_on_closed_door()
    h.fresh()
    h.maybe_start_on_closed_door()
    assert h.waypoint_cycle_armed and not h.entry_enabled
    assert not h.auto_start_consumed and not h.nav_client.sent
    assert h.statuses[-1] == 'waypoint到着・開扉確認済み：起動条件の確認・安定待ち'
    assert not any(event['event'] == 'start_refused' for event in h.events())

    stable[0] = True
    h.maybe_start_on_closed_door()
    assert h.entry_enabled and h.auto_start_consumed and not h.waypoint_cycle_armed


def test_map_switch_waypoint_requires_explicit_opt_in(monkeypatch):
    h = ready(monkeypatch)
    h.switch_map_after_exit = True
    assert not h.arm_waypoint_cycle(Trigger.Request(), Trigger.Response()).success
    h.waypoint_allow_map_switch = True
    h.departure_map_ready = lambda: True
    assert h.arm_waypoint_cycle(Trigger.Request(), Trigger.Response()).success
