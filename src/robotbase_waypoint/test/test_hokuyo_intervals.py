"""Recorded exclusion intervals follow arrival progress in both navigation modes."""

from types import SimpleNamespace

import pytest

from robotbase_waypoint.waypoint_follow import (
    GoalRecord, hokuyo_disabled_at, hokuyo_switch_indices, ShutdownRequest, WaypointFollower,
)
from robotbase_waypoint.waypoint_io import Waypoint


def route():
    """Nonsequential waypoint numbers must still count two list entries ahead."""
    return [Waypoint(number=n * 10, x=n, y=0, angle_radians=0,
                     hokuyo_disable_for=2 if n == 1 else 0) for n in range(5)]


def follower():
    node = WaypointFollower.__new__(WaypointFollower)
    node._init_shutdown_state(ShutdownRequest())
    node.waypoints = route()
    node.index = 1
    node.hokuyo_reached_index = 0
    node.finished = node.manual_paused = node.person_search_faulted = False
    node.states = []
    node.cues = []
    node.hokuyo_disable_publisher = SimpleNamespace(
        publish=lambda msg: node.states.append(msg.data))
    node.hokuyo_voice = SimpleNamespace(
        play=lambda disabled: node.cues.append(disabled), check=lambda: None)
    node.get_logger = lambda: SimpleNamespace(info=lambda *_: None)
    return node


def test_interval_starts_at_marker_and_ends_at_second_next_arrival():
    assert [hokuyo_disabled_at(route(), i) for i in range(-1, 5)] == [
        False, False, True, True, False, False]


def test_overlap_extends_interval_and_short_route_restores_at_last_point():
    waypoints = route()
    waypoints[2] = Waypoint(99, 2, 0, 0, hokuyo_disable_for=2)
    assert [hokuyo_disabled_at(waypoints, i) for i in range(5)] == [
        False, True, True, True, False]
    assert not hokuyo_disabled_at(waypoints[:2], 1)
    assert hokuyo_switch_indices(waypoints) == {1, 4}
    assert hokuyo_switch_indices(waypoints[:2]) == set()


def test_to_pose_advance_activates_marker_and_restores_after_two_points():
    node = follower()
    node._send_current_goal = node._publish_hokuyo_state
    node._publish_hokuyo_state()
    node._advance()
    node._advance()
    node._advance()
    assert node.states == [False, True, True, False]
    assert node.cues == [True, False]


def test_through_feedback_uses_remaining_goals_and_rejects_stale_feedback():
    node = follower()
    node.goal_serial = 3
    node.goal_records[3] = GoalRecord(start_index=0, end_index=4)
    for remaining in (3, 2, 1):
        node._goal_feedback(SimpleNamespace(feedback=SimpleNamespace(
            number_of_poses_remaining=remaining)), 3)
    assert node.states == [True, True, False]
    assert node.cues == [True, False]
    node._goal_feedback(SimpleNamespace(feedback=SimpleNamespace(
        number_of_poses_remaining=3)), 2)
    assert node.states == [True, True, False]


def test_endpoint_restores_before_waiting_and_loop_restarts_with_hokuyo_enabled():
    node = follower()
    node.index = 3
    node.hokuyo_reached_index = 2
    node.waypoints[3] = Waypoint(30, 3, 0, 0, wait_time=10)
    node._finish_arrival = lambda *_: None
    node._publish_hokuyo_state()
    node._arrive_at_waypoint(exact_arrival=True)
    assert node.states == [True, False]
    assert node.cues == [True, False]
    node.loop = True
    node._send_current_goal = node._publish_hokuyo_state
    node._complete_or_loop()
    assert node.hokuyo_reached_index == -1
    assert node.states[-1] is False


@pytest.mark.parametrize('condition', [
    'manual_paused', 'finished', 'person_search_faulted',
    'map_switch', 'elevator_task', 'shutdown'])
def test_interruption_restores_hokuyo(condition):
    node = follower()
    node.hokuyo_reached_index = 1
    node._publish_hokuyo_state()
    if condition == 'shutdown':
        node.request_shutdown('test')
    else:
        setattr(node, condition, True)
    node._publish_hokuyo_state()
    assert node.states == [True, False]
    assert node.cues == [True, False]
