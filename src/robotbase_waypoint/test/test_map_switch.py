"""Map boundaries stop navigation until both map loading and localization succeed."""

from concurrent.futures import Future
from types import SimpleNamespace

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.srv import LoadMap
import pytest
from rclpy.time import Time

from robotbase_waypoint import waypoint_follow as module
from robotbase_waypoint.waypoint_follow import (
    navigation_segment_end, waypoint_arrival_threshold, WaypointFollower,
)
from robotbase_waypoint.waypoint_io import Waypoint, write_waypoints


def wp(number=1, **kwargs):
    """Return a compact route point."""
    return Waypoint(number, 2.0 * number, 0.0, 0.0, **kwargs)


@pytest.fixture
def follower(tmp_path, monkeypatch):
    """Exercise the state machine without ROS nodes or robot commands."""
    monkeypatch.setattr(module, 'monotonic', lambda: 100.0)
    node = WaypointFollower.__new__(WaypointFollower)
    node._init_shutdown_state()
    node.waypoints = [wp(change_map='next')]
    node.index = 0
    node.maps_dir = tmp_path / 'maps'
    node.waypoints_dir = tmp_path / 'waypoints'
    node.maps_dir.mkdir()
    node.waypoints_dir.mkdir()
    (node.maps_dir / 'next.yaml').write_text('image: next.pgm\n')
    write_waypoints(str(node.waypoints_dir / 'next.yaml'), [wp(3), wp(4)])
    node.loop = False
    node.navigation_only = False
    node.map_switch = None
    node.map_switch_timeout_sec = 60.0
    node.map_switch_settle_sec = 2.0
    node.map_frame = 'map'
    node.precise_threshold = 0.35
    node.goal_serial = 0
    node.goal_records = {}
    node.goal_handle = None
    node.goal_result_future = None
    node.finished = False
    node.holding_stop = False
    node.events = []
    node.get_node_names = lambda: ['amcl', 'map_server']
    node.get_clock = lambda: SimpleNamespace(now=lambda: Time(seconds=10))
    node.get_logger = lambda: SimpleNamespace(
        info=lambda *args: None, error=lambda *args: None)
    node._publish_stop = lambda value: node.events.append(('stop', value)) or True
    node._publish_safety_zero = lambda: node.events.append(('zero', None))
    node._configure_person_search = lambda: node.events.append(('camera', None))
    node._send_current_goal = lambda: node.events.append(('goal', node.index))
    node.initial_pose_publisher = SimpleNamespace(
        publish=lambda pose: node.events.append(('pose', pose)))
    node.route_markers = SimpleNamespace(
        message=None, publish=lambda: node.events.append(('markers', None)))
    node.load_future = Future()
    node.map_client = SimpleNamespace(
        service_is_ready=lambda: True,
        call_async=lambda request: (
            node.events.append(('load', request.map_url)) or node.load_future))
    return node


def start_loading(node, monkeypatch):
    """Finish zero settling and dispatch one map request."""
    node._start_map_switch()
    monkeypatch.setattr(module, 'monotonic', lambda: 101.0)
    node._continue_map_switch()


def test_boundary_splits_continuous_route_and_uses_precise_threshold():
    """Both navigation modes treat the map switch as a stopping task."""
    route = [wp(), wp(2, change_map='next'), wp(3)]
    assert navigation_segment_end(route, 0) == 1
    assert navigation_segment_end(route, 0, navigation_only=True) == 1
    assert waypoint_arrival_threshold(route[1], 1.0, 0.35) == 0.35


def test_success_waits_for_fresh_localization_before_restarting(follower, monkeypatch):
    """Never send the destination route before AMCL confirms the new pose."""
    start_loading(follower, monkeypatch)
    assert follower.map_switch['stage'] == 'loading'
    assert not any(event[0] == 'pose' for event in follower.events)
    follower.load_future.set_result(LoadMap.Response(result=0))
    follower._continue_map_switch()
    initial = next(value for event, value in follower.events if event == 'pose')
    assert initial.pose.pose.position.x == 6.0
    monkeypatch.setattr(module, 'monotonic', lambda: 104.0)
    follower._continue_map_switch()
    assert not any(event[0] == 'goal' for event in follower.events)
    estimate = PoseWithCovarianceStamped()
    estimate.header.frame_id = 'map'
    estimate.pose = initial.pose
    estimate.header.stamp = Time(seconds=9).to_msg()
    follower._map_switch_amcl_pose(estimate)
    assert not follower.map_switch['pose_confirmed']
    estimate.header.stamp = Time(seconds=11).to_msg()
    follower._map_switch_amcl_pose(estimate)
    follower._continue_map_switch()
    assert follower.map_switch is None
    assert follower.index == 0
    assert follower.waypoints[0].number == 3
    assert follower.events[-1] == ('goal', 0)
    assert follower.holding_stop is False
    assert follower.route_markers.message is not None


@pytest.mark.parametrize(
    'problem', ['keepout', 'slam', 'missing_map', 'missing_route', 'bad_route'])
def test_preflight_failure_never_calls_map_service(follower, problem):
    """Leave the current map untouched if destination validation fails."""
    if problem == 'keepout':
        follower.get_node_names = lambda: ['amcl', 'filter_mask_server']
    elif problem == 'slam':
        follower.get_node_names = lambda: ['slam_toolbox']
    elif problem == 'missing_map':
        (follower.maps_dir / 'next.yaml').unlink()
    elif problem == 'missing_route':
        (follower.waypoints_dir / 'next.yaml').unlink()
    else:
        (follower.waypoints_dir / 'next.yaml').write_text('waypoints: []\n')
    follower._start_map_switch()
    assert follower.finished and follower.holding_stop
    assert follower.map_switch['stage'] == 'fault'
    assert not any(event[0] == 'load' for event in follower.events)


def test_load_failure_keeps_original_route_and_holds_stop(follower, monkeypatch):
    """Inspect the LoadMap response code, even when the transport succeeded."""
    start_loading(follower, monkeypatch)
    follower.load_future.set_result(LoadMap.Response(result=3))
    follower._continue_map_switch()
    assert follower.map_switch['stage'] == 'fault'
    assert follower.waypoints[0].change_map == 'next'
    assert not any(event[0] in ('pose', 'goal') for event in follower.events)


@pytest.mark.parametrize('complete_load', [False, True])
def test_timeout_and_late_reply_cannot_resume(follower, monkeypatch, complete_load):
    """Service and AMCL timeouts latch a stop, including late service replies."""
    start_loading(follower, monkeypatch)
    if complete_load:
        follower.load_future.set_result(LoadMap.Response(result=0))
        follower._continue_map_switch()
    monkeypatch.setattr(module, 'monotonic', lambda: 161.0)
    follower._continue_map_switch()
    if not complete_load:
        follower.load_future.set_result(LoadMap.Response(result=0))
    follower._continue_map_switch()
    assert follower.finished and follower.holding_stop
    assert not any(event[0] == 'goal' for event in follower.events)


def test_shutdown_prevents_map_request(follower, monkeypatch):
    """An interrupted transition cannot mutate the map or start navigation."""
    follower._start_map_switch()
    follower.request_shutdown('test interrupt')
    monkeypatch.setattr(module, 'monotonic', lambda: 101.0)
    follower._continue_map_switch()
    assert not any(event[0] == 'load' for event in follower.events)


def test_wait_finishes_with_map_switch_instead_of_advancing(follower, monkeypatch):
    """A wait on a boundary runs before switching rather than skipping it."""
    follower.waypoints = [wp(wait_time=1.0, change_map='next')]
    follower.transition_in_progress = False
    follower.active_cancel_transition_generation = None
    follower.person_area_cancel_started_at = None
    follower._finish_arrival(1.0, True)
    assert follower.waiting_until is not None
    assert follower.map_switch is None
    follower.manual_paused = False
    follower.person_area_stop_pending = False
    follower.person_search_waiting = False
    follower.person_search_faulted = False
    follower.get_clock = lambda: SimpleNamespace(now=lambda: Time(seconds=12))
    follower._timer_callback()
    assert follower.map_switch['stage'] == 'ready'
    assert not any(event[0] == 'goal' for event in follower.events)


def test_map_request_waits_for_confirmed_goal_cancellation(follower):
    """Entering the arrival radius alone cannot switch an active Nav2 map."""
    from action_msgs.msg import GoalInfo, GoalStatus

    follower.navigation_only = False
    follower.stop_duration = 5.0
    follower.cancel_transition_generation = 0
    follower.person_search_faulted = False
    follower.goal_handle = object()
    follower.goal_result_future = Future()
    cancel = Future()
    identity = GoalInfo()
    identity.goal_id.uuid = list(range(16))
    record = SimpleNamespace(goal_id=identity.goal_id)
    follower._record_for_handle = lambda handle: record
    follower._ensure_cancel_requested = lambda record: cancel
    follower._arrive_at_waypoint(exact_arrival=False)
    assert follower.transition_in_progress
    assert follower.map_switch is None
    cancel.set_result(SimpleNamespace(return_code=0, goals_canceling=[identity]))
    assert follower.map_switch is None
    follower.goal_result_future.set_result(SimpleNamespace(status=GoalStatus.STATUS_CANCELED))
    assert follower.map_switch['stage'] == 'ready'
    assert not any(event[0] == 'load' for event in follower.events)


@pytest.mark.parametrize('stage', ['loading', 'localizing'])
def test_shutdown_blocks_late_map_or_localization_results(follower, monkeypatch, stage):
    """Late replies after Ctrl-C cannot publish poses or destination goals."""
    start_loading(follower, monkeypatch)
    if stage == 'localizing':
        follower.load_future.set_result(LoadMap.Response(result=0))
        follower._continue_map_switch()
        follower.map_switch['pose_confirmed'] = True
    follower.request_shutdown('test interrupt')
    before = len(follower.events)
    if stage == 'loading':
        follower.load_future.set_result(LoadMap.Response(result=0))
    monkeypatch.setattr(module, 'monotonic', lambda: 105.0)
    follower._continue_map_switch()
    assert follower.events[before:] == []
