from concurrent.futures import Future
import json
from types import SimpleNamespace as NS

import pytest

from robotbase_waypoint.elevator_task import ElevatorTaskLink
from robotbase_waypoint.waypoint_follow import navigation_segment_end, waypoint_arrival_threshold
from robotbase_waypoint.waypoint_io import Waypoint, load_waypoints, write_waypoints


class Follower(ElevatorTaskLink):
    def __init__(self):
        self.elevator_task = None
        self.goal_records = {}
        self.finished = False
        self.shutting_down = False
        self.holding_stop = False
        self.waypoints = [Waypoint(1, 0., 0., 0., elevator=True), Waypoint(2, 1., 0., 0.)]
        self.index = 0
        self.elevator_timeout_sec = 300.
        self.requests, self.messages, self.advances = [], [], []
        self.reply = Future()
        self.elevator_client = NS(service_is_ready=lambda: True, call_async=self.request)
        self.elevator_cancel_publisher = NS(publish=lambda m: self.messages.append(m.data))

    def request(self, _):
        self.requests.append('request')
        return self.reply

    def _is_shutting_down(self):
        return self.shutting_down

    def _commit_mission(self, fn, *args):
        if self.shutting_down:
            return False
        fn(*args)
        return True

    def _commit_state(self, **changes):
        if self.shutting_down:
            return False
        for k,v in changes.items():
            setattr(self,k,v)
        return True

    def _publish_stop(self, value):
        self.messages.append(value)
        return True

    def get_logger(self):
        return NS(info=lambda _: None, error=lambda _: None)

    def get_clock(self):
        return NS(now=lambda: NS(nanoseconds=100_000_000_000))

    def _advance(self):
        self.advances.append('next')

    def _complete_or_loop(self):
        self.advances.append('done')


def running(monkeypatch):
    now=[100.]
    monkeypatch.setattr('robotbase_waypoint.elevator_task.monotonic',lambda:now[0])
    f=Follower()
    f._start_elevator()
    f._continue_elevator()
    f.reply.set_result(NS(success=True,message='new-id'))
    f._continue_elevator()
    return f,now


def packet(**changes):
    data=dict(waypoint_cycle_id='new-id',state_sequence=1,stamp_ros=100.,session_id='session',
              waypoint_cycle_result='GOAL_REACHED',finished=True,phase='FINISHED',
              nav_unresolved=False,normal_profile_ready=True,safety=dict(doorway_region='OUTSIDE'))
    data.update(changes)
    return NS(data=json.dumps(data))


def test_cycle_request_sent_once_and_waits_without_zero_or_latched_stop(monkeypatch):
    f,_=running(monkeypatch)
    for _ in range(10):
        f._continue_elevator()
    assert f.requests==['request']
    assert f.messages==[False] and not f.advances
    f._elevator_state(packet())
    assert f.advances==['next']
    f._elevator_state(packet(state_sequence=2))
    assert f.advances==['next']


@pytest.mark.parametrize('changes', [dict(waypoint_cycle_id='old-id'),dict(waypoint_cycle_result=''),
    dict(finished=False),dict(phase='WAIT_CLOSE'),dict(nav_unresolved=True),
    dict(safety=dict(doorway_region='INSIDE')),dict(normal_profile_ready=False),dict(stamp_ros=80.)])
def test_incomplete_or_old_completion_never_advances(monkeypatch,changes):
    f,_=running(monkeypatch)
    f._elevator_state(packet(**changes))
    assert not f.advances


def test_duplicate_state_does_not_extend_timeout(monkeypatch):
    f,now=running(monkeypatch)
    f._elevator_state(packet(phase='WAIT_CLOSE',finished=False))
    now[0]=105.
    f._elevator_state(packet(phase='WAIT_CLOSE',finished=False))
    f._continue_elevator()
    assert f.finished and f.elevator_task['stage']=='fault' and not f.advances
    assert True not in f.messages and f.messages[-1]=='stop'


def test_failure_never_advances_and_cancels_elevator(monkeypatch):
    f,_=running(monkeypatch)
    f._elevator_state(packet(waypoint_cycle_result='NG_DOOR_TIMEOUT'))
    assert f.finished and not f.advances and f.messages[-1]=='stop'
    assert True not in f.messages


def test_rejection_is_not_retried(monkeypatch):
    monkeypatch.setattr('robotbase_waypoint.elevator_task.monotonic',lambda:100.)
    f=Follower();f._start_elevator();f._continue_elevator()
    f.reply.set_result(NS(success=False,message='busy'))
    f._continue_elevator();f._continue_elevator()
    assert f.requests==['request'] and not f.advances and f.finished


def test_elevator_waypoint_is_a_precise_section_endpoint(tmp_path):
    route=[Waypoint(1,0.,0.,0.),Waypoint(2,1.,0.,0.,elevator=True),Waypoint(3,2.,0.,0.)]
    assert navigation_segment_end(route,0)==1
    assert waypoint_arrival_threshold(route[1],1.,.2)==.2
    p=tmp_path/'route.yaml';write_waypoints(str(p),route)
    assert load_waypoints(str(p))==route


@pytest.mark.parametrize('extra',['elevator: "true"','elevator: true\n    person_area: true','elevator: true\n    change_map: other'])
def test_invalid_task_combination_rejected(tmp_path,extra):
    p=tmp_path/'bad.yaml'
    p.write_text('waypoints:\n  - x: 0\n    y: 0\n    angle_radians: 0\n    '+extra+'\n')
    with pytest.raises(ValueError):load_waypoints(str(p))
