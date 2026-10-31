"""Reported opposing pedestrians and failed-refuge negative regressions."""
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace

import pytest
from builtin_interfaces.msg import Time
from restaurant_sim.controller import Controller, Router
from restaurant_sim.passage import Passage

spec=importlib.util.spec_from_file_location('issues_reproducer',Path(__file__).resolve().parents[3]/'tools/restaurant_ralph/reproduce_issues.py')
repro=importlib.util.module_from_spec(spec);spec.loader.exec_module(repro)


def test_reported_pair_retreats_passes_and_resumes_original_goals():
    node=repro.make_node('low')
    try:
        a,b=node.people[4],node.people[2]
        node.people=[a,b];node.elapsed=170
        a.update(pos=[10.265893916583138,3.45],index=2,wait=0,posture='STANDING')
        b.update(pos=[10.8,3.0],index=1,wait=0,posture='STANDING')
        for p in node.people:p.update(active=True,path=None)
        initial={p['name']:p['pos'][:] for p in node.people}
        saw_retreat=False
        for i in range(201):
            t=170+i/10;stamp=Time(sec=int(t),nanosec=(i%10)*100000000)
            node.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=170000000000+i*100000000,to_msg=lambda:stamp))
            before={p['name']:p['pos'][:] for p in node.people}
            node.tick()
            assert math.dist(a['pos'],b['pos'])>=.66-1e-9
            for p in node.people:
                sweep = node.seating.access[p['seat_id']] if p['posture'] in ('SITTING_DOWN','SEATED') else node.router
                assert sweep.segment_free(before[p['name']],p['pos'])
                assert math.dist(before[p['name']],p['pos'])<=p['speed']*.100001
            saw_retreat |= a['pos'][0]<initial[a['name']][0]-.3
        assert saw_retreat
        assert [e['event'] for e in node.passage.events]==['reserve','release']
        assert not node.passage.plans
        assert all(math.dist(p['pos'],initial[p['name']])>1.2 for p in node.people)
        assert a['index']>=2 and b['index']>=1
    finally:node.worker.shutdown()


def test_closed_corridor_cannot_create_a_refuge_and_monitor_still_fails():
    # 0.62m effective corridor is too narrow to pass at 0.66m clearance.
    router=Router({'obstacles':[dict(x=8,y=y,w=16,d=.1,yaw=0) for y in (1.35,2.65)]})
    node=Controller.__new__(Controller);node.router=router;node.robot=None;node.standing_reservations={};node.passage=Passage()
    node.get_logger=lambda:SimpleNamespace(info=lambda *a:None)
    def person(name,x,goal):
        return dict(name=name,pos=[x,2],active=True,done=False,posture='STANDING',transition=None,wait=0,index=0,steps=[('exit',goal,0)],path=None)
    # Goals lie beyond the 3 m retreat limit on both sides, so neither can
    # step back past the other's goal: the blockage is genuinely unresolvable.
    a=person('customer_00',5,(15,2));b=person('customer_01',5.7,(1,2));node.people=[a,b]
    assert not node.passage.request(node,a,0)
    assert not node.passage.plans
    from restaurant_sim.motion_monitor import MotionMonitor
    monitor=MotionMonitor();monitor.observe(a,0,node.people)
    with pytest.raises(RuntimeError,match='MOTION_STALL'):monitor.observe(a,20,node.people,'No safe refuge')


def test_reserved_refuge_is_exclusive_and_has_finite_deadline():
    node=repro.make_node('low')
    try:
        a,b=node.people[4],node.people[2];node.people=[a,b];node.elapsed=170
        a.update(pos=[10.265893916583138,3.45],index=2,wait=0,active=True)
        b.update(pos=[10.8,3.0],index=1,wait=0,active=True)
        assert node.passage.request(node,a,0)
        owner,plan=next(iter(node.passage.plans.items()))
        outsider=dict(name='new_customer',pos=[5,5],active=True,done=False)
        assert plan['refuge'] in node.obstacles_for(outsider)
        assert not node.passage.request(node,a,1)
        with pytest.raises(RuntimeError,match='PASSAGE_TIMEOUT'):node.passage.update(node,15.1)
        # A timeout is an error, not an implicit success or reservation overwrite.
        assert owner in node.passage.plans
    finally:node.worker.shutdown()


def test_single_file_corridor_resolves_by_retreating_past_the_passers_goal():
    # Same 0.62 m corridor: nobody can pass, but customer_00 can back out
    # beyond customer_01's nearby goal along a swept route.
    router=Router({'obstacles':[dict(x=8,y=y,w=16,d=.1,yaw=0) for y in (1.35,2.65)]})
    node=Controller.__new__(Controller);node.router=router;node.robot=None;node.standing_reservations={};node.passage=Passage()
    node.get_logger=lambda:SimpleNamespace(info=lambda *a:None)
    def person(name,x,goal):
        return dict(name=name,pos=[x,2],active=True,done=False,posture='STANDING',transition=None,wait=0,index=0,steps=[('exit',goal,0)],path=None)
    a=person('customer_00',5,(9,2));b=person('customer_01',5.7,(3,2));node.people=[a,b]
    assert node.passage.request(node,a,0)
    plan=node.passage.plans['customer_00']
    assert plan['passer']=='customer_01' and plan['refuge'][0] < 3-.66
    assert all(router.segment_free(p,q) for p,q in zip([a['pos']]+a['path'][:-1],a['path']))
    assert all(math.dist(p,plan['refuge'])>=.66 for p in b['path'])
