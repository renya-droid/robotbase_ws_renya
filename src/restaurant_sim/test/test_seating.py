"""Seat access geometry and occupied standing-space regressions."""
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from restaurant_sim.controller import Controller, Router
from restaurant_sim.seating import Seating

ROOT=Path(__file__).resolve().parents[1]
LAYOUT=json.loads((ROOT/'config/layout.json').read_text())


def fixture_node(slot=6):
    node=Controller.__new__(Controller)
    node.seating=Seating(ROOT,LAYOUT,Router)
    b=node.seating.bindings[slot]
    a=dict(name='unit_customer',binding=b,seat_id=b['seat_id'],pos=b['approach'][:],
           z=node.seating.standing_z,yaw=b['yaw'],speed=.8,posture='STANDING',
           active=True,done=False,index=0,path=[],wait=0,transition=None)
    node.people=[a];node.robot=None;node.standing_reservations={};node.dirty={}
    return node,a


def test_all_bindings_match_sdf_and_allow_access_but_forbid_normal_walking():
    seats=Seating(ROOT,LAYOUT,Router)
    models={m.get('name'):m for m in ET.parse(ROOT/'worlds/restaurant_base.sdf').findall('.//world/model')}
    assert len(seats.bindings)==len(LAYOUT['seats'])==18
    assert len({b['seat_id'] for b in seats.bindings})==18
    router=Router(LAYOUT)
    for b in seats.bindings:
        m=models[b['seat_id']];pose=list(map(float,m.findtext('pose').split()))
        dims=list(map(float,m.findtext('.//collision/geometry/box/size').split()))
        assert math.dist(pose[:2],b['center'])<1e-9
        assert abs(pose[2]+dims[2]/2-b['surface_z'])<1e-9
        assert not router.free(*b['center'])
        router.route(LAYOUT['goals']['entry'],b['approach'])
        # Independent dense rectangle-envelope test, without production sweep.
        for i in range(501):
            p=[b['approach'][k]+(b['center'][k]-b['approach'][k])*i/500 for k in (0,1)]
            for o in LAYOUT['obstacles']:
                if o['name']==b['seat_id']:continue
                dx,dy=p[0]-o['x'],p[1]-o['y'];c,s=math.cos(o['yaw']),math.sin(o['yaw'])
                assert abs(c*dx+s*dy)>=o['w']/2+.29 or abs(-s*dx+c*dy)>=o['d']/2+.29


def test_sit_stand_are_bounded_and_touch_seat_without_penetration():
    node,a=fixture_node()
    b=a['binding']
    for sitting in (True,False):
        assert node.begin_transition(a,sitting,17)
        assert a['name'] in node.standing_reservations
        for _ in range(100):
            before=(*a['pos'],a['z'])
            node.advance_transition(a,.1)
            assert math.dist(before,(*a['pos'],a['z'])) <= .080000001
            if math.dist(a['pos'],b['approach'])>1e-9:
                assert a['z']-.85>=b['surface_z']-1e-9
            if a['transition'] is None:break
        assert a['transition'] is None
        assert not node.standing_reservations
        assert a['posture']==('SEATED' if sitting else 'STANDING')
        assert math.dist(a['pos'],b['center'] if sitting else b['approach'])<1e-9
    assert a['index']==1
    assert a['z']==node.seating.standing_z


def test_occupied_standing_space_preserves_seat_then_releases_reservation():
    node,a=fixture_node();b=a['binding']
    a.update(pos=b['center'][:],z=node.seating.seated_z(b),posture='SEATED')
    blocker=dict(name='unit_blocker',pos=b['approach'][:],active=True,done=False)
    node.people.append(blocker)
    before=(*a['pos'],a['z'])
    assert not node.begin_transition(a,False)
    assert before==(*a['pos'],a['z']) and a['posture']=='SEATED'
    assert not node.standing_reservations
    blocker['done']=True
    assert node.begin_transition(a,False)
    # Another customer must see the reserved approach, even while owner is seated.
    assert b['approach'] in node.obstacles_for(blocker)
    for _ in range(100):
        node.advance_transition(a,.1)
        if a['transition'] is None:break
    assert a['posture']=='STANDING' and not node.standing_reservations


def test_new_occupant_interrupts_transition_without_overlap():
    node,a=fixture_node()
    assert node.begin_transition(a,True,17)
    blocker=dict(name='late_blocker',pos=a['binding']['center'][:],active=True,done=False)
    node.people.append(blocker)
    before=(*a['pos'],a['z'])
    node.advance_transition(a,.1)
    assert before==(*a['pos'],a['z'])
    assert a['transition'] is not None and a['route_error'].startswith('Seat transition occupied')
