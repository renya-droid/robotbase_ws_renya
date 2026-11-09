"""Regression for real low/seed42 corner stall; independent dense sampling."""
import json
import math
from pathlib import Path
import pytest
from restaurant_sim.controller import Router


def layout_router():
    return Router(json.loads((Path(__file__).parents[1]/'config/layout.json').read_text()))


def test_reported_stall_segment_cuts_furniture_despite_free_endpoints():
    router = layout_router()
    start, end = (9.75, 4.134106083416862), (9.75, 4.05)
    assert router.free(*start) and router.free(*end)
    assert not router.segment_free(start, end)
    # Check the real geometry independently of the slab implementation.
    assert any(not router.free(9.75, start[1]+(end[1]-start[1])*i/1000)
               for i in range(1001))
    target = (12.45, 7.35)
    path = router.route(start, target)
    for a, b in zip([start]+path, path):
        assert router.segment_free(a, b)
        assert all(router.free(*(a[k]+(b[k]-a[k])*i/100 for k in (0,1)))
                   for i in range(101))
    assert path[-1] == target


def test_all_real_routes_have_safe_exact_endpoint_connections_and_edges():
    router = layout_router()
    for seat in router.layout['seats']:
        for target in router.layout['goals'].values():
            path = router.route(seat, target)
            assert all(router.segment_free(a,b) for a,b in zip([seat]+path,path))


def test_dynamic_sweep_checks_between_endpoints():
    router = Router({'obstacles': []})
    assert not router.segment_free((1,2),(3,2),[(2,2)])
    assert router.segment_free((1,3),(3,3),[(2,2)])
    path = router.route((1,2),(3,2),[(2,2)])
    for a,b in zip([(1,2)]+path,path):
        dx,dy=b[0]-a[0],b[1]-a[1]
        t=max(0,min(1,((2-a[0])*dx+(2-a[1])*dy)/(dx*dx+dy*dy))) if (dx or dy) else 0
        assert math.dist((a[0]+t*dx,a[1]+t*dy),(2,2)) >= .66


def test_closed_corridor_fails_instead_of_crossing_wall():
    router = Router({'obstacles':[dict(x=8,y=6,w=.1,d=30,yaw=0)]})
    with pytest.raises(ValueError, match='No swept route'):
        router.route((7,6),(9,6))


def test_blocked_start_is_diagnosed():
    router = Router({'obstacles':[dict(x=1,y=1,w=.5,d=.5,yaw=0)]})
    with pytest.raises(ValueError, match='Blocked start'):
        router.route((1,1),(3,3))
