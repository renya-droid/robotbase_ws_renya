import json
from pathlib import Path
from restaurant_sim.controller import Router

def test_all_seats_reach_services():
    layout=json.loads((Path(__file__).parents[1]/'config/layout.json').read_text())
    router=Router(layout)
    for seat in layout['seats']:
        for target in layout['goals'].values():
            path=router.route(seat,target)
            assert path
            assert all(router.free(*point) for point in path)

def test_route_avoids_stationary_person():
    import math
    router=Router({'obstacles':[]})
    person=(2.0,2.0)
    path=router.route((1.0,2.0),(3.0,2.0),[person])
    assert all(math.dist(p,person)>=.66 for p in path)
    assert any(abs(p[1]-2.0)>.5 for p in path)
