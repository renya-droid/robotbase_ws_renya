"""Close-start regression and negative swept recovery checks."""
import math

import pytest

from restaurant_sim.controller import Router
from test_swept_routes import layout_router


def test_reported_close_start_has_safe_outward_connector():
    router = layout_router()
    start = (7.95, 6.618212166833715)
    blocker = (7.35, 6.45)
    assert .62 < math.dist(start, blocker) < .66
    path = router.route(start, (12.45, 7.35), [blocker])
    samples = [tuple(start[k]+(path[0][k]-start[k])*i/1000 for k in (0,1))
               for i in range(1001)]
    distances = [math.dist(p, blocker) for p in samples]
    assert all(b >= a-1e-12 for a,b in zip(distances, distances[1:]))
    assert distances[-1] >= .66
    assert all(router.free(*p) for p in samples)
    assert all(router.segment_free(a,b,[blocker]) for a,b in zip(path,path[1:]))


def test_recovery_never_closes_distance_or_crosses_another_person():
    router = Router({'obstacles': []})
    assert router.segment_free((2.63,2),(2.8,2),[(2,2)],allow_escape=True)
    assert not router.segment_free((2.63,2),(2.6,2),[(2,2)],allow_escape=True)
    assert not router.segment_free((2.63,2),(3.4,2),[(2,2),(3.4,2)],allow_escape=True)
    assert not router.segment_free((2.63,2),(2.8,2),[(2,2)])


def test_physical_overlap_cannot_be_recovered_by_teleport():
    router = Router({'obstacles': []})
    with pytest.raises(ValueError, match='No swept route'):
        router.route((2.4,2),(4,2),[(2,2)])


def test_outward_recovery_cannot_cross_wall():
    router = Router({'obstacles': [dict(x=3.1,y=2,w=.1,d=30,yaw=0)]})
    assert not router.segment_free((2.63,2),(3.6,2),[(2,2)],allow_escape=True)
