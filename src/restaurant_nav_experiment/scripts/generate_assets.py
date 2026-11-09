#!/usr/bin/env python3
"""Generate the robot-ready restaurant worlds, Nav2 map, base and serving goals.

Run from the workspace root; outputs go to src/restaurant_nav_experiment/generated
and are checked by tests. Inputs are never modified:
  src/restaurant_sim/config/layout.json, seat_bindings.json, service_queues.json
  src/restaurant_sim/worlds/restaurant_<scenario>.sdf
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from restaurant_nav_experiment import geometry as G  # noqa: E402

FLOOR_OLD = "<model name='floor'><static>true</static><pose>8.5 6 0.05 0 0 0</pose>"
FLOOR_NEW = "<model name='floor'><static>true</static><pose>8.5 6 -0.05 0 0 0</pose>"
# Serving goal search around each bound chair (assumed, not from the drawing).
GOAL_DIST = np.arange(.70, 1.45, .05)
GOAL_ANGLES = np.radians(np.arange(0, 360, 10))
MIN_GOAL_CLEARANCE = .10   # m between robot rectangle and furniture
BASE_CLEARANCE = .15
# The robot must be able to turn in place at the base and at a serving pose
# (circumscribed radius 0.58 m + 0.05 m): facing a chair 0.10 m away it could
# not turn back and the return leg timed out in the no-people check.
ROTATE_CLEARANCE = math.hypot(G.ROBOT_LENGTH_BACK, G.ROBOT_HALF_WIDTH) + .05


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_worlds(sim, out):
    made = {}
    for scenario in ('base', 'low', 'medium', 'high'):
        src = sim/'worlds'/f'restaurant_{scenario}.sdf'
        text = src.read_text()
        if text.count(FLOOR_OLD) != 1:
            raise SystemExit(f'Unexpected floor model in {src}')
        # The source floor top is z=0.10 while robots spawn at z=0.01: lower the
        # floor so its top is z=0. Furniture already stands on z=0; people
        # cylinders keep their z (standing bottom 0.10 m above the floor).
        dst = out/'worlds'/f'restaurant_robot_{scenario}.sdf'
        dst.write_text(text.replace(FLOOR_OLD, FLOOR_NEW))
        made[scenario] = dict(source=str(src.relative_to(sim.parents[1])), source_sha256=sha(src))
    return made


def centre_distance(dist, xs, ys, x, y):
    i, j = int((y-ys[0])/(ys[1]-ys[0])+.5), int((x-xs[0])/(xs[1]-xs[0])+.5)
    return dist[i, j] if 0 <= i < dist.shape[0] and 0 <= j < dist.shape[1] else 0.0


def choose_base(layout, dist, xs, ys, people_points):
    """East side of the kitchen pass-through counter, facing east (yaw 0)."""
    best = None
    for x in np.arange(6.75, 7.50, .05):
        for y in np.arange(2.0, 5.0, .05):
            clear = G.footprint_clearance(dist, xs, ys, x, y, 0.0)
            if clear < BASE_CLEARANCE or centre_distance(dist, xs, ys, x, y) < ROTATE_CLEARANCE:
                continue
            away = min(math.dist((x, y), p) for p in people_points)
            score = (round(away, 2), -abs(x-6.85))
            if best is None or score > best[0]:
                best = (score, (round(float(x), 2), round(float(y), 2)), clear)
    if best is None:
        raise SystemExit('No base pose with clearance east of the counter')
    return best[1], best[2]


def serving_goals(bindings, tables, dist, xs, ys, reach):
    goals = []
    def cell(x, y):
        return int((y-ys[0])/(ys[1]-ys[0])+.5), int((x-xs[0])/(xs[1]-xs[0])+.5)
    for b in bindings:
        cx, cy = b['center']
        cands = []
        for r in GOAL_DIST:
            for a in GOAL_ANGLES:
                x, y = cx + r*math.cos(a), cy + r*math.sin(a)
                yaw = math.atan2(cy-y, cx-x)  # face the customer
                clear = G.footprint_clearance(dist, xs, ys, x, y, yaw)
                if clear < MIN_GOAL_CLEARANCE or centre_distance(dist, xs, ys, x, y) < ROTATE_CLEARANCE:
                    continue
                i, j = cell(x, y)
                if not (0 <= i < reach.shape[0] and 0 <= j < reach.shape[1]) or not reach[i, j]:
                    continue
                cands.append((round(r, 2), -clear, x, y, yaw, clear))
        if not cands:
            goals.append(dict(seat_id=b['seat_id'], slot=b['slot'], reachable=False))
            continue
        r, _, x, y, yaw, clear = min(cands)
        goals.append(dict(seat_id=b['seat_id'], slot=b['slot'], reachable=True,
                          world=[round(x, 3), round(y, 3)], yaw=round(yaw, 4),
                          distance_to_chair=r, clearance=round(clear, 3)))
    return goals


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--workspace', default='.')
    args = p.parse_args()
    ws = Path(args.workspace).resolve()
    sim = ws/'src/restaurant_sim'
    out = ws/'src/restaurant_nav_experiment/generated'
    (out/'worlds').mkdir(parents=True, exist_ok=True); (out/'maps').mkdir(parents=True, exist_ok=True)
    layout = G.load_layout(sim/'config/layout.json')
    bindings = json.loads((sim/'config/seat_bindings.json').read_text())['bindings']
    queues = json.loads((sim/'config/service_queues.json').read_text())

    occ, xs, ys = G.occupancy(layout)
    dist = G.distance_field(occ)
    people_points = [tuple(v) for v in layout['goals'].values()]
    people_points += [tuple(s) for f in queues['facilities'].values() for s in f['slots']]
    base, base_clear = choose_base(layout, dist, xs, ys, people_points)
    bi, bj = int((base[1]-ys[0])/G.RESOLUTION+.5), int((base[0]-xs[0])/G.RESOLUTION+.5)
    reach = G.reachable(dist, (bi, bj), G.ROBOT_HALF_WIDTH + .05)

    # Nav2 map in the base-origin map frame (PGM rows run top-down).
    pgm = np.where(occ, 0, 254).astype(np.uint8)[::-1]
    h, w = pgm.shape
    (out/'maps/restaurant.pgm').write_bytes(f'P5\n{w} {h}\n255\n'.encode() + pgm.tobytes())
    origin = G.world_to_map(G.WORLD_MIN, base)
    (out/'maps/restaurant.yaml').write_text(
        'image: restaurant.pgm\nmode: trinary\n'
        f'resolution: {G.RESOLUTION}\norigin: [{origin[0]:.3f}, {origin[1]:.3f}, 0.0]\n'
        'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')

    goals = serving_goals(bindings, None, dist, xs, ys, reach)
    worlds = write_worlds(sim, out)
    meta = dict(
        schema_version=1,
        note=('Generated by scripts/generate_assets.py. Map frame origin = robot base; '
              'map = world - base (yaw 0). Base and serving goals are assumptions.'),
        base_world=list(base), base_yaw=0.0, base_clearance=round(float(base_clear), 3),
        robot_footprint=[[G.ROBOT_LENGTH_FRONT, G.ROBOT_HALF_WIDTH], [-G.ROBOT_LENGTH_BACK, -G.ROBOT_HALF_WIDTH]],
        inputs=dict(layout_sha256=sha(sim/'config/layout.json'),
                    seat_bindings_sha256=sha(sim/'config/seat_bindings.json')),
        worlds=worlds, goals=goals)
    (out/'experiment.json').write_text(json.dumps(meta, indent=1))
    ok = sum(g['reachable'] for g in goals)
    print(f'base {base} clearance {base_clear:.2f} m; reachable serving goals {ok}/{len(goals)}')
    for g in goals:
        print(g)


if __name__ == '__main__':
    main()
