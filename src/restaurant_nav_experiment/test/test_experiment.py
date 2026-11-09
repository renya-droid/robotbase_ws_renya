import importlib.util
import json
import math
from pathlib import Path

import numpy as np
import pytest

from restaurant_nav_experiment import geometry as G
from restaurant_nav_experiment.metrics import LegStats, footprint_gap

PKG = Path(__file__).resolve().parents[1]
WS = PKG.parents[1]
META = json.loads((PKG/'generated/experiment.json').read_text())


def test_footprint_gap_front_side_and_inside():
    assert footprint_gap((0, 0), 0, (1.0, 0)) == pytest.approx(1.0 - .425 - .23)
    assert footprint_gap((0, 0), math.pi/2, (0, 1.0)) == pytest.approx(1.0 - .425 - .23)
    assert footprint_gap((0, 0), 0, (0, 1.0)) == pytest.approx(1.0 - .33 - .23)
    assert footprint_gap((0, 0), 0, (0, 0)) < -.23


def test_leg_counts_near_miss_entries_not_samples():
    leg = LegStats('to_table', 1, 's', 0.0)
    walk = lambda d: [dict(xy=(.425 + .23 + d, 0), walking=True)]
    for d in (.5, .1, .1, .5, .1):
        leg.sample(.1, (0, 0), 0, .3, walk(d), 0)
    row = leg.finish(1.0, 'SUCCEEDED', True)
    assert row['near_miss_events'] == 2 and row['min_gap_walking'] == pytest.approx(.1)


def test_split_agents_by_posture_and_frame():
    spec = importlib.util.spec_from_file_location('bridge', PKG/'restaurant_nav_experiment/people_bridge.py')
    bridge = importlib.util.module_from_spec(spec); spec.loader.exec_module(bridge)
    snap = dict(agents=[
        dict(active=True, posture='SEATED', position=dict(x=10, y=5), yaw=0),
        dict(active=True, posture='STANDING', position=dict(x=7, y=4), yaw=1),
        dict(active=True, posture='SITTING_DOWN', position=dict(x=8, y=4), yaw=1),
        dict(active=False, posture='STANDING', position=dict(x=0, y=0), yaw=0)])
    everyone, walking, seated = bridge.split_agents(snap, (6.9, 3.1))
    assert len(everyone) == 3 and len(walking) == 2 and len(seated) == 1
    assert seated[0][:2] == pytest.approx((3.1, 1.9))


def test_generated_map_matches_layout_and_base_frame():
    layout = G.load_layout(WS/'src/restaurant_sim/config/layout.json')
    occ, xs, ys = G.occupancy(layout)
    raw = (PKG/'generated/maps/restaurant.pgm').read_bytes()
    header, data = raw.split(b'\n', 3)[:3], raw.split(b'\n', 3)[3]
    w, h = map(int, header[1].split())
    pgm = np.frombuffer(data, np.uint8).reshape(h, w)[::-1]
    assert (pgm == 0).tolist() == occ.tolist()
    yaml_text = (PKG/'generated/maps/restaurant.yaml').read_text()
    bx, by = META['base_world']
    assert f'origin: [{G.WORLD_MIN[0]-bx:.3f}, {G.WORLD_MIN[1]-by:.3f}, 0.0]' in yaml_text
    import hashlib
    assert META['inputs']['layout_sha256'] == hashlib.sha256(
        (WS/'src/restaurant_sim/config/layout.json').read_bytes()).hexdigest()


def test_base_and_goals_have_robot_clearance():
    layout = G.load_layout(WS/'src/restaurant_sim/config/layout.json')
    occ, xs, ys = G.occupancy(layout); dist = G.distance_field(occ)
    assert G.footprint_clearance(dist, xs, ys, *META['base_world'], 0.0) >= .15
    goals = [g for g in META['goals'] if g['reachable']]
    assert len(goals) >= 10
    for g in goals:
        assert G.footprint_clearance(dist, xs, ys, *g['world'], g['yaw']) >= .10


def test_worlds_only_lower_the_floor():
    for scenario, info in META['worlds'].items():
        src = (WS/info['source']).read_text()
        dst = (PKG/f'generated/worlds/restaurant_robot_{scenario}.sdf').read_text()
        assert dst == src.replace("<pose>8.5 6 0.05 0 0 0</pose>", "<pose>8.5 6 -0.05 0 0 0</pose>", 1)
        assert "<world name='restaurant'>" in dst


def test_condition_layers():
    spec = importlib.util.spec_from_file_location('exp_launch', PKG/'launch/experiment.launch.py')
    launch = importlib.util.module_from_spec(spec); spec.loader.exec_module(launch)
    vals = dict(all_radius='1.0', all_cost='220', walking_radius='1.0', walking_cost='220',
                seated_radius='0.0', seated_cost='120', people_timeout='1.5', bt_server_timeout_ms='200', local_static='true')
    base = WS/'params/sim/nav2.yaml'
    import yaml
    for cond, expected in (('B0', []), ('B1', ['people_layer']), ('P', ['people_walking_layer'])):
        path, layers = launch.nav2_params(base, cond, vals.get)
        g = yaml.safe_load(Path(path).read_text())['global_costmap']['global_costmap']['ros__parameters']
        assert [p for p in g['plugins'] if 'people' in p] == expected
        assert yaml.safe_load(Path(path).read_text())['bt_navigator']['ros__parameters']['default_server_timeout'] == 200
        assert yaml.safe_load(Path(path).read_text())['local_costmap']['local_costmap']['ros__parameters']['plugins'][0] == 'static_layer'
        Path(path).unlink()
    vals['seated_radius'] = '0.4'
    path, layers = launch.nav2_params(base, 'P', vals.get)
    assert list(layers) == ['people_walking_layer', 'people_seated_layer']
    Path(path).unlink()
    with pytest.raises(ValueError):
        launch.nav2_params(base, 'X', vals.get)


def test_ground_truth_picks_world_pose_over_relative_links():
    from geometry_msgs.msg import TransformStamped
    from tf2_msgs.msg import TFMessage
    spec = importlib.util.spec_from_file_location('gt', PKG/'restaurant_nav_experiment/ground_truth_tf.py')
    gt = importlib.util.module_from_spec(spec); spec.loader.exec_module(gt)
    def tf(x, y, name=''):
        t = TransformStamped(); t.child_frame_id = name
        t.transform.translation.x, t.transform.translation.y = x, y; return t
    node = gt.GroundTruthTF.__new__(gt.GroundTruthTF); node.model = 'robot'
    links = [tf(0, .2175), tf(.3, 0), tf(0, 0)]
    assert node.pick(TFMessage(transforms=links + [tf(6.9, 3.1)])).transform.translation.x == 6.9
    assert node.pick(TFMessage(transforms=links)) is None
    assert node.pick(TFMessage(transforms=[tf(6.9, 3.1), tf(8, 8, 'robot')])).transform.translation.x == 8
