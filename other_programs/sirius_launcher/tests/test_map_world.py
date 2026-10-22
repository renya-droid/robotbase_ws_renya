"""Occupancy-map geometry must retain scale, coordinate axes and narrow walls."""

import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from PIL import Image
import yaml

from simulation_worlds import WORKSPACE, load_worlds


def generator():
    path = WORKSPACE / 'src/robotbase_sim/scripts/generate_map_world.py'
    spec = importlib.util.spec_from_file_location('generate_map_world', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_rectangles_preserve_holes_isolated_pixels_and_one_cell_walls():
    mask = np.array([[1, 1, 1, 0, 1], [1, 0, 1, 0, 0],
                     [1, 1, 1, 1, 0], [0, 0, 0, 1, 0]], dtype=bool)
    coverage = np.zeros(mask.shape, dtype=int)
    for x0, x1, y0, y1 in generator().rectangles(mask):
        coverage[y0:y1, x0:x1] += 1
    np.testing.assert_array_equal(coverage, mask.astype(int))


def test_mesh_cell_has_correct_extent_and_outward_normals(tmp_path):
    path = tmp_path / 'cell.obj'
    vertices, triangles = generator().write_mesh(np.array([[True]]), .05, 2., path)
    assert (vertices, triangles) == (8, 12)
    lines = path.read_text().splitlines()
    points = np.array([[float(v) for v in row.split()[1:]]
                       for row in lines if row.startswith('v ')])
    np.testing.assert_allclose(points.min(axis=0), [0, 0, 0])
    np.testing.assert_allclose(points.max(axis=0), [.05, .05, 2])
    center = points.mean(axis=0)
    assert len([row for row in lines if row.startswith('vn ')]) == 6
    for row in lines:
        if row.startswith('f '):
            p = points[[int(v.split('//')[0]) - 1 for v in row.split()[1:]]]
            assert np.dot(np.cross(p[1]-p[0], p[2]-p[0]), p.mean(axis=0)-center) > 0


@pytest.mark.parametrize('world_id,source_name', [
    ('map_1f_0821', '1F-0821.yaml'),
    ('map_0920_nakanoshima', '0920-nakanoshima.yaml'),
])
def test_map_world_and_spawn_match_source(world_id, source_name):
    world = next(w for w in load_worlds() if w['id'] == world_id)
    source = WORKSPACE / 'maps_waypoints/maps' / source_name
    original = yaml.safe_load(source.read_text())
    paired = yaml.safe_load(Path(world['map']).read_text())
    for key in ('origin', 'resolution', 'occupied_thresh', 'free_thresh', 'negate', 'mode'):
        assert paired[key] == original[key]
    pixels = np.array(Image.open(source.parent / original['image']))
    generated = np.array(Image.open(Path(world['map']).parent / paired['image']))
    np.testing.assert_array_equal(generated, pixels)
    x, y, _ = world['spawn']
    res = paired['resolution']
    col = int((x-paired['origin'][0])/res)
    row = generated.shape[0]-1-int((y-paired['origin'][1])/res)
    # Whole robot footprint (including its corners) fits in known free space.
    rr, cc = np.mgrid[-9:10, -9:10]
    disk = rr**2 + cc**2 <= 9**2
    assert np.all(generated[row+rr[disk], col+cc[disk]] >= 254)
    root = ET.parse(world['world']).getroot()
    model = root.find("world/model[@name='map_obstacles']")
    assert model.find('static').text == 'true'
    np.testing.assert_allclose(list(map(float, model.find('pose').text.split())),
                               [*paired['origin'][:2], 0, 0, 0, paired['origin'][2]])
    for uri in model.findall('.//mesh/uri'):
        assert (Path(world['world']).parent / uri.text).is_file()
    assert not root.findall('.//include')
