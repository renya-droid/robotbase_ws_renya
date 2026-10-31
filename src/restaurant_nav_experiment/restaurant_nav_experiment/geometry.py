"""Shared geometry for the restaurant navigation experiment.

Frames
------
world: restaurant_sim coordinates (layout.json, people snapshot).
map:   Nav2 frame. Its origin is the robot base (spawn) pose, because the
       Gazebo DiffDrive odometry starts at the spawn pose and Nav2's
       localization:=static publishes map->odom as identity. Spawn yaw is 0,
       so map = world - base.
"""
import json
import math
from pathlib import Path

import numpy as np

ROBOT_LENGTH_FRONT = 0.425   # params/sim/nav2.yaml footprint
ROBOT_LENGTH_BACK = 0.475
ROBOT_HALF_WIDTH = 0.33
RESOLUTION = 0.05
WORLD_MIN = (-1.0, -1.0)
WORLD_MAX = (18.0, 13.5)
ROOM = (0.0, 0.0, 17.0, 11.5)  # outer wall centre lines


def load_layout(path):
    return json.loads(Path(path).read_text())


def rect_corners(x, y, w, d, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c*px - s*py, y + s*px + c*py)
            for px, py in ((w/2, d/2), (-w/2, d/2), (-w/2, -d/2), (w/2, -d/2))]


def occupancy(layout, resolution=RESOLUTION):
    """Boolean grid (row 0 = WORLD_MIN y) of furniture, walls and outdoors."""
    nx = int(round((WORLD_MAX[0]-WORLD_MIN[0])/resolution))
    ny = int(round((WORLD_MAX[1]-WORLD_MIN[1])/resolution))
    xs = WORLD_MIN[0] + (np.arange(nx)+.5)*resolution
    ys = WORLD_MIN[1] + (np.arange(ny)+.5)*resolution
    X, Y = np.meshgrid(xs, ys)
    occ = np.zeros((ny, nx), bool)
    for o in layout['obstacles']:
        c, s = math.cos(o['yaw']), math.sin(o['yaw'])
        dx, dy = X-o['x'], Y-o['y']
        occ |= (np.abs(c*dx+s*dy) <= o['w']/2) & (np.abs(-s*dx+c*dy) <= o['d']/2)
    # Outdoors (beyond the wall centre lines, including the street behind the
    # entrance gap) is not part of the serving area.
    half = .08
    occ |= (X < ROOM[0]-half) | (X > ROOM[2]+half) | (Y < ROOM[1]-half) | (Y > ROOM[3]+half)
    return occ, xs, ys


def distance_field(occ, resolution=RESOLUTION):
    """Euclidean distance (m) from each cell centre to the nearest occupied cell."""
    from scipy import ndimage
    return ndimage.distance_transform_edt(~occ) * resolution


def footprint_clearance(dist, xs, ys, x, y, yaw, samples=7):
    """Minimum obstacle distance over points of the robot rectangle."""
    c, s = math.cos(yaw), math.sin(yaw)
    best = float('inf')
    for px in np.linspace(-ROBOT_LENGTH_BACK, ROBOT_LENGTH_FRONT, samples):
        for py in np.linspace(-ROBOT_HALF_WIDTH, ROBOT_HALF_WIDTH, 5):
            wx, wy = x + c*px - s*py, y + s*px + c*py
            i = int((wy-ys[0])/(ys[1]-ys[0]) + .5); j = int((wx-xs[0])/(xs[1]-xs[0]) + .5)
            if not (0 <= i < len(ys) and 0 <= j < len(xs)):
                return 0.0
            best = min(best, dist[i, j])
    return best


def reachable(dist, start_ij, radius):
    """Cells reachable from start for a disc of the given radius (4-connected)."""
    from scipy import ndimage
    free = dist > radius
    labels, _ = ndimage.label(free)
    lab = labels[start_ij]
    return labels == lab if lab else np.zeros_like(free)


def world_to_map(p, base):
    return (p[0]-base[0], p[1]-base[1])


def map_to_world(p, base):
    return (p[0]+base[0], p[1]+base[1])
