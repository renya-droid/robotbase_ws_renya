#!/usr/bin/env python3
"""Extrude a ROS trinary occupancy map into an exact, static Gazebo mesh."""

import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image
import yaml


def rectangles(mask):
    """Disjoint rectangles covering every true cell, with exclusive upper bounds."""
    active = {}
    result = []
    for y, row in enumerate(mask):
        edges = np.flatnonzero(np.diff(np.r_[False, row, False]))
        current = {}
        for x0, x1 in zip(edges[::2], edges[1::2]):
            previous = active.pop((x0, x1), (x0, x1, y, y))
            current[x0, x1] = (int(x0), int(x1), previous[2], y + 1)
        result.extend(active.values())
        active = current
    result.extend(active.values())
    return result


def write_mesh(mask, resolution, height, output):
    """Merge coplanar cells; emit only external faces, preserving one-cell walls."""
    vertices, faces, lookup = [], [], {}
    normals = [(-1, 0, 0), (1, 0, 0), (0, -1, 0), (0, 1, 0), (0, 0, -1), (0, 0, 1)]

    def quad(points):
        ids = []
        for x, y, z in points:
            key = (int(x), int(y), int(z))
            if key not in lookup:
                lookup[key] = len(vertices) + 1
                vertices.append((x * resolution, y * resolution, z * height))
            ids.append(lookup[key])
        a, b, c = points[:3]
        u = [b[i] - a[i] for i in range(3)]
        v = [c[i] - a[i] for i in range(3)]
        cross = (u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0])
        normal = tuple(int(np.sign(value)) for value in cross)
        ni = normals.index(normal) + 1
        faces.extend(((ids[0], ids[1], ids[2], ni), (ids[0], ids[2], ids[3], ni)))

    for x0, x1, y0, y1 in rectangles(mask):
        quad([(x0, y0, 1), (x1, y0, 1), (x1, y1, 1), (x0, y1, 1)])
        quad([(x0, y1, 0), (x1, y1, 0), (x1, y0, 0), (x0, y0, 0)])

    south = mask & ~np.pad(mask[:-1], ((1, 0), (0, 0)))
    north = mask & ~np.pad(mask[1:], ((0, 1), (0, 0)))
    west = mask & ~np.pad(mask[:, :-1], ((0, 0), (1, 0)))
    east = mask & ~np.pad(mask[:, 1:], ((0, 0), (0, 1)))
    # A boundary layer can be one cell wide on several adjacent rows. Keep each
    # row's boundary rather than treating those rows as one long horizontal face.
    for edges, top in ((south, False), (north, True)):
        for y, row in enumerate(edges):
            ends = np.flatnonzero(np.diff(np.r_[False, row, False]))
            for x0, x1 in zip(ends[::2], ends[1::2]):
                yy = y + int(top)
                points = [(x0, yy, 0), (x1, yy, 0), (x1, yy, 1), (x0, yy, 1)]
                quad(points[::-1] if top else points)
    for edges, right in ((west, False), (east, True)):
        for x, column in enumerate(edges.T):
            ends = np.flatnonzero(np.diff(np.r_[False, column, False]))
            for y0, y1 in zip(ends[::2], ends[1::2]):
                xx = x + int(right)
                points = [(xx, y0, 0), (xx, y1, 0), (xx, y1, 1), (xx, y0, 1)]
                quad(points if right else points[::-1])
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('w') as stream:
        stream.write('# Exact occupied-cell extrusion. Units: metres. Z up.\no occupied\n')
        for point in vertices:
            stream.write('v ' + ' '.join(f'{v:.6f}' for v in point) + '\n')
        # Explicit normals are required by gz-physics's DART mesh loader.
        for normal in normals:
            stream.write('vn ' + ' '.join(map(str, normal)) + '\n')
        for a, b, c, normal in faces:
            stream.write(f'f {a}//{normal} {b}//{normal} {c}//{normal}\n')
    return len(vertices), len(faces)


def generate(map_yaml, package, world_id='map_1f_0821', wall_height=2.0):
    metadata = yaml.safe_load(map_yaml.read_text())
    if metadata.get('mode', 'trinary') != 'trinary':
        raise ValueError('Only trinary occupancy maps are supported')
    image_path = map_yaml.parent / metadata['image']
    image = Image.open(image_path).convert('L')
    pixels = np.array(image)
    probability = pixels / 255.0 if metadata.get('negate', 0) else 1.0 - pixels / 255.0
    occupied = np.flipud(probability > metadata['occupied_thresh'])
    resolution = float(metadata['resolution'])
    if resolution <= 0 or wall_height <= 0 or not occupied.any():
        raise ValueError('Positive resolution/height and at least one occupied cell are required')
    map_dir = package / 'maps' / world_id
    map_dir.mkdir(parents=True, exist_ok=True)
    image.save(map_dir / 'map.png')  # Lossless; unknown/free cells are unchanged.
    generated_metadata = dict(metadata, image='map.png')
    (map_dir / 'map.yaml').write_text(yaml.safe_dump(generated_metadata, sort_keys=False))
    mesh_path = package / 'models' / world_id / 'meshes' / 'occupied.obj'
    vertex_count, triangle_count = write_mesh(occupied, resolution, wall_height, mesh_path)

    tree = ET.parse(package / 'worlds/test_arena.sdf')
    world = tree.getroot().find('world')
    world.set('name', f'robotbase_{world_id}')
    for model in list(world.findall('model')):
        if model.get('name') != 'ground':
            world.remove(model)
    ox, oy, yaw = map(float, metadata['origin'])
    width, height = image.width * resolution, image.height * resolution
    cx = ox + math.cos(yaw) * width / 2 - math.sin(yaw) * height / 2
    cy = oy + math.sin(yaw) * width / 2 + math.cos(yaw) * height / 2
    ground = world.find("model[@name='ground']")
    ET.SubElement(ground, 'pose').text = f'{cx} {cy} 0 0 0 {yaw}'
    for size in ground.findall('link/*/geometry/plane/size'):
        size.text = f'{width + 4} {height + 4}'
    model = ET.SubElement(world, 'model', name='map_obstacles')
    ET.SubElement(model, 'static').text = 'true'
    ET.SubElement(model, 'pose').text = f'{ox} {oy} 0 0 0 {yaw}'
    link = ET.SubElement(model, 'link', name='occupied_cells')
    for kind in ('collision', 'visual'):
        item = ET.SubElement(link, kind, name=f'occupied_{kind}')
        geometry = ET.SubElement(item, 'geometry')
        mesh = ET.SubElement(geometry, 'mesh')
        ET.SubElement(mesh, 'uri').text = f'../models/{world_id}/meshes/occupied.obj'
        if kind == 'visual':
            material = ET.SubElement(item, 'material')
            ET.SubElement(material, 'ambient').text = '0.64 0.67 0.70 1'
            ET.SubElement(material, 'diffuse').text = '0.64 0.67 0.70 1'
    ET.indent(tree, space='  ')
    tree.write(package / 'worlds' / f'{world_id}.sdf', encoding='utf-8', xml_declaration=True)
    manifest = {
        'source_map': map_yaml.name,
        'source_image_sha256': hashlib.sha256(image_path.read_bytes()).hexdigest(),
        'image_size': [image.width, image.height],
        'resolution': resolution, 'origin': metadata['origin'],
        'wall_height': wall_height, 'occupied_cells': int(occupied.sum()),
        'vertices': vertex_count, 'triangles': triangle_count,
        'unknown_policy': 'No invented walls in unknown cells; ground covers the map extent.',
    }
    (map_dir / 'generation.json').write_text(json.dumps(manifest, indent=2) + '\n')
    return manifest


if __name__ == '__main__':
    package = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', type=Path,
                        default=package.parents[1] / 'maps_waypoints/maps/1F-0821.yaml')
    parser.add_argument('--wall-height', type=float, default=2.0)
    parser.add_argument('--world-id', default='map_1f_0821',
                        help='Output name for the world, mesh and paired map')
    args = parser.parse_args()
    if not args.world_id or any(c not in 'abcdefghijklmnopqrstuvwxyz0123456789_' for c in args.world_id):
        parser.error('--world-id must contain only lowercase letters, numbers and underscores')
    print(json.dumps(generate(args.map, package, world_id=args.world_id,
                              wall_height=args.wall_height), indent=2))
