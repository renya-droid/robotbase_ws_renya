#!/usr/bin/env python3
"""Generate the bundled 40 x 30 m facility and its geometrically matching map."""

import math
from pathlib import Path
import xml.etree.ElementTree as ET


def obstacles():
    # name, centre x/y, size x/y/z. All dimensions are metres.
    boxes = [
        ('north_wall', 0, 15.1, 40.4, .2, 2.5),
        ('south_wall', 0, -15.1, 40.4, .2, 2.5),
        ('east_wall', 20.1, 0, .2, 30, 2.5),
        ('west_wall', -20.1, 0, .2, 30, 2.5),
    ]
    # Four-metre-wide cross corridor, eight rooms, 2.4 m doorways.
    room_spans = [(-20, -11), (-11, -2), (2, 11), (11, 20)]
    for side in (-1, 1):
        for room, (left, right) in enumerate(room_spans):
            middle = (left + right) / 2
            for edge, (lo, hi) in enumerate(((left, middle - 1.2), (middle + 1.2, right))):
                boxes.append((f'door_wall_{side}_{room}_{edge}', (lo+hi)/2,
                              side*2.1, hi-lo, .2, 2.5))
            # Asymmetric furniture helps distinguish rooms during localization.
            boxes.append((f'desk_{side}_{room}', middle + (room % 2)*1.2 - .6,
                          side*(11.0 + (room % 3)*.7), 2.4, 1.2, .85))
        for column in (-11, -2.1, 2.1, 11):
            boxes.append((f'room_partition_{side}_{column}', column,
                          side*8.6, .2, 12.8, 2.5))
    return boxes


def generate(package):
    package = Path(package)
    template = ET.parse(package / 'worlds/test_arena.sdf')
    world = template.getroot().find('world')
    world.set('name', 'robotbase_indoor_facility')
    for model in list(world.findall('model')):
        if model.get('name') != 'ground':
            world.remove(model)
    for size in world.findall("model[@name='ground']/link/*/geometry/plane/size"):
        size.text = '50 40'
    for name, x, y, sx, sy, sz in obstacles():
        model = ET.SubElement(world, 'model', name=name)
        ET.SubElement(model, 'static').text = 'true'
        ET.SubElement(model, 'pose').text = f'{x} {y} {sz/2} 0 0 0'
        link = ET.SubElement(model, 'link', name='link')
        for kind in ('collision', 'visual'):
            item = ET.SubElement(link, kind, name=kind)
            geometry = ET.SubElement(item, 'geometry')
            box = ET.SubElement(geometry, 'box')
            ET.SubElement(box, 'size').text = f'{sx} {sy} {sz}'
            if kind == 'visual':
                material = ET.SubElement(item, 'material')
                color = '0.35 0.45 0.65 1' if name.startswith('desk') else '0.78 0.75 0.66 1'
                ET.SubElement(material, 'ambient').text = color
                ET.SubElement(material, 'diffuse').text = color
    ET.indent(template, space='  ')
    template.write(package / 'worlds/indoor_facility.sdf', encoding='utf-8', xml_declaration=True)

    resolution, xmin, ymin, width, height = .05, -20.5, -15.5, 820, 620
    pixels = bytearray([254]) * (width * height)
    # Mark all cells intersecting physical obstacles as occupied (conservative).
    for _name, x, y, sx, sy, _sz in obstacles():
        x0 = max(0, math.floor((x - sx/2 - xmin)/resolution + 1e-8))
        x1 = min(width, math.ceil((x + sx/2 - xmin)/resolution - 1e-8))
        y0 = max(0, math.floor((y - sy/2 - ymin)/resolution + 1e-8))
        y1 = min(height, math.ceil((y + sy/2 - ymin)/resolution - 1e-8))
        for row in range(height-y1, height-y0):
            pixels[row*width+x0:row*width+x1] = bytes(x1-x0)
    output = package / 'maps/indoor_facility'
    output.mkdir(parents=True, exist_ok=True)
    (output / 'map.pgm').write_bytes(f'P5\n{width} {height}\n255\n'.encode() + pixels)
    (output / 'map.yaml').write_text(
        'image: map.pgm\nmode: trinary\nresolution: 0.05\n'
        'origin: [-20.5, -15.5, 0.0]\nnegate: 0\n'
        'occupied_thresh: 0.65\nfree_thresh: 0.25\n')


if __name__ == '__main__':
    generate(Path(__file__).resolve().parents[1])
