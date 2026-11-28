"""Validate a destination mask before changing a live Keepout filter."""

import json
import math
from pathlib import Path

from PIL import Image
import yaml


def destination_keepout_enabled(map_path: Path) -> bool:
    state_path = map_path.parent / '.keepout' / map_path.stem / 'settings.json'
    if not state_path.is_file():
        return False
    return json.loads(state_path.read_text()).get('enabled') is True


def destination_keepout_mask(map_path: Path):
    state_path = map_path.parent / '.keepout' / map_path.stem / 'settings.json'
    if not state_path.is_file():
        raise ValueError(f'Destination Keepout mask is missing: {state_path}')
    state = json.loads(state_path.read_text())
    if state.get('enabled') is not True:
        raise ValueError(f'Destination Keepout is not enabled: {state_path}')
    source = Path(state.get('source_map', ''))
    relocated = (tuple(source.parts[-3:]) == tuple(map_path.parts[-3:])
                 and tuple(map_path.parts[-3:-1]) == ('maps_waypoints', 'maps'))
    if source != map_path and not relocated:
        raise ValueError('Destination Keepout belongs to a different map')
    meta = yaml.safe_load(map_path.read_text())
    with Image.open(map_path.parent / meta['image']) as image:
        size = list(image.size)
    geometry = dict(resolution=meta['resolution'], origin=meta['origin'], size=size)
    if state.get('map_metadata') != geometry:
        raise ValueError('Destination Keepout geometry does not match its map')
    mask_path = state_path.parent / 'mask.yaml'
    if not mask_path.is_file():
        raise ValueError(f'Destination Keepout mask YAML is missing: {mask_path}')
    mask_meta = yaml.safe_load(mask_path.read_text())
    if (mask_meta.get('resolution') != geometry['resolution']
            or mask_meta.get('origin') != geometry['origin']):
        raise ValueError('Destination Keepout mask metadata differs from its map')
    with Image.open(mask_path.parent / mask_meta['image']) as image:
        if list(image.size) != size:
            raise ValueError('Destination Keepout mask image size differs from its map')
    return mask_path, geometry


def mask_grid_matches(grid, geometry) -> bool:
    info = grid.info
    origin = geometry['origin']
    return (grid.header.frame_id == 'map'
            and info.width == geometry['size'][0]
            and info.height == geometry['size'][1]
            and math.isclose(info.resolution, geometry['resolution'], abs_tol=1e-7)
            and math.isclose(info.origin.position.x, origin[0], abs_tol=1e-5)
            and math.isclose(info.origin.position.y, origin[1], abs_tol=1e-5)
            and math.isclose(2 * math.atan2(info.origin.orientation.z,
                                              info.origin.orientation.w),
                             origin[2], abs_tol=1e-5)
            and len(grid.data) == info.width * info.height)
