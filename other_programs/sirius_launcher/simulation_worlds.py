"""World catalogue and last-selection handoff between Gazebo and Nav2."""

import argparse
import json
import math
import os
from pathlib import Path
import sys


WORKSPACE = Path(__file__).resolve().parents[2]


def load_worlds(workspace=WORKSPACE):
    root = Path(workspace) / 'src/robotbase_sim'
    worlds = json.loads((root / 'worlds/catalog.json').read_text())
    for world in worlds:
        for key in ('world', 'map'):
            world[key] = str(root / world[key])
            if not Path(world[key]).is_file():
                raise ValueError(f"Missing {key}: {world[key]}")
    return worlds


def selection_path(workspace=WORKSPACE, domain=None):
    domain = str(domain if domain is not None else os.environ.get('ROS_DOMAIN_ID', 'unset'))
    if not domain.isdigit() or not 0 <= int(domain) <= 232:
        raise ValueError('A valid simulation ROS_DOMAIN_ID is required')
    return Path(workspace) / '.runtime' / f'sim_world_{domain}.json'


def last_selection(workspace=WORKSPACE, domain=None):
    path = selection_path(workspace, domain)
    if not path.exists():
        return None
    saved = json.loads(path.read_text())
    # Resolve against the current checkout, even if it has moved since saving.
    world = next((world for world in load_worlds(workspace)
                  if world['id'] == saved.get('world_id')), None)
    if world and 'spawn' in saved:
        world['spawn'] = saved['spawn']
    return world


def launch_arguments(world, overrides):
    defaults = {'world': world['world'], **dict(zip(
        ('spawn_x', 'spawn_y', 'spawn_yaw'), world['spawn']))}
    supplied = {arg.partition(':=')[0] for arg in overrides if ':=' in arg}
    return [f'{key}:={value}' for key, value in defaults.items() if key not in supplied] + overrides


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--list', action='store_true')
    parser.add_argument('--world-id')
    parser.add_argument('--current-map', action='store_true')
    parser.add_argument('--current-pose', action='store_true')
    args, extra = parser.parse_known_args()
    worlds = load_worlds()
    if args.list:
        for world in worlds:
            print(f"{world['id']}: {world['label']}")
        return 0
    if args.current_map or args.current_pose:
        world = last_selection()
        if world:
            print(world['map'] if args.current_map else ' '.join(map(str, world['spawn'])))
        return 0
    extra = [arg for arg in extra if arg != '--']
    custom_world = next((arg.split(':=', 1)[1] for arg in extra if arg.startswith('world:=')), None)
    if args.world_id and custom_world:
        parser.error('Use either --world-id or world:=, not both')
    selected = None
    if not custom_world:
        world_id = args.world_id
        if world_id is None and sys.stdin.isatty():
            for index, world in enumerate(worlds, 1):
                print(f"  [{index}] {world['label']}", flush=True)
            try:
                answer = input('ワールド番号 [1]: ').strip() or '1'
                if not answer.isdigit() or not 1 <= int(answer) <= len(worlds):
                    raise ValueError('一覧にあるワールド番号を入力してください')
                world_id = worlds[int(answer) - 1]['id']
            except (EOFError, ValueError) as error:
                parser.error(str(error))
        world_id = world_id or 'test_arena'
        selected = next((world for world in worlds if world['id'] == world_id), None)
        if selected is None:
            parser.error(f'Unknown world: {world_id}')
        extra = launch_arguments(selected, extra)
        # Keep Nav2's initial pose consistent with explicit spawn overrides too.
        values = dict(arg.split(':=', 1) for arg in extra if ':=' in arg)
        try:
            spawn = [float(values[f'spawn_{key}']) for key in ('x', 'y', 'yaw')]
        except ValueError:
            parser.error('Spawn pose must contain numbers')
        if not all(math.isfinite(value) for value in spawn):
            parser.error('Spawn pose must be finite')
        selected['spawn'] = spawn
        print(f"ワールド: {selected['label']}", flush=True)
    # This is the last selected world, not proof that Gazebo is running.
    state = selection_path()
    state.parent.mkdir(parents=True, exist_ok=True)
    temporary = state.with_suffix('.tmp')
    temporary.write_text(json.dumps({
        'world_id': selected['id'] if selected else None,
        'spawn': selected['spawn'] if selected else None,
    }))
    temporary.replace(state)
    os.execvp('ros2', ['ros2', 'launch', 'robotbase_sim', 'sim.launch.py', *extra])


if __name__ == '__main__':
    raise SystemExit(main())
