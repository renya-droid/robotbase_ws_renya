"""Run the production tick with a deterministic clock, without Gazebo transport.

This records logic-only evidence, never Gazebo validation or a gate pass.
"""
import argparse
from copy import deepcopy
import json
import math
from pathlib import Path
from types import SimpleNamespace, MethodType
from unittest.mock import patch

from builtin_interfaces.msg import Time
from restaurant_sim.controller import Controller


class LogicFailure(RuntimeError):
    def __init__(self, message, records):
        super().__init__(message)
        self.records = records


def make_node(scenario):
    root = Path(__file__).resolve().parents[2] / 'src/restaurant_sim'
    node = Controller.__new__(Controller)
    node.declare_parameter = lambda *args: None
    node.get_parameter = lambda key: SimpleNamespace(value=scenario if key == 'scenario' else 1.0)
    node.create_subscription = lambda *args: None
    node.create_timer = lambda *args: None
    node.create_publisher = lambda *args: SimpleNamespace(publish=lambda msg: None)
    node.get_logger = lambda: SimpleNamespace(info=lambda *a: None, warning=lambda *a, **k: None, error=lambda *a: None)
    node.odom = lambda *args: None
    node.move_model = MethodType(Controller.move_model, node)
    node.tick = MethodType(Controller.tick, node)
    node.flush = lambda: None
    with patch('restaurant_sim.controller.Node.__init__'), patch('restaurant_sim.controller.get_package_share_directory', return_value=str(root)):
        Controller.__init__(node)
    return node


def capture_tick(node, sim_time):
    """Detach mutable plans from future ticks, including on a failing tick."""
    return [dict(sim_time=sim_time, name=a['name'], pos=a['pos'][:],
                 index=a['index'], wait=a['wait'], blocked=a['blocked'],
                 active=a['active'], done=a['done'], z=a['z'], yaw=a['yaw'],
                 seat_id=a['seat_id'], posture=a['posture'],
                 elapsed=node.elapsed, route_error=a.get('route_error'),
                 path=deepcopy(a['path']), transition=deepcopy(a['transition']),
                 passage=deepcopy(node.passage.plans.get(a['name'])))
            for a in node.people]


def run(scenario, seconds, record_every_tick=False, events=None):
    node = make_node(scenario)
    records = []
    try:
        for i in range(int(seconds * 10) + 1):
            stamp = Time(sec=i // 10, nanosec=(i % 10) * 100000000)
            node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=i * 100000000, to_msg=lambda: stamp))
            try:
                node.tick()
            except RuntimeError:
                records.extend(capture_tick(node, i / 10))
                raise
            if record_every_tick or i % 10 == 0:
                records.extend(capture_tick(node, i / 10))
    except RuntimeError as error:
        raise LogicFailure(str(error), records) from error
    finally:
        if events is not None:events.extend(node.passage.events)
        node.worker.shutdown()
    return records


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--scenario', default='low')
    p.add_argument('--seconds', type=float, default=240)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--record-every-tick', action='store_true')
    args = p.parse_args()
    failure = None
    events = []
    try:
        records = run(args.scenario, args.seconds, args.record_every_tick, events)
    except LogicFailure as error:
        records, failure = error.records, str(error)
    count = len({r['name'] for r in records})
    summary = {'kind': 'logic_only', 'seed': 42, 'scenario': args.scenario,
               'requested_sim_seconds': args.seconds, 'failure': failure,
               'passage_events': events,
               'max_blocked_sim_seconds': max((r['blocked'] for r in records), default=0),
               'final': records[-count:] if count else []}
    summary['all_done'] = bool(count) and all(r['done'] for r in summary['final'])
    args.output.write_text(json.dumps(dict(summary, records=records), indent=2))
    print(json.dumps(summary, indent=2))
    if failure or not summary['all_done']:
        raise SystemExit(1)
