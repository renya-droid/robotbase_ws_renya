"""Fail explicitly when a moving agent makes insufficient progress in sim time."""
from collections import deque
import json
import math


class MotionMonitor:
    window = 20.0
    minimum_distance = 0.10

    def __init__(self):
        self.history = {}

    def observe(self, agent, now, people, route_error=None):
        name = agent['name']
        if (not agent['active'] or agent['done'] or agent['wait'] > 0
                or agent['index'] >= len(agent['steps'])):
            self.history.pop(name, None)
            return
        state, target, _ = agent['steps'][agent['index']]
        key = (agent['index'], state)
        previous = self.history.get(name)
        if previous is None or previous[0] != key:
            previous = (key, deque())
            self.history[name] = previous
        samples = previous[1]
        if samples and now < samples[-1][0]:
            raise RuntimeError('Simulation clock moved backwards')
        samples.append((now, tuple(agent['pos'])))
        # Keep the sample immediately before the window boundary as well.
        while len(samples) > 1 and samples[1][0] <= now - self.window:
            samples.popleft()
        distance = sum(math.dist(a[1], b[1]) for a, b in zip(samples, list(samples)[1:]))
        if now - samples[0][0] >= self.window and distance < self.minimum_distance:
            blockers = [dict(name=b['name'], pos=b['pos']) for b in people
                        if b is not agent and b['active'] and not b['done']
                        and math.dist(agent['pos'], b['pos']) < 0.8]
            raise RuntimeError('MOTION_STALL ' + json.dumps(dict(
                sim_time=now, name=name, position=agent['pos'], activity=state,
                goal=target, blockers=blockers, route_error=route_error,
                interval=now-samples[0][0], distance=distance), sort_keys=True))
