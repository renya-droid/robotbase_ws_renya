"""FIFO service queues with explicit waiting slots (cashier, water).

Without a queue every customer targeted the same service point, so the second
arrival stopped 0.66 m short of it forever. Here an agent joins the queue when
its service step starts; the head is promoted to the service point only while
the facility has free capacity, and the others wait on fixed slots. Slot
positions are assumptions stored in config/service_queues.json, not drawing
measurements. Waiting on a slot is an intentional wait, timed separately from
motion stalls (EVALUATION G3: explicit service queues within 120 sim seconds).
"""
import json
import math


class ServiceQueues:
    def __init__(self, config, router):
        self.timeout = float(config.get('max_wait_sim_seconds', 120.0))
        self.facilities = {}
        for name, f in config['facilities'].items():
            slots = [tuple(p) for p in f['slots']]
            for p in slots:
                if not router.free(*p):
                    raise ValueError(f'Queue slot {p} of {name} is inside an obstacle')
            for p, q in zip(slots, slots[1:]):
                if math.dist(p, q) < router.person_clearance:
                    raise ValueError(f'Queue slots {p} and {q} of {name} are closer than the person clearance')
            self.facilities[name] = dict(capacity=int(f['capacity']), slots=slots, waiting=[], serving=[])
        self.joined = {}
        self.events = []

    @classmethod
    def from_file(cls, path, router):
        return cls(json.loads(path.read_text()), router)

    def handles(self, state):
        return state in self.facilities

    @staticmethod
    def current_state(a):
        return a['steps'][a['index']][0] if not a['done'] and a['index'] < len(a['steps']) else None

    def update(self, people, now):
        """Drop finished users, promote the head while capacity allows, check waits."""
        by_name = {a['name']: a for a in people}
        for name, f in self.facilities.items():
            for lst in (f['serving'], f['waiting']):
                for agent in [n for n in lst if self.current_state(by_name[n]) != name]:
                    lst.remove(agent)
                    self.joined.pop((name, agent), None)
                    self.events.append(dict(event='leave', time=now, facility=name, agent=agent))
            while f['waiting'] and len(f['serving']) < f['capacity']:
                agent = f['waiting'].pop(0)
                f['serving'].append(agent)
                self.events.append(dict(event='serve', time=now, facility=name, agent=agent,
                                        waited=now - self.joined[(name, agent)]))
        late = [(name, agent, now - t) for (name, agent), t in self.joined.items()
                if agent in self.facilities[name]['waiting'] and now - t > self.timeout]
        return late

    def target(self, a, service_point, now):
        """Point the agent should walk to for its current service step."""
        name = self.current_state(a)
        f = self.facilities[name]
        if a['name'] in f['serving']:
            return tuple(service_point), False
        if a['name'] not in f['waiting']:
            f['waiting'].append(a['name'])
            self.joined[(name, a['name'])] = now
            self.events.append(dict(event='join', time=now, facility=name, agent=a['name']))
            if len(f['serving']) < f['capacity'] and f['waiting'][0] == a['name']:
                f['waiting'].pop(0)
                f['serving'].append(a['name'])
                self.events.append(dict(event='serve', time=now, facility=name, agent=a['name'], waited=0.0))
                return tuple(service_point), False
        k = f['waiting'].index(a['name'])
        # Beyond the last slot the agent heads for the tail slot and the normal
        # clearance stops it behind whoever stands there; that time still
        # counts as queue waiting, bounded by self.timeout.
        return f['slots'][min(k, len(f['slots']) - 1)], True

    def full(self, name, pending=0):
        """True when every waiting slot is taken: newcomers should stay seated.

        pending counts customers already standing up for this facility; they
        join only once upright, so without them several could rise at once."""
        f = self.facilities[name]
        return len(f['waiting']) + pending >= len(f['slots'])

    def describe(self, a):
        name = self.current_state(a)
        if name not in self.facilities:
            return None
        f = self.facilities[name]
        if a['name'] in f['serving']:
            return dict(facility=name, role='serving')
        if a['name'] in f['waiting']:
            return dict(facility=name, role='waiting', position=f['waiting'].index(a['name']))
        return None
