"""Explicit chair bindings and conservative access, without changing furniture."""
import hashlib
import json
import math


class Seating:
    cylinder_half_height = .85  # unchanged SDF customer cylinder length 1.7
    standing_z = .95  # SDF floor top .10 + cylinder half height

    def __init__(self, root, layout, router_type):
        config = json.loads((root/'config/seat_bindings.json').read_text())
        if config['layout_sha256'] != hashlib.sha256((root/'config/layout.json').read_bytes()).hexdigest():
            raise ValueError('Seat bindings do not match layout geometry')
        self.bindings = config['bindings']
        self.access = {}
        ids = set()
        for slot, b in enumerate(self.bindings):
            if b['slot'] != slot or b['seat_id'] in ids:
                raise ValueError('Duplicate or unordered seat binding')
            ids.add(b['seat_id'])
            chair = next(o for o in layout['obstacles'] if o['name'] == b['seat_id'])
            if 'chair' not in chair['name'] or math.dist(b['center'], (chair['x'],chair['y'])) > 1e-9 or abs(b['surface_z']-chair['h']) > 1e-9:
                raise ValueError('Seat geometry mismatch')
            access = router_type(dict(layout, obstacles=[o for o in layout['obstacles'] if o is not chair]))
            if not router_type(layout).free(*b['approach']) or not access.segment_free(b['approach'], b['center']):
                raise ValueError('Unsafe seat access: '+b['seat_id'])
            self.access[b['seat_id']] = access

    def seated_z(self, binding):
        # The cylinder's bottom touches the chair top; it never sinks into it.
        return binding['surface_z'] + self.cylinder_half_height
