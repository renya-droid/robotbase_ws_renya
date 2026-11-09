"""Finite, swept refuge reservations for pedestrians in a mutual blockage.

No agent is exempt from collision checks. A reservation only exists when both
its retreat and the other pedestrian's subsequent passage have safe routes.
"""
import heapq
import math


class Passage:
    max_retreat_length = 3.0  # m along the swept grid route, bends allowed

    def __init__(self):
        self.plans = {}
        self.events = []
        self.requested = {}

    def reserved(self, name):
        return [p['refuge'] for owner, p in self.plans.items() if owner != name]

    def busy(self, name):
        return name in self.plans or any(p['passer'] == name for p in self.plans.values())

    @staticmethod
    def moving(a):
        return (a['active'] and not a['done'] and a['posture'] == 'STANDING'
                and a['transition'] is None and a['wait'] <= 0
                and a['index'] < len(a['steps']))

    def request(self, node, agent, now):
        self.requested.setdefault(agent['name'], now)
        if self.busy(agent['name']):
            return False
        nearby = [b for b in node.people if b is not agent and self.moving(b)
                  and not self.busy(b['name']) and math.dist(agent['pos'], b['pos']) < 1.1]
        for other in sorted(nearby, key=lambda b: (self.requested.get(b['name'], now), b['name'])):
            self.requested.setdefault(other['name'], now)
            # Oldest request gets first attempt to pass; infeasible retreats
            # reverse the roles. Requests survive failed attempts (no age reset).
            ordered = sorted((agent, other), key=lambda b: (self.requested[b['name']], b['name']))
            for passer, yielding in (ordered, ordered[::-1]):
                if self._reserve(node, yielding, passer, now):
                    return True
        return False

    def retreats(self, router, start, dynamic):
        """Grid cells reachable from start within max_retreat_length, with paths.

        One bounded Dijkstra replaces per-candidate A* calls; every edge uses
        the same swept furniture/person checks as normal walking."""
        r = router.resolution
        x, y = (round(v/r) for v in start)
        point = lambda c: (c[0]*r, c[1]*r)
        cost, prev, heap = {}, {}, []
        for c in [(x+dx, y+dy) for dx in range(-4, 5) for dy in range(-4, 5)]:
            d = math.dist(start, point(c))
            if (d <= 4.5*r and all(math.dist(point(c), p) >= router.person_clearance for p in dynamic)
                    and router.segment_free(start, point(c), dynamic, allow_escape=True)):
                cost[c] = d; prev[c] = None; heapq.heappush(heap, (d, c))
        while heap:
            g, u = heapq.heappop(heap)
            if g != cost[u]:
                continue
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                v = (u[0]+dx, u[1]+dy); nv = g + r
                if nv > self.max_retreat_length or nv >= cost.get(v, 1e9):
                    continue
                if router.segment_free(point(u), point(v), dynamic):
                    cost[v] = nv; prev[v] = u; heapq.heappush(heap, (nv, v))
        def path(c):
            out = []
            while c is not None:
                out.append(point(c)); c = prev[c]
            return out[::-1]
        return [(g, point(c), path(c)) for c, g in cost.items()]

    def _reserve(self, node, yielding, passer, now):
        router = node.router
        start = yielding['pos']
        others = [b['pos'] for b in node.people if b is not passer and b is not yielding and node.blocks_walking(b)]
        others += list(node.standing_reservations.values()) + self.reserved(passer['name'])
        others += node.robot_points() if hasattr(node,'robot_points') else ([node.robot] if node.robot is not None else [])
        goal = node.current_goal(passer)
        # The passer's route if the yielding agent were gone: a refuge on it
        # would only move the blockage, so such refuges are skipped up front.
        try:
            free_path = router.route(passer['pos'], goal, others)
        except ValueError:
            return False
        candidates = []
        for length, refuge, retreat in self.retreats(router, start, node.obstacles_for(yielding)):
            if math.dist(start, refuge) < .30 or math.dist(refuge, passer['pos']) < .9:
                continue
            if min(math.dist(refuge, q) for q in free_path) < router.person_clearance + .05:
                continue
            candidates.append((length, refuge, retreat))
        candidates.sort(key=lambda c: (c[0], c[1]))
        for checked, (length, refuge, retreat) in enumerate(candidates):
            if checked >= 24:
                break
            try:
                path = router.route(passer['pos'], goal, others + [refuge])
            except ValueError:
                continue
            self.plans[yielding['name']] = dict(passer=passer['name'],refuge=refuge,
                conflict=tuple(start),created=now,deadline=now+15.,phase='retreat')
            yielding['path']=retreat;yielding['route_error']=None
            passer['path']=path;passer['route_error']=None
            event=dict(event='reserve',time=now,yielding=yielding['name'],passer=passer['name'],refuge=refuge,
                       conflict=tuple(start),retreat_length=round(length,3))
            self.events.append(event);node.get_logger().info('PASSAGE '+str(event))
            return True
        return False

    def update(self,node,now,on_timeout=None):
        """on_timeout(message, agent) may record a timeout instead of raising;
        the expired plan is then dropped so both agents re-plan."""
        for name,plan in list(self.plans.items()):
            a=next(p for p in node.people if p['name']==name)
            b=next(p for p in node.people if p['name']==plan['passer'])
            if now > plan['deadline']:
                message='PASSAGE_TIMEOUT '+str(dict(yielding=name,**plan))
                if on_timeout is None:raise RuntimeError(message)
                on_timeout(message,a)
                del self.plans[name];a['path']=None;b['path']=None
                self.requested.pop(name,None);self.requested.pop(b['name'],None)
                self.events.append(dict(event='expire',time=now,yielding=name,passer=b['name']))
                continue
            if math.dist(a['pos'],plan['refuge']) < 1e-6:
                plan['phase']='holding'
            # Released once the passer has left the conflict, has reached its
            # goal near it, or is queueing behind others (no longer passing).
            arrived = b.get('path') == [] and b['wait'] <= 0
            arrived |= bool(b.get('queue_waiting')) and b.get('blocked', 0) >= 1.0
            if (plan['phase']=='holding' and (b['done'] or arrived or math.dist(b['pos'],plan['conflict'])>=1.2)):
                del self.plans[name];a['path']=None
                self.requested.pop(name,None);self.requested.pop(b['name'],None)
                event=dict(event='release',time=now,yielding=name,passer=b['name'])
                self.events.append(event);node.get_logger().info('PASSAGE '+str(event))

    def holding(self, name):
        return name in self.plans and self.plans[name]['phase']=='holding'
