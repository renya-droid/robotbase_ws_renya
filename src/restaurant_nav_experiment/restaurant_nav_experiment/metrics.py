"""Pure metric helpers (no ROS), unit-tested."""
import math

from restaurant_nav_experiment.geometry import ROBOT_HALF_WIDTH, ROBOT_LENGTH_BACK, ROBOT_LENGTH_FRONT

PERSON_RADIUS = 0.23  # restaurant_sim cylinder


def footprint_gap(robot_xy, robot_yaw, person_xy, person_radius=PERSON_RADIUS):
    """Distance from the robot rectangle to the person's cylinder surface (m).

    Negative values mean the shapes overlap."""
    dx, dy = person_xy[0]-robot_xy[0], person_xy[1]-robot_xy[1]
    c, s = math.cos(robot_yaw), math.sin(robot_yaw)
    px, py = c*dx + s*dy, -s*dx + c*dy
    ox = max(-ROBOT_LENGTH_BACK - px, 0.0, px - ROBOT_LENGTH_FRONT)
    oy = max(abs(py) - ROBOT_HALF_WIDTH, 0.0)
    if ox == 0.0 and oy == 0.0:  # centre inside the rectangle
        inside = min(px + ROBOT_LENGTH_BACK, ROBOT_LENGTH_FRONT - px, ROBOT_HALF_WIDTH - abs(py))
        return -inside - person_radius
    return math.hypot(ox, oy) - person_radius


class LegStats:
    """Accumulates one navigation leg from 10 Hz samples."""
    NEAR_MISS = 0.20   # m footprint-to-person gap
    STOPPED = 0.05     # m/s
    HUMAN_NEAR = 1.20  # m robot centre to person centre

    def __init__(self, leg, slot, seat_id, start_sim):
        self.row = dict(leg=leg, slot=slot, seat_id=seat_id, start_sim=round(start_sim, 2),
                        end_sim=None, duration=None, result=None, path_length=0.0,
                        robot_stopped_s=0.0, min_gap_any=math.inf, min_gap_walking=math.inf,
                        near_miss_events=0, contact_samples=0, human_stopped_near_robot_s=0.0,
                        seat_occupied_at_end=None)
        self.last_xy = None
        self.in_near_miss = False

    def sample(self, dt, robot_xy, robot_yaw, robot_speed, agents, stopped_humans_near):
        r = self.row
        if self.last_xy is not None:
            r['path_length'] += math.dist(self.last_xy, robot_xy)
        self.last_xy = robot_xy
        if abs(robot_speed) < self.STOPPED:
            r['robot_stopped_s'] += dt
        gaps_walk = []
        for a in agents:
            g = footprint_gap(robot_xy, robot_yaw, a['xy'])
            r['min_gap_any'] = min(r['min_gap_any'], g)
            if a['walking']:
                gaps_walk.append(g)
        if gaps_walk:
            g = min(gaps_walk)
            r['min_gap_walking'] = min(r['min_gap_walking'], g)
            near = g < self.NEAR_MISS
            if near and not self.in_near_miss:
                r['near_miss_events'] += 1
            self.in_near_miss = near
            if g < 0:
                r['contact_samples'] += 1
        r['human_stopped_near_robot_s'] += dt * stopped_humans_near

    def finish(self, end_sim, result, seat_occupied):
        r = self.row
        r.update(end_sim=round(end_sim, 2), duration=round(end_sim - r['start_sim'], 2), result=result,
                 seat_occupied_at_end=seat_occupied)
        for k in ('path_length', 'robot_stopped_s', 'human_stopped_near_robot_s'):
            r[k] = round(r[k], 3)
        for k in ('min_gap_any', 'min_gap_walking'):
            r[k] = None if math.isinf(r[k]) else round(r[k], 3)
        return r
