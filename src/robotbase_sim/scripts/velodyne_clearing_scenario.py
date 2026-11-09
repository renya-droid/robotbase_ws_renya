#!/usr/bin/env python3
"""Place, move and remove obstacles and measure residual Velodyne costmap cells.

Timeline (simulation time) is fixed so runs are comparable.  Obstacles are
moved with the Gazebo /world/<world>/set_pose service (bridged to ROS).  The
commanded pose is the ground truth.  The robot is expected to stay at its
spawn pose (0, 0, 0), so the costmap frame (<prefix>/odom) equals the world
frame; the node checks this from /odom and records the robot pose.

For every costmap message the lethal cells are classified with the ground
truth at the costmap stamp:
  present  : within present_margin of an obstacle footprint now
  recent   : within present_margin of a footprint during the last
             recent_window seconds (sensor/costmap latency of moving objects)
  residual : every other lethal cell (cost that should have been cleared)

Outputs in output_dir: timeline.csv, events.jsonl, summary.json.
"""

import csv
import json
import math
import os
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

PARK = {
    'pedestrian': (60.0, 60.0, 0.0),
    'car': (70.0, 60.0, 0.0),
    'low_box': (75.0, 60.0, 0.0),
    'reference_wall': (80.0, 60.0, 0.0),
}
# Footprint half sizes (x, y) in the model frame and visual height.
SHAPE = {
    'pedestrian': (0.22, 0.22, 1.72),
    'car': (2.2, 0.9, 1.5),
    'low_box': (0.3, 0.3, 0.5),
    'reference_wall': (0.1, 5.0, 2.0),
}
SHAPE_COLOR = {
    'pedestrian': (0.95, 0.45, 0.1),
    'car': (0.85, 0.1, 0.1),
    'low_box': (0.95, 0.85, 0.1),
    'reference_wall': (0.7, 0.7, 0.75),
}


def static(x, y, yaw=0.0):
    return lambda _t: (x, y, yaw)


def linear(start, end, duration, yaw):
    def pose(t):
        a = max(0.0, min(1.0, t / duration))
        return (start[0] + a * (end[0] - start[0]), start[1] + a * (end[1] - start[1]), yaw)
    return pose


def build_timeline():
    walk_d = 10.0 / 1.3
    car_d = 30.0 / 4.0
    return [
        ('warmup', 4.0, {}),
        ('pedestrian_near_static', 5.0, {'pedestrian': static(2.5, 0.0)}),
        ('pedestrian_near_removed', 10.0, {}),
        ('pedestrian_far_static', 5.0, {'pedestrian': static(6.0, 1.0)}),
        ('pedestrian_far_removed', 10.0, {}),
        ('pedestrian_walk_x4', walk_d,
         {'pedestrian': linear((4.0, -5.0), (4.0, 5.0), walk_d, math.pi / 2)}),
        ('pedestrian_walk_removed', 10.0, {}),
        ('car_pass_x9', car_d, {'car': linear((9.0, -15.0), (9.0, 15.0), car_d, math.pi / 2)}),
        ('car_pass_removed', 10.0, {}),
        ('low_box_static', 5.0, {'low_box': static(1.0, -0.6)}),
        ('low_box_removed', 10.0, {}),
        ('indoor_ref_static', 5.0,
         {'pedestrian': static(3.0, 0.0), 'reference_wall': static(8.0, 0.0)}),
        ('indoor_ref_pedestrian_removed', 10.0, {'reference_wall': static(8.0, 0.0)}),
        ('wall_removed', 10.0, {}),
    ]


def footprint_mask(xs, ys, pose, half, margin):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = xs - x, ys - y
    lx = c * dx + s * dy
    ly = -s * dx + c * dy
    return (np.abs(lx) <= half[0] + margin) & (np.abs(ly) <= half[1] + margin)


class ClearingScenario(Node):

    def __init__(self):
        super().__init__('velodyne_clearing_scenario')
        p = self.declare_parameter
        self.world = p('world_name', 'velodyne_clearing_study').value
        self.frame = p('frame_id', 'robot/odom').value
        self.costmap_topic = p('costmap_topic', '/velodyne_study/costmap').value
        self.start_delay = float(p('start_delay', 3.0).value)
        self.margin = float(p('present_margin', 0.30).value)
        self.recent_window = float(p('recent_window', 0.6).value)
        self.loop = bool(p('loop', False).value)
        self.shutdown_when_done = bool(p('shutdown_when_done', False).value)
        out = p('output_dir', '').value or os.path.join(
            os.path.expanduser('~'), '.ros', 'velodyne_clearing_study',
            time.strftime('%Y%m%d_%H%M%S'))
        self.output_dir = os.path.expanduser(out)
        os.makedirs(self.output_dir, exist_ok=True)

        self.timeline = build_timeline()
        self.total = sum(d for _, d, _ in self.timeline)
        self.t0 = None
        self.history = []  # (sim_t, {name: pose}) for the recent window
        self.commanded = dict(PARK)
        self.pending = {}
        self.model_state = {}
        self.robot_pose = None
        self.robot_max_offset = 0.0
        self.phase_stats = {}
        self.done = False
        self.cycle = 0

        self.client = self.create_client(SetEntityPose, f'/world/{self.world}/set_pose')
        self.pub_gt = self.create_publisher(MarkerArray, '/velodyne_study/ground_truth', 2)
        self.pub_res = self.create_publisher(Marker, '/velodyne_study/residual_cells', 2)
        self.pub_phase = self.create_publisher(String, '/velodyne_study/phase', 2)
        self.create_subscription(OccupancyGrid, self.costmap_topic, self.on_costmap, 2)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_timer(0.05, self.tick)

        self.csv_file = open(os.path.join(self.output_dir, 'timeline.csv'), 'w', newline='')
        self.csv = csv.writer(self.csv_file)
        self.csv.writerow(['sim_time', 'scenario_time', 'cycle', 'phase', 'robot_x', 'robot_y',
                           'robot_yaw', 'active_obstacles', 'lethal', 'present', 'recent',
                           'residual', 'residual_max_dist', 'residual_centroid_x',
                           'residual_centroid_y'])
        self.events = open(os.path.join(self.output_dir, 'events.jsonl'), 'w')
        self.get_logger().info(f'output: {self.output_dir}  timeline {self.total:.1f} s')

    # ----------------------------------------------------------------- helpers
    def sim_now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def log_event(self, **kw):
        self.events.write(json.dumps(kw, ensure_ascii=False) + '\n')
        self.events.flush()

    def phase_at(self, st):
        """Return (cycle, index, name, phase_t, poses) for scenario time st."""
        cycle = int(st // self.total) if self.loop else 0
        t = st - cycle * self.total if self.loop else st
        acc = 0.0
        for i, (name, dur, objs) in enumerate(self.timeline):
            if t < acc + dur:
                local = t - acc
                return cycle, i, name, local, {k: f(local) for k, f in objs.items()}
            acc += dur
        return cycle, None, 'finished', 0.0, {}

    def on_odom(self, msg):
        p = msg.pose.pose
        yaw = 2.0 * math.atan2(p.orientation.z, p.orientation.w)
        self.robot_pose = (p.position.x, p.position.y, yaw)
        self.robot_max_offset = max(self.robot_max_offset, math.hypot(p.position.x, p.position.y))

    # -------------------------------------------------------------- simulation
    def tick(self):
        now = self.sim_now()
        if now <= 0.0:
            return
        if self.t0 is None:
            if not self.client.service_is_ready():
                self.get_logger().info('waiting for set_pose service', throttle_duration_sec=5.0)
                return
            self.t0 = now + self.start_delay
            self.log_event(event='start', sim_time=self.t0)
            for name, pose in PARK.items():
                self.send_pose(name, pose, now)
            return
        st = now - self.t0
        if st < 0.0:
            return
        cycle, idx, phase, _, poses = self.phase_at(st)
        if idx is None:
            if not self.done:
                self.finish(now)
            return
        if cycle != self.cycle:
            self.cycle = cycle
        active = {name: poses.get(name, PARK[name]) for name in PARK}
        for name, pose in active.items():
            if any(abs(a - b) > 1e-4 for a, b in zip(pose, self.commanded[name])):
                self.send_pose(name, pose, now)
        self.history.append((now, {k: v for k, v in poses.items()}))
        while self.history and self.history[0][0] < now - self.recent_window - 1.0:
            self.history.pop(0)
        self.pub_phase.publish(String(data=phase))
        self.publish_ground_truth(poses)

    def send_pose(self, name, pose, now):
        if self.pending.get(name):
            return  # keep at most one request in flight per model
        req = SetEntityPose.Request()
        req.entity = Entity(name=name, type=Entity.MODEL)
        req.pose.position.x, req.pose.position.y = float(pose[0]), float(pose[1])
        req.pose.position.z = 0.0
        req.pose.orientation.z = math.sin(pose[2] / 2.0)
        req.pose.orientation.w = math.cos(pose[2] / 2.0)
        self.pending[name] = True
        self.commanded[name] = pose
        future = self.client.call_async(req)
        parked = abs(pose[0] - PARK[name][0]) < 1e-6 and abs(pose[1] - PARK[name][1]) < 1e-6

        def done(fut, name=name, pose=pose, sent=now, parked=parked):
            self.pending[name] = False
            ok = bool(fut.result() and fut.result().success)
            if not ok:
                self.commanded[name] = (math.nan, math.nan, math.nan)  # force resend
                self.get_logger().warn(f'set_pose failed for {name}')
            state = 'parked' if parked else 'placed'
            # Log only state changes (placed <-> parked) and failures, not every motion step.
            if not ok or self.model_state.get(name) != state:
                self.model_state[name] = state
                self.log_event(event='set_pose', model=name, state=state, sim_time_sent=sent,
                               sim_time_done=self.sim_now(), x=pose[0], y=pose[1],
                               yaw=pose[2], success=ok)
        future.add_done_callback(done)

    # -------------------------------------------------------------- evaluation
    def on_costmap(self, msg):
        if self.t0 is None or self.done:
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        st = stamp - self.t0
        if st < 0.0:
            return
        cycle, idx, phase, phase_t, poses = self.phase_at(st)
        if idx is None:
            return
        info = msg.info
        grid = np.asarray(msg.data, dtype=np.int16).reshape(info.height, info.width)
        rows, cols = np.nonzero(grid >= 100)
        res = info.resolution
        xs = info.origin.position.x + (cols + 0.5) * res
        ys = info.origin.position.y + (rows + 0.5) * res
        present = np.zeros(len(xs), dtype=bool)
        for name, pose in poses.items():
            present |= footprint_mask(xs, ys, pose, SHAPE[name][:2], self.margin)
        recent = np.zeros(len(xs), dtype=bool)
        for t_hist, hist_poses in self.history:
            if stamp - self.recent_window <= t_hist <= stamp:
                for name, pose in hist_poses.items():
                    recent |= footprint_mask(xs, ys, pose, SHAPE[name][:2], self.margin)
        recent &= ~present
        residual = ~(present | recent)
        rx, ry = xs[residual], ys[residual]
        n_res = int(residual.sum())
        rp = self.robot_pose or (math.nan, math.nan, math.nan)
        self.csv.writerow([
            f'{stamp:.3f}', f'{st:.3f}', cycle, phase, f'{rp[0]:.3f}', f'{rp[1]:.3f}',
            f'{rp[2]:.3f}', '|'.join(sorted(poses)), len(xs), int(present.sum()),
            int(recent.sum()), n_res,
            f'{float(np.max(np.hypot(rx, ry))):.2f}' if n_res else '',
            f'{float(rx.mean()):.2f}' if n_res else '', f'{float(ry.mean()):.2f}' if n_res else '',
        ])
        self.csv_file.flush()

        key = f'{cycle}:{phase}'
        s = self.phase_stats.setdefault(key, {
            'phase': phase, 'cycle': cycle, 'samples': 0, 'max_residual': 0,
            'first_residual': None, 'last_residual': None, 'first_zero_time': None,
            'max_present': 0})
        s['samples'] += 1
        s['max_residual'] = max(s['max_residual'], n_res)
        s['max_present'] = max(s['max_present'], int(present.sum()))
        if s['first_residual'] is None:
            s['first_residual'] = n_res
        s['last_residual'] = n_res
        s['last_phase_time'] = round(phase_t, 3)
        if n_res == 0 and s['first_zero_time'] is None:
            s['first_zero_time'] = round(phase_t, 3)

        m = Marker(header=msg.header, ns='residual', id=0, type=Marker.CUBE_LIST)
        m.scale.x = m.scale.y = res
        m.scale.z = 0.4
        m.pose.orientation.w = 1.0
        m.pose.position.z = 0.2
        m.color = ColorRGBA(r=1.0, g=0.0, b=1.0, a=0.8)
        m.points = [Point(x=float(a), y=float(b)) for a, b in zip(rx, ry)]
        if not m.points:
            m.action = Marker.DELETE
        self.pub_res.publish(m)

    def publish_ground_truth(self, poses):
        markers = []
        for i, name in enumerate(PARK):
            m = Marker(ns='ground_truth', id=i)
            m.header.frame_id = self.frame
            if name not in poses:
                m.action = Marker.DELETE
                markers.append(m)
                continue
            x, y, yaw = poses[name]
            hx, hy, h = SHAPE[name]
            m.type = Marker.CYLINDER if name == 'pedestrian' else Marker.CUBE
            m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, h / 2.0
            m.pose.orientation.z, m.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            m.scale.x, m.scale.y, m.scale.z = 2 * hx, 2 * hy, h
            rgb = SHAPE_COLOR[name]
            m.color = ColorRGBA(r=rgb[0], g=rgb[1], b=rgb[2], a=0.35)
            markers.append(m)
        self.pub_gt.publish(MarkerArray(markers=markers))

    def finish(self, now):
        self.done = True
        for name, pose in PARK.items():
            self.send_pose(name, pose, now)
        summary = {
            'world': self.world,
            'timeline': [{'phase': n, 'duration': d, 'obstacles': sorted(o)}
                         for n, d, o in self.timeline],
            'present_margin_m': self.margin,
            'recent_window_s': self.recent_window,
            'robot_max_offset_from_origin_m': round(self.robot_max_offset, 4),
            'phases': list(self.phase_stats.values()),
        }
        with open(os.path.join(self.output_dir, 'summary.json'), 'w') as f:
            json.dump(summary, f, ensure_ascii=False, indent=2)
        self.log_event(event='finished', sim_time=now)
        self.get_logger().info('scenario finished; summary.json written')
        for s in self.phase_stats.values():
            self.get_logger().info(
                '%-32s residual first/max/last %4s/%4s/%4s  zero at %s' % (
                    s['phase'], s['first_residual'], s['max_residual'], s['last_residual'],
                    s['first_zero_time']))
        if self.shutdown_when_done:
            self.create_timer(1.0, lambda: rclpy.shutdown())

    def destroy_node(self):
        self.csv_file.close()
        self.events.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ClearingScenario()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
