#!/usr/bin/env python3
"""Move the robot over the slope course and record false ground obstacles.

The robot is moved kinematically: every 20 ms (sim time) the node advances x
by speed*dt and sets the model pose with Gazebo set_pose.  z is the surface
height under the wheels, pitch is the slope of the chord between the front
and rear caster contact points (an approximation of a suspended chassis; the
transition shape of the real robot is not modelled).

The course has no obstacles, so every lethal cell in the Velodyne costmap is
the ground (ramp surface) being marked as an obstacle.  The pose read back
from Gazebo dynamic_pose/info is logged as ground truth at the same sim time.

Outputs in output_dir: timeline.csv, summary.json.
"""

import csv
import json
import math
import os
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


def quat_pitch(q):
    return math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))


class SlopeDrive(Node):

    def __init__(self):
        super().__init__('velodyne_slope_drive')
        p = self.declare_parameter
        self.course = json.loads(p('course_json', '{}').value)
        self.speed = float(p('speed', 0.4).value)
        self.start_delay = float(p('start_delay', 4.0).value)
        self.robot_name = p('robot_model', 'robot').value
        self.world = p('world_name', 'velodyne_slope_study').value
        self.front_caster = float(p('front_caster_x', 0.30).value)
        self.rear_caster = float(p('rear_caster_x', -0.35).value)
        self.shutdown_when_done = bool(p('shutdown_when_done', False).value)
        self.loop = bool(p('loop', False).value)
        out = p('output_dir', '').value or os.path.join(
            os.path.expanduser('~'), '.ros', 'velodyne_slope_study', time.strftime('%Y%m%d_%H%M%S'))
        self.output_dir = os.path.expanduser(out)
        os.makedirs(self.output_dir, exist_ok=True)
        self.end_x = float(self.course.get('end_x', 20.0))

        self.gt = None  # (x, y, z, pitch)
        self.odom = None
        self.imu_pitch = math.nan
        self.viz = {}
        self.t_start = None
        self.state = 'wait'
        self.direction = 1.0
        self.stats = {}
        self.false_cells_ever = set()

        self.client = self.create_client(SetEntityPose, f'/world/{self.world}/set_pose')
        self.cmd_x = 0.0
        self.last_tick = None
        self.pending = False
        self.pub_phase = self.create_publisher(String, '/velodyne_study/phase', 2)
        self.create_subscription(TFMessage, '/velodyne_study/world_poses', self.on_world, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(Imu, '/imu', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(String, '/velodyne_coverage_viz/status', self.on_viz, 5)
        self.create_subscription(OccupancyGrid, '/velodyne_study/costmap', self.on_costmap, 2)
        self.create_timer(0.05, self.tick)

        self.csv_file = open(os.path.join(self.output_dir, 'timeline.csv'), 'w', newline='')
        self.csv = csv.writer(self.csv_file)
        self.csv.writerow([
            'sim_time', 'phase', 'cmd_x', 'cmd_z', 'cmd_pitch_deg', 'gt_x', 'gt_z', 'gt_pitch_deg_noseup', 'imu_pitch_deg_noseup', 'odom_x',
            'lethal_cells', 'lethal_ahead_min_dist', 'scan3_inf_ratio', 'scan3_winner_by_ring'])
        self.get_logger().info(f'course {self.course}  output {self.output_dir}')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_world(self, msg):
        # ros_gz_bridge (jazzy) leaves child_frame_id empty for Pose_V entries.
        # The robot is the only dynamic model, and gz lists the model pose first,
        # followed by its links, so use the named entry if present, else index 0.
        if not msg.transforms:
            return
        tf = next((t for t in msg.transforms if t.child_frame_id == self.robot_name),
                  msg.transforms[0])
        t, q = tf.transform.translation, tf.transform.rotation
        self.gt = (t.x, t.y, t.z, quat_pitch(q))

    def on_odom(self, msg):
        p = msg.pose.pose
        self.odom = (p.position.x, p.position.y, 2.0 * math.atan2(p.orientation.z, p.orientation.w))

    def on_imu(self, msg):
        self.imu_pitch = quat_pitch(msg.orientation)

    def on_viz(self, msg):
        self.viz = json.loads(msg.data)

    def phase(self, x):
        c = self.course
        if x < c['up_start']:
            return '1_flat_before'
        if x < c['up_end']:
            return '2_uphill'
        if x < c['down_start']:
            return '3_plateau'
        if x < c['down_end']:
            return '4_downhill'
        return '5_flat_after'

    def surface(self, x):
        c = self.course
        a = math.radians(c['slope_deg'])
        if x <= c['up_start'] or x >= c['down_end']:
            return 0.0
        if x < c['up_end']:
            return (x - c['up_start']) * math.tan(a)
        if x <= c['down_start']:
            return c['rise']
        return c['rise'] - (x - c['down_start']) * math.tan(a)

    def pose_at(self, x):
        """(z, nose-up pitch) with wheels on the surface and the chassis along
        the chord between the caster contact points."""
        zf = self.surface(x + self.front_caster)
        zr = self.surface(x + self.rear_caster)
        pitch_up = math.atan2(zf - zr, self.front_caster - self.rear_caster)
        return self.surface(x), pitch_up

    def send_pose(self, x):
        if self.pending or not self.client.service_is_ready():
            return
        z, up = self.pose_at(x)
        req = SetEntityPose.Request()
        req.entity = Entity(name=self.robot_name, type=Entity.MODEL)
        req.pose.position.x, req.pose.position.z = float(x), float(z)
        # gz pitch is positive nose-down
        req.pose.orientation.y = math.sin(-up / 2.0)
        req.pose.orientation.w = math.cos(-up / 2.0)
        self.pending = True
        fut = self.client.call_async(req)
        fut.add_done_callback(lambda _f: setattr(self, 'pending', False))

    def tick(self):
        now = self.now()
        if now <= 0.0 or self.gt is None:
            return
        if self.t_start is None:
            self.t_start = now + self.start_delay
        dt = 0.0 if self.last_tick is None else max(0.0, now - self.last_tick)
        self.last_tick = now
        if now < self.t_start or self.state == 'done':
            return
        if self.state == 'wait':
            self.state = 'drive'
        if self.state == 'drive':
            self.cmd_x += self.direction * self.speed * dt
            if self.direction > 0 and self.cmd_x >= self.end_x:
                self.cmd_x = self.end_x
                if self.loop:
                    self.direction = -1.0
                else:
                    self.state = 'settle'
                    self.settle_until = now + 3.0
            elif self.direction < 0 and self.cmd_x <= 0.0:
                self.cmd_x = 0.0
                self.direction = 1.0
            self.send_pose(self.cmd_x)
        elif self.state == 'settle':
            self.send_pose(self.cmd_x)
            if now >= self.settle_until:
                self.finish()
        self.pub_phase.publish(String(data=self.phase(self.cmd_x)))

    def on_costmap(self, msg):
        if self.gt is None or self.odom is None or self.state not in ('drive', 'settle'):
            return
        info = msg.info
        grid = np.asarray(msg.data, dtype=np.int16).reshape(info.height, info.width)
        rows, cols = np.nonzero(grid >= 100)
        xs = info.origin.position.x + (cols + 0.5) * info.resolution
        ys = info.origin.position.y + (rows + 0.5) * info.resolution
        ox, oy, oyaw = self.odom
        dx, dy = xs - ox, ys - oy
        fwd = np.cos(oyaw) * dx + np.sin(oyaw) * dy
        dist = np.hypot(dx, dy)
        ahead = dist[fwd * self.direction > 0]
        # odom-frame cell keys (the rolling window's own indices move with the robot)
        res = info.resolution
        for cx, cy in zip(np.floor(xs / res).astype(int).tolist(),
                          np.floor(ys / res).astype(int).tolist()):
            self.false_cells_ever.add((cx, cy))
        phase = self.phase(self.cmd_x)
        cz, cup = self.pose_at(self.cmd_x)
        n = int(len(xs))
        s = self.stats.setdefault(phase, {
            'samples': 0, 'max_lethal': 0, 'min_pitch_noseup_deg': 99.0, 'max_pitch_noseup_deg': -99.0,
            'scan3_inf_ratio_min': 1.0, 'scan3_inf_ratio_max': 0.0, 'closest_lethal_ahead': None})
        s['samples'] += 1
        s['max_lethal'] = max(s['max_lethal'], n)
        pd = -math.degrees(self.gt[3])  # nose-up positive
        s['min_pitch_noseup_deg'] = round(min(s['min_pitch_noseup_deg'], pd), 2)
        s['max_pitch_noseup_deg'] = round(max(s['max_pitch_noseup_deg'], pd), 2)
        inf_r = self.viz.get('scan3_inf_ratio')
        if inf_r is not None:
            s['scan3_inf_ratio_min'] = round(min(s['scan3_inf_ratio_min'], inf_r), 4)
            s['scan3_inf_ratio_max'] = round(max(s['scan3_inf_ratio_max'], inf_r), 4)
        if len(ahead):
            a = float(ahead.min())
            if s['closest_lethal_ahead'] is None or a < s['closest_lethal_ahead']:
                s['closest_lethal_ahead'] = round(a, 2)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.csv.writerow([
            f'{stamp:.3f}', phase, f'{self.cmd_x:.3f}', f'{cz:.3f}', f'{math.degrees(cup):.2f}',
            f'{self.gt[0]:.3f}', f'{self.gt[2]:.3f}', f'{pd:.2f}',
            f'{-math.degrees(self.imu_pitch):.2f}', f'{ox:.3f}', n,
            f'{float(ahead.min()):.2f}' if len(ahead) else '',
            f'{inf_r:.4f}' if inf_r is not None else '',
            json.dumps(self.viz.get('scan3_winner_by_ring', {}))])
        self.csv_file.flush()

    def finish(self):
        self.state = 'done'
        summary = {
            'course': self.course, 'speed_mps': self.speed,
            'final_gt': [round(v, 3) for v in self.gt],
            'final_odom': [round(v, 3) for v in self.odom] if self.odom else None,
            'distinct_lethal_cells_ever': len(self.false_cells_ever),
            'motion': 'kinematic set_pose; pitch = caster chord; nose-up positive',
            'phases': dict(sorted(self.stats.items())),
        }
        with open(os.path.join(self.output_dir, 'summary.json'), 'w') as f:
            json.dump(summary, f, ensure_ascii=False, indent=2)
        self.get_logger().info('slope drive finished: ' + json.dumps(summary['phases']))
        if self.shutdown_when_done:
            self.create_timer(1.0, lambda: rclpy.shutdown())

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SlopeDrive()
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
