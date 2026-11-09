"""Serving trips base -> table -> base under one costmap condition, with metrics.

The table sequence is a seeded shuffle of the reachable serving goals, the same
for every condition, so conditions differ only in the costmap. Whether the
seat was occupied on arrival is recorded because people react to the robot and
can diverge between conditions.
Outputs (in out_dir): legs.csv, samples.csv, summary.json.
"""
import csv
import json
import math
from pathlib import Path
import random
import subprocess
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException

from restaurant_nav_experiment.metrics import LegStats, footprint_gap

LEG_FIELDS = ['condition', 'scenario', 'seed', 'trip', 'leg', 'slot', 'seat_id', 'start_sim', 'end_sim',
              'duration', 'result', 'path_length', 'robot_stopped_s', 'min_gap_any', 'min_gap_walking',
              'near_miss_events', 'contact_samples', 'human_stopped_near_robot_s', 'seat_occupied_at_end']


class Stop(Exception):
    """ROS was shut down (SIGINT/SIGTERM): leave every wait loop."""


class ServingRunner(BasicNavigator):
    def __init__(self):
        super().__init__('restaurant_serving_runner')
        for name, default in (('condition', 'B0'), ('scenario', 'high'), ('seed', 42),
                              ('experiment_json', ''), ('out_dir', ''), ('start_sim', 40.0),
                              ('stop_new_trips_sim', 330.0), ('leg_timeout', 120.0),
                              ('robot_base_frame', 'robot/base_footprint')):
            self.declare_parameter(name, default)
        g = lambda k: self.get_parameter(k).value
        self.condition, self.scenario, self.seed = g('condition'), g('scenario'), g('seed')
        meta = json.loads(Path(g('experiment_json')).read_text())
        self.base = tuple(meta['base_world'])
        self.goals = [x for x in meta['goals'] if x['reachable']]
        order = list(range(len(self.goals)))
        random.Random(self.seed).shuffle(order)
        self.order = order
        self.out = Path(g('out_dir')); self.out.mkdir(parents=True, exist_ok=True)
        self.start_sim, self.stop_sim, self.leg_timeout = g('start_sim'), g('stop_new_trips_sim'), g('leg_timeout')
        self.frame = g('robot_base_frame')
        self.tf = Buffer(); self.tf_listener = TransformListener(self.tf, self)
        self.snapshot = None; self.speed = 0.0; self.leg = None
        self.prev_agents = {}; self.last_sim = None
        self.samples = open(self.out/'samples.csv', 'w', newline='')
        self.sample_writer = csv.writer(self.samples)
        self.sample_writer.writerow(['sim', 'robot_x', 'robot_y', 'robot_yaw', 'speed', 'leg', 'active_people',
                                     'walking_people', 'min_gap_any', 'min_gap_walking'])
        self.legs_file = open(self.out/'legs.csv', 'w', newline='')
        self.legs_writer = csv.DictWriter(self.legs_file, LEG_FIELDS); self.legs_writer.writeheader()
        self.rows = []; self.failures_seen = 0; self.resets = []; self.lifecycle_retries = []
        self.create_subscription(String, '/restaurant/agents_snapshot', self.on_snapshot, 10)
        self.create_subscription(Odometry, '/odom', lambda m: setattr(self, 'speed', m.twist.twist.linear.x), 10)
        self.agents = []; self.last_sample = None
        self.create_timer(0.1, self.sample)

    # --- sampling ---------------------------------------------------------
    def robot_pose(self):
        try:
            t = self.tf.lookup_transform('map', self.frame, rclpy.time.Time())
        except TransformException:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        return (t.transform.translation.x + self.base[0], t.transform.translation.y + self.base[1]), yaw

    def on_snapshot(self, msg):
        """Keep the latest people state; stopped-near-robot uses the people clock."""
        snap = json.loads(msg.data); self.snapshot = snap
        sim = snap['elapsed_sim_seconds']; self.failures_seen = snap.get('failures', 0)
        dt = 0.0 if self.last_sim is None else max(0.0, sim - self.last_sim); self.last_sim = sim
        pose = self.robot_pose()
        agents, stopped_near = [], 0
        for a in snap['agents']:
            if not a['active']:
                continue
            xy = (a['position']['x'], a['position']['y'])
            agents.append(dict(xy=xy, walking=a['posture'] != 'SEATED'))
            prev = self.prev_agents.get(a['id'])
            if (pose and dt > 0 and prev and a['posture'] == 'STANDING' and math.dist(prev, xy)/dt < .05
                    and not (a.get('queue') or {}).get('role') == 'waiting'
                    and a['diagnostics']['wait_remaining_sim_seconds'] <= 0
                    and math.dist(xy, pose[0]) < LegStats.HUMAN_NEAR):
                stopped_near += 1
            self.prev_agents[a['id']] = xy
        self.agents = agents
        if self.leg:
            self.leg.row['human_stopped_near_robot_s'] += dt * stopped_near

    def sample(self):
        """10 Hz on the sim clock, so a furniture-only world is measured too."""
        sim = self.sim_now()
        dt = 0.0 if self.last_sample is None else max(0.0, sim - self.last_sample); self.last_sample = sim
        pose = self.robot_pose()
        if pose is None:
            return
        (rx, ry), yaw = pose
        agents = self.agents
        gaps = [footprint_gap((rx, ry), yaw, a['xy']) for a in agents]
        gw = [footprint_gap((rx, ry), yaw, a['xy']) for a in agents if a['walking']]
        self.sample_writer.writerow([round(sim, 2), round(rx, 3), round(ry, 3), round(yaw, 3), round(self.speed, 3),
                                     self.leg.row['leg'] if self.leg else '', len(agents),
                                     sum(a['walking'] for a in agents),
                                     round(min(gaps), 3) if gaps else '', round(min(gw), 3) if gw else ''])
        if self.leg:
            self.leg.sample(dt, (rx, ry), yaw, self.speed, agents, 0)

    # --- navigation -------------------------------------------------------
    def spin(self, timeout=0.1):
        if not rclpy.ok():
            raise Stop
        rclpy.spin_once(self, timeout_sec=timeout)

    def done(self):
        if not rclpy.ok():
            raise Stop
        return self.isTaskComplete()

    def call(self, client, request, timeout=5.0):
        if not client.wait_for_service(timeout_sec=1.0):
            return None
        future = client.call_async(request); t0 = time.monotonic()
        while not future.done() and time.monotonic() - t0 < timeout:
            self.spin(0.1)
        return future.result() if future.done() else None

    def wait_nav2(self):
        """Like waitUntilNav2Active, but leaves on shutdown instead of looping.

        Under start-up load the lifecycle manager sometimes aborts bringup
        ("get_state ... async_send_request failed"); after 15 s inactive the
        manager is asked to STARTUP again (logged in summary.json)."""
        from lifecycle_msgs.srv import GetState
        from nav2_msgs.srv import ManageLifecycleNodes
        for node, manager in (('map_server', 'lifecycle_manager_static_map'),
                              ('bt_navigator', 'lifecycle_manager_navigation')):
            client = self.create_client(GetState, f'{node}/get_state')
            restart = self.create_client(ManageLifecycleNodes, f'{manager}/manage_nodes')
            t0 = time.monotonic()
            while True:
                state = self.call(client, GetState.Request())
                if state and state.current_state.label == 'active':
                    break
                if time.monotonic() - t0 > 15.0:
                    req = ManageLifecycleNodes.Request(); req.command = ManageLifecycleNodes.Request.STARTUP
                    reply = self.call(restart, req, timeout=30.0)
                    self.lifecycle_retries.append(dict(manager=manager, ok=bool(reply and reply.success)))
                    self.get_logger().warning(f'{node} inactive after 15 s: asked {manager} to STARTUP again')
                    t0 = time.monotonic()
                self.spin(0.5)
            self.destroy_client(client); self.destroy_client(restart)
        self.get_logger().info('Nav2 is active')

    def sim_now(self):
        """Gazebo simulation time (/clock). The people controller's elapsed
        time starts a few seconds later, when that node starts."""
        return self.get_clock().now().nanoseconds / 1e9

    def pose_msg(self, world_xy, yaw):
        p = PoseStamped(); p.header.frame_id = 'map'; p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x, p.pose.position.y = world_xy[0]-self.base[0], world_xy[1]-self.base[1]
        p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        return p

    def seat_occupied(self, seat_id):
        if not self.snapshot:
            return None
        return any(a['active'] and a['seat_id'] == seat_id and a['posture'] == 'SEATED' for a in self.snapshot['agents'])

    def run_leg(self, trip, leg, goal):
        seat = goal.get('seat_id', 'base'); slot = goal.get('slot', -1)
        self.leg = LegStats(leg, slot, seat, self.sim_now())
        target = self.pose_msg(goal['world'], goal['yaw'])
        result = 'REJECTED'
        if self.goToPose(target):
            t0 = self.sim_now()
            while not self.done():
                if self.sim_now() - t0 > self.leg_timeout:
                    self.cancelTask(); result = 'TIMEOUT'
                    while not self.done():
                        pass
                    break
            if result != 'TIMEOUT':
                result = {TaskResult.SUCCEEDED: 'SUCCEEDED', TaskResult.CANCELED: 'CANCELED',
                          TaskResult.FAILED: 'FAILED'}.get(self.getResult(), 'UNKNOWN')
        row = self.leg.finish(self.sim_now(), result, self.seat_occupied(seat) if leg == 'to_table' else None)
        row.update(condition=self.condition, scenario=self.scenario, seed=self.seed, trip=trip)
        self.legs_writer.writerow(row); self.legs_file.flush(); self.samples.flush()
        self.rows.append(row); self.leg = None
        self.get_logger().info(f'trip {trip} {leg} {seat}: {result} in {row["duration"]} s')
        return result

    def reset_to_base(self, trip):
        """After a failed leg: teleport to the base and clear costmaps so one
        failure does not decide every later leg (ground-truth TF follows)."""
        req = (f'name: "robot", position: {{x: {self.base[0]}, y: {self.base[1]}, z: 0.01}}, '
               'orientation: {w: 1}')
        r = subprocess.run(['gz', 'service', '-s', '/world/restaurant/set_pose', '--reqtype', 'gz.msgs.Pose',
                            '--reptype', 'gz.msgs.Boolean', '--timeout', '3000', '--req', req],
                           capture_output=True, text=True, timeout=10)
        ok = r.returncode == 0 and 'true' in r.stdout
        t0 = self.sim_now()
        while self.sim_now() - t0 < 2.0:
            self.spin(0.1)
        self.clearAllCostmaps()
        self.resets.append(dict(trip=trip, sim=round(self.sim_now(), 2), ok=ok))
        self.get_logger().warning(f'reset to base after trip {trip}: {"ok" if ok else r.stdout + r.stderr}')

    def run(self):
        self.wait_nav2()
        while self.sim_now() < self.start_sim:
            self.spin(0.2)
        base_goal = dict(world=self.base, yaw=0.0)
        trip = 0
        while self.sim_now() < self.stop_sim:
            goal = self.goals[self.order[trip % len(self.order)]]
            if self.run_leg(trip, 'to_table', goal) != 'SUCCEEDED':
                self.reset_to_base(trip); trip += 1
                continue
            # Pause at the table as a serving robot would (assumed 5 s).
            t0 = self.sim_now()
            while self.sim_now() - t0 < 5.0:
                self.spin(0.1)
            if self.run_leg(trip, 'to_base', base_goal) != 'SUCCEEDED':
                self.reset_to_base(trip)
            trip += 1
        self.write_summary(trip)

    def write_summary(self, trips):
        def mean(key, rows):
            vals = [r[key] for r in rows if r[key] is not None]
            return round(sum(vals)/len(vals), 3) if vals else None
        table = [r for r in self.rows if r['leg'] == 'to_table']
        summary = dict(condition=self.condition, scenario=self.scenario, seed=self.seed, trips=trips,
                       legs=len(self.rows), succeeded=sum(r['result'] == 'SUCCEEDED' for r in self.rows),
                       mean_duration=mean('duration', self.rows), mean_path_length=mean('path_length', self.rows),
                       mean_robot_stopped_s=mean('robot_stopped_s', self.rows),
                       min_gap_walking=min((r['min_gap_walking'] for r in self.rows if r['min_gap_walking'] is not None), default=None),
                       near_miss_events=sum(r['near_miss_events'] for r in self.rows),
                       contact_samples=sum(r['contact_samples'] for r in self.rows),
                       human_stopped_near_robot_s=round(sum(r['human_stopped_near_robot_s'] for r in self.rows), 2),
                       seat_occupied_on_arrival=sum(bool(r['seat_occupied_at_end']) for r in table),
                       people_failures_reported=self.failures_seen, resets=self.resets,
                       lifecycle_retries=self.lifecycle_retries,
                       table_legs=len(table), table_succeeded=sum(r['result'] == 'SUCCEEDED' for r in table),
                       mean_table_duration_succeeded=mean('duration', [r for r in table if r['result'] == 'SUCCEEDED']),
                       wall_clock_finished=time.strftime('%Y-%m-%dT%H:%M:%S'))
        (self.out/'summary.json').write_text(json.dumps(summary, indent=1))
        self.get_logger().info('SUMMARY ' + json.dumps(summary))


def main():
    rclpy.init(); node = ServingRunner()
    try:
        node.run()
    except (KeyboardInterrupt, Stop):
        pass
    finally:
        node.samples.close(); node.legs_file.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
