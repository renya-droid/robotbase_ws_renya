#!/usr/bin/env python3
"""Verify a RUNNING test_arena simulation (no robot motion/goals are issued).

Sends manual full snapshots and requests plans with explicit synthetic starts.
Use a private simulation ROS domain and stop all other people publishers first.
"""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.srv import GetCostmap
from lifecycle_msgs.srv import GetState
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters


class Verification:
    def __init__(self):
        self.node = rclpy.create_node('verify_external_camera_people_layer')
        self.publisher = self.node.create_publisher(PoseArray, '/external_camera/people', 1)
        self.points, self.frame, self.sending = [], 'map', True
        self.timer = self.node.create_timer(.1, self.publish,
            clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.costmap = self.node.create_client(GetCostmap, '/global_costmap/get_costmap')
        self.parameters = self.node.create_client(SetParameters, '/global_costmap/global_costmap/set_parameters')
        self.planner = ActionClient(self.node, ComputePathToPose, '/compute_path_to_pose')
        self.planner_state = self.node.create_client(GetState, '/planner_server/get_state')

    def publish(self):
        if not self.sending:
            return
        msg = PoseArray()
        msg.header.frame_id = self.frame
        msg.header.stamp = self.node.get_clock().now().to_msg()  # Wall time vs sim-time receiver.
        for x, y in self.points:
            p = Pose()
            p.position.x, p.position.y, p.orientation.w = float(x), float(y), 1.0
            msg.poses.append(p)
        self.publisher.publish(msg)

    def wait(self, predicate, timeout=15):
        until = time.monotonic()+timeout
        while time.monotonic()<until:
            rclpy.spin_once(self.node, timeout_sec=.02)
            if predicate():
                return
        raise RuntimeError('Verification timed out; check active Nav2 and people-layer logs.')

    def pump(self, seconds):
        until = time.monotonic()+seconds
        while time.monotonic()<until:
            rclpy.spin_once(self.node, timeout_sec=.02)

    def response(self, future):
        self.wait(future.done)
        result = future.result()
        if result is None:
            raise RuntimeError(str(future.exception()))
        return result

    def grid(self):
        return self.response(self.costmap.call_async(GetCostmap.Request())).map

    @staticmethod
    def cost(grid, point):
        meta = grid.metadata
        x = int(math.floor((point[0]-meta.origin.position.x)/meta.resolution))
        y = int(math.floor((point[1]-meta.origin.position.y)/meta.resolution))
        assert 0 <= x < meta.size_x and 0 <= y < meta.size_y, 'Requires the supplied test_arena map'
        return grid.data[y*meta.size_x+x]

    def wait_cost(self, point, predicate, timeout=8):
        until = time.monotonic()+timeout
        while time.monotonic()<until:
            grid = self.grid()
            if predicate(self.cost(grid, point)):
                return grid
            self.pump(.08)
        raise RuntimeError(f'Unexpected cost at {point}: {self.cost(self.grid(), point)}')

    def enabled(self, enabled):
        request = SetParameters.Request(parameters=[Parameter(name='people_layer.enabled',
            value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=enabled))])
        response = self.response(self.parameters.call_async(request))
        assert all(r.successful for r in response.results), str(response)

    def plan(self):
        def pose(x, y):
            p = PoseStamped()
            p.header.frame_id = 'map'
            p.pose.position.x, p.pose.position.y, p.pose.orientation.w = x, y, 1.0
            return p
        goal = ComputePathToPose.Goal(start=pose(-4., -4.), goal=pose(0., -4.),
            use_start=True, planner_id='GridBased')
        handle = self.response(self.planner.send_goal_async(goal))
        assert handle.accepted
        response = self.response(handle.get_result_async())
        assert response.status == 4 and response.result.error_code == 0, str(response)
        return [(p.pose.position.x, p.pose.position.y) for p in response.result.path.poses]

    def run(self):
        assert self.costmap.wait_for_service(timeout_sec=90), 'Global costmap service unavailable'
        assert self.parameters.wait_for_service(timeout_sec=15)
        assert self.planner.wait_for_server(timeout_sec=15)
        assert self.planner_state.wait_for_service(timeout_sec=15)
        until = time.monotonic()+90
        while self.response(self.planner_state.call_async(GetState.Request())).current_state.id != 3:
            if time.monotonic()>until:
                raise RuntimeError('Planner never became active; check simulator TF and sensors')
            self.pump(.2)
        self.wait(lambda: self.publisher.get_subscription_count()>0)
        self.pump(.4)
        assert self.node.count_publishers('/external_camera/people') == 1, 'Stop other people publishers first'
        self.enabled(True)
        self.pump(1.0)
        a, b, wall = (-2., -4.), (-2., -3.), (-2., -1.8)
        baseline = self.wait_cost(a, lambda c: c == 0)
        assert self.cost(baseline, wall) == 254
        baseline_path = self.plan()
        self.points = [a]
        occupied = self.wait_cost(a, lambda c: 150<c<=252)
        detour_path = self.plan()
        distance = lambda path: min(math.dist(p, a) for p in path)
        assert distance(detour_path)>distance(baseline_path)+.4
        self.points = [b]
        self.wait_cost(b, lambda c: 150<c<=252)
        moved = self.wait_cost(a, lambda c: c == self.cost(baseline, a))
        assert self.cost(moved, wall) == 254
        self.points = []
        self.wait_cost(b, lambda c: c == self.cost(baseline, b))
        empty_path = self.plan()
        self.points = [a, b]
        self.wait_cost(a, lambda c: c>150)
        self.wait_cost(b, lambda c: c>150)
        self.sending = False
        timeout_started = time.monotonic()
        expired = self.wait_cost(a, lambda c: c == 0)
        timeout_elapsed = time.monotonic()-timeout_started
        assert self.cost(expired, b) == self.cost(baseline, b)
        self.sending, self.points = True, [a]
        self.wait_cost(a, lambda c: c>150)
        self.sending = False
        self.enabled(False)
        self.wait_cost(a, lambda c: c == 0)
        self.enabled(True)
        self.pump(.7)
        assert self.cost(self.grid(), a) == 0
        self.sending, self.frame = True, 'wrong_map'
        self.wait_cost(a, lambda c: c == 0)
        self.frame, self.points = 'map', [(float('nan'), 0), (float('inf'), 0), (1e300, 0), a]
        self.wait_cost(a, lambda c: c>150)
        self.frame = 'wrong_map'
        self.wait_cost(a, lambda c: c == 0)
        self.frame, self.points = 'map', []
        final = self.wait_cost(a, lambda c: c == 0)
        assert self.cost(final, wall) == 254
        return dict(test_map='test_arena', robot_motion_commanded=False,
            layer_loaded=True, full_snapshots=True, multiple_people=True, move_clears_old=True,
            empty_clears=True, timeout_clears=True, observed_timeout_seconds=timeout_elapsed,
            disable_reenable_clears=True, invalid_frame_rejected=True,
            nonfinite_and_huge_ignored=True, existing_wall_preserved=True,
            baseline_path_clearance_m=distance(baseline_path),
            people_path_clearance_m=distance(detour_path),
            after_empty_path_clearance_m=distance(empty_path),
            baseline_path=baseline_path, people_path=detour_path,
            map_metadata=dict(size_x=occupied.metadata.size_x, size_y=occupied.metadata.size_y,
                resolution=occupied.metadata.resolution,
                origin=[occupied.metadata.origin.position.x, occupied.metadata.origin.position.y]),
            baseline_costmap=list(baseline.data), people_costmap=list(occupied.data))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default='/tmp/external_camera_people_layer_check.json')
    args = parser.parse_args()
    rclpy.init(args=[])
    verification = Verification()
    try:
        report = verification.run()
        Path(args.output).write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps({k:v for k,v in report.items() if 'path' not in k and 'costmap' not in k}, indent=2))
        print(f'Full report: {args.output}')
    finally:
        verification.sending, verification.points, verification.frame = True, [], 'map'
        verification.publish()
        verification.pump(.15)
        verification.node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
