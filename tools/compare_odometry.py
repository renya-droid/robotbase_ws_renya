#!/usr/bin/env python3
"""Compare raw and filtered odometry against a manually measured movement.

This program only subscribes to ROS topics. Drive the robot separately.
"""

import argparse
import math
import select
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


class PoseSeries:
    def __init__(self):
        self.first = None
        self.last = None
        self.previous_yaw = None
        self.first_yaw = None
        self.unwrapped_yaw = 0.0
        self.count = 0
        self.latest_time = None
        self.max_gap = 0.0

    def add(self, msg):
        now = time.monotonic()
        angle = yaw(msg.pose.pose.orientation)
        if self.previous_yaw is not None:
            self.max_gap = max(self.max_gap, now - self.latest_time)
            self.unwrapped_yaw += math.atan2(
                math.sin(angle - self.previous_yaw), math.cos(angle - self.previous_yaw))
        pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if self.first is None:
            self.first = pose
            self.first_yaw = angle
        self.last = pose
        self.previous_yaw = angle
        self.latest_time = now
        self.count += 1

    def result(self, offset_x=0.0, offset_y=0.0):
        def measured_point(position, angle):
            return (position[0] + offset_x * math.cos(angle) - offset_y * math.sin(angle),
                    position[1] + offset_x * math.sin(angle) + offset_y * math.cos(angle))

        start = measured_point(self.first, self.first_yaw)
        end = measured_point(self.last, self.previous_yaw)
        return (math.hypot(end[0] - start[0], end[1] - start[1]),
                math.degrees(self.unwrapped_yaw))


class CompareNode(Node):
    def __init__(self):
        super().__init__("compare_odometry")
        self.series = {"/odom": PoseSeries(), "/odom/filtered": PoseSeries()}
        for topic in self.series:
            self.create_subscription(Odometry, topic,
                                     lambda msg, name=topic: self.series[name].add(msg), 50)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("straight", "turn"))
    parser.add_argument("--duration", type=float, default=25.0)
    parser.add_argument("--manual", action="store_true",
                        help="press Enter to start and press Enter again after stopping the robot")
    parser.add_argument("--start-delay", type=float, default=5.0)
    parser.add_argument("--actual-m", type=float,
                        help="measured floor distance, for a straight run")
    parser.add_argument("--actual-deg", type=float,
                        help="signed physical turn; counterclockwise is positive")
    parser.add_argument("--wheel-circumference", type=float, default=0.6283)
    parser.add_argument("--track-width", type=float, default=0.435)
    parser.add_argument("--measure-offset-x", type=float, default=0.0,
                        help="metres from base_link origin to the floor measurement point, forward positive")
    parser.add_argument("--measure-offset-y", type=float, default=0.0,
                        help="metres from base_link origin to the floor measurement point, left positive")
    args = parser.parse_args()
    if args.duration <= 0 or args.start_delay < 0:
        parser.error("duration must be positive and start-delay nonnegative")
    if args.wheel_circumference <= 0 or args.track_width <= 0:
        parser.error("wheel-circumference and track-width must be positive")
    if args.mode == "straight" and args.actual_deg is not None:
        parser.error("--actual-deg is only for turn mode")
    if args.mode == "turn" and args.actual_m is not None:
        parser.error("--actual-m is only for straight mode")
    if args.actual_m is not None and args.actual_m <= 0:
        parser.error("--actual-m must be positive")
    if args.manual and not sys.stdin.isatty():
        parser.error("--manual needs an interactive terminal")

    rclpy.init()
    node = CompareNode()
    try:
        deadline = time.monotonic() + 6.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if all(s.count for s in node.series.values()):
                break
        if not all(s.count for s in node.series.values()):
            raise RuntimeError("both /odom and /odom/filtered must be publishing")

        if args.manual:
            input("ロボットを開始線で停止させ、準備ができたら Enter を押してください: ")
        else:
            print(f"待機中：ロボットを動かさないでください。{args.start_delay:g} 秒後に計測を開始します。", flush=True)
            deadline = time.monotonic() + args.start_delay
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
        node.series = {topic: PoseSeries() for topic in node.series}
        if args.manual:
            # Capture the starting pose while the robot is still stationary.
            deadline = time.monotonic() + 2.0
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
                if all(s.count for s in node.series.values()):
                    break
            if not all(s.count for s in node.series.values()):
                raise RuntimeError("starting odom samples were not received")
            print("今から動かしてください。終了線でキーオペの s を押し、計測端末に戻って Enter を押してください。", flush=True)
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.05)
                if select.select([sys.stdin], [], [], 0)[0]:
                    sys.stdin.readline()
                    break
        else:
            print(f"今から動かしてください！ 計測時間は {args.duration:g} 秒です。終了前にキーオペの s で停止してください。", flush=True)
            deadline = time.monotonic() + args.duration
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)

        results = {}
        for topic, series in node.series.items():
            if series.count < 2 or series.max_gap > 0.5 or time.monotonic() - series.latest_time > 0.5:
                raise RuntimeError(f"{topic} was missing or stale; discard this run")
            results[topic] = series.result(args.measure_offset_x, args.measure_offset_y)
            distance, turn = results[topic]
            print(f"{topic:15} displacement={distance:.3f} m  turn={turn:+.2f} deg")

        if args.measure_offset_x or args.measure_offset_y:
            print(f"measurement point: x={args.measure_offset_x:+.3f} m, y={args.measure_offset_y:+.3f} m from base_link")

        raw_distance, raw_turn = results["/odom"]
        actual_m = args.actual_m
        actual_deg = args.actual_deg
        if sys.stdin.isatty() and args.mode == "straight" and actual_m is None:
            entry = input("床で測った移動距離を m 単位で入力（測っていなければ空欄で Enter）: ").strip()
            actual_m = float(entry) if entry else None
        if sys.stdin.isatty() and args.mode == "turn" and actual_deg is None:
            entry = input("実際の旋回角度を度単位で入力（反時計回りは正、測っていなければ空欄）: ").strip()
            actual_deg = float(entry) if entry else None
        if actual_m is not None and actual_m <= 0:
            raise RuntimeError("floor distance must be positive")
        if args.mode == "straight" and actual_m is not None:
            if raw_distance < 0.1:
                raise RuntimeError("raw odom displacement is too small for calibration")
            error = (raw_distance / actual_m - 1) * 100
            suggested = args.wheel_circumference * actual_m / raw_distance
            filtered_distance = results["/odom/filtered"][0]
            print(f"floor distance={actual_m:.3f} m")
            print(f"/odom distance difference={raw_distance - actual_m:+.3f} m ({error:+.2f}%)")
            print(f"/odom/filtered distance difference={filtered_distance - actual_m:+.3f} m")
            print(f"suggested wheel_circumference={suggested:.5f} m")
        if args.mode == "turn" and actual_deg is not None:
            if abs(actual_deg) < 10 or abs(raw_turn) < 10:
                raise RuntimeError("turn at least 10 degrees for calibration")
            if raw_turn * actual_deg <= 0:
                raise RuntimeError("raw odom and physical turns have opposite signs; check encoder signs")
            suggested = args.track_width * raw_turn / actual_deg
            print(f"physical turn={actual_deg:+.2f} deg")
            print(f"suggested track_width={suggested:.5f} m")
        print("Repeat in both directions; compare runs before editing parameters.")
    except (RuntimeError, KeyboardInterrupt, ValueError, EOFError) as exc:
        print(f"Measurement aborted: {exc}")
        return 2
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
