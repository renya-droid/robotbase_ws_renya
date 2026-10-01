#!/usr/bin/env python3
"""Drive a bounded odometry distance for wheel-circumference calibration."""

import argparse
import json
import math
import signal
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool


class DistanceCalibration(Node):
    def __init__(self, topic):
        super().__init__("wheel_distance_calibration")
        self.pub = self.create_publisher(Twist, topic, 10)
        self.create_subscription(Odometry, "/odom", self._odom, 50)
        self.create_subscription(Bool, "/stop", self._stop, 10)
        self.odom = None
        self.odom_time = 0.0
        self.stop_active = False

    def _odom(self, msg):
        self.odom = msg
        self.odom_time = time.monotonic()

    def _stop(self, msg):
        self.stop_active = bool(msg.data)

    def command(self, speed):
        msg = Twist()
        msg.linear.x = speed
        self.pub.publish(msg)

    def zero(self, duration=0.8):
        end = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < end:
            self.command(0.0)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.05)


def spin_available(node):
    for _ in range(6):
        rclpy.spin_once(node, timeout_sec=0.0)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", action="store_true")
    parser.add_argument("--target", type=float, default=2.5)
    parser.add_argument("--speed", type=float, default=0.2)
    parser.add_argument("--command-topic", default="/cmd_vel_direct")
    parser.add_argument("--output-dir", default="motor_id_results")
    parser.add_argument("--wheel-circumference", type=float, default=0.6283)
    args = parser.parse_args()
    if not (0.5 <= args.target <= 3.0):
        parser.error("target must be between 0.5 and 3.0 m")
    if not (0.05 <= abs(args.speed) <= 0.30):
        parser.error("absolute speed must be between 0.05 and 0.30 m/s")
    if not args.run:
        print(f"DRY RUN: target={args.target:.2f} m speed={args.speed:.2f} m/s")
        return 0

    interrupted = False

    def interrupt(_signum, _frame):
        nonlocal interrupted
        interrupted = True

    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    rclpy.init()
    node = DistanceCalibration(args.command_topic)
    samples = []
    try:
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.odom is not None and node.pub.get_subscription_count() > 0:
                break
        if node.odom is None:
            raise RuntimeError("no /odom received")
        if node.pub.get_subscription_count() == 0:
            raise RuntimeError(f"{args.command_topic} has no subscriber")
        if node.stop_active:
            raise RuntimeError("/stop is active")

        node.zero(1.0)
        start = node.odom.pose.pose.position
        previous = (start.x, start.y)
        path_length = 0.0
        started = time.monotonic()
        max_duration = args.target / abs(args.speed) * 2.0 + 5.0
        while path_length < args.target:
            spin_available(node)
            if interrupted:
                raise RuntimeError("interrupted")
            if node.stop_active:
                raise RuntimeError("/stop became true")
            if time.monotonic() - node.odom_time > 0.5:
                raise RuntimeError("/odom became stale")
            if time.monotonic() - started > max_duration:
                raise RuntimeError("distance target timeout")
            position = node.odom.pose.pose.position
            current = (position.x, position.y)
            step = math.hypot(current[0] - previous[0], current[1] - previous[1])
            if step < 0.05:
                path_length += step
            previous = current
            displacement = math.hypot(current[0] - start.x, current[1] - start.y)
            samples.append({
                "t": time.monotonic() - started,
                "path_length": path_length,
                "displacement": displacement,
                "x": current[0],
                "y": current[1],
            })
            node.command(args.speed)
            time.sleep(0.05)
        node.zero()
        elapsed = time.monotonic() - started
        result = {
            "target_odom_m": args.target,
            "odom_path_m": path_length,
            "odom_displacement_m": samples[-1]["displacement"],
            "command_speed_mps": args.speed,
            "elapsed_s": elapsed,
            "actual_distance_m": None,
            "wheel_circumference_m": args.wheel_circumference,
        }
        stamp = time.strftime("%Y%m%d_%H%M%S")
        output = Path(args.output_dir) / f"{stamp}_wheel_distance.json"
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(result, indent=2))
        print(f"JSON: {output}")
    except Exception as exc:
        node.zero()
        print(f"ABORTED: {exc}", file=sys.stderr)
        return 2
    finally:
        node.zero(0.3)
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
