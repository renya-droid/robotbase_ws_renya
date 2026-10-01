#!/usr/bin/env python3
"""Safely excite and identify the robot's closed-loop differential drive.

The measured plant includes twist_mux, the Roboteq driver command shaping,
motor/controller dynamics, drivetrain, and encoder velocity estimation.
"""

import argparse
import csv
import json
import math
import signal
import statistics
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool


TRACK_WIDTH = 0.435
SAMPLE_HZ = 20.0


@dataclass(frozen=True)
class Phase:
    name: str
    duration: float
    command: float


PROFILES = {
    "linear": [
        Phase("settle", 2.0, 0.0),
        Phase("forward_1", 3.0, 0.15), Phase("stop_1", 2.0, 0.0),
        Phase("reverse_1", 3.0, -0.15), Phase("stop_2", 2.0, 0.0),
        Phase("forward_2", 3.0, 0.30), Phase("stop_3", 2.0, 0.0),
        Phase("reverse_2", 3.0, -0.30), Phase("final_stop", 2.0, 0.0),
    ],
    "angular": [
        Phase("settle", 2.0, 0.0),
        Phase("left_1", 4.0, 0.40), Phase("stop_1", 2.0, 0.0),
        Phase("right_1", 4.0, -0.40), Phase("stop_2", 2.0, 0.0),
        Phase("left_2", 4.0, 0.65), Phase("stop_3", 2.0, 0.0),
        Phase("right_2", 4.0, -0.65), Phase("final_stop", 2.0, 0.0),
    ],
    "angular_low": [
        Phase("settle", 2.0, 0.0),
        Phase("left_010", 2.5, 0.10), Phase("stop_010a", 1.0, 0.0),
        Phase("right_010", 2.5, -0.10), Phase("stop_010b", 1.0, 0.0),
        Phase("left_020", 2.5, 0.20), Phase("stop_020a", 1.0, 0.0),
        Phase("right_020", 2.5, -0.20), Phase("stop_020b", 1.0, 0.0),
        Phase("left_030", 2.5, 0.30), Phase("stop_030a", 1.0, 0.0),
        Phase("right_030", 2.5, -0.30), Phase("stop_030b", 1.0, 0.0),
        Phase("left_040", 2.5, 0.40), Phase("stop_040a", 1.0, 0.0),
        Phase("right_040", 2.5, -0.40), Phase("final_stop", 2.0, 0.0),
    ],
    "angular_onset": [
        Phase("settle", 2.0, 0.0),
        Phase("left_045", 2.5, 0.45), Phase("stop_045a", 1.0, 0.0),
        Phase("right_045", 2.5, -0.45), Phase("stop_045b", 1.0, 0.0),
        Phase("left_050", 2.5, 0.50), Phase("stop_050a", 1.0, 0.0),
        Phase("right_050", 2.5, -0.50), Phase("stop_050b", 1.0, 0.0),
        Phase("left_055", 2.5, 0.55), Phase("stop_055a", 1.0, 0.0),
        Phase("right_055", 2.5, -0.55), Phase("stop_055b", 1.0, 0.0),
        Phase("left_060", 2.5, 0.60), Phase("stop_060a", 1.0, 0.0),
        Phase("right_060", 2.5, -0.60), Phase("stop_060b", 1.0, 0.0),
        Phase("left_065", 2.5, 0.65), Phase("stop_065a", 1.0, 0.0),
        Phase("right_065", 2.5, -0.65), Phase("final_stop", 2.0, 0.0),
    ],
}


class IdentificationNode(Node):
    def __init__(self, axis: str, command_topic: str):
        super().__init__("drive_identification")
        self.axis = axis
        self.command_topic = command_topic
        self.publisher = self.create_publisher(Twist, command_topic, 10)
        self.create_subscription(Twist, "/cmd_vel", self._cmd_callback, 20)
        self.create_subscription(Odometry, "/odom", self._odom_callback, 50)
        self.create_subscription(Bool, "/stop", self._stop_callback, 10)
        self.last_cmd = Twist()
        self.last_cmd_received = 0.0
        self.last_odom = None
        self.last_odom_received = 0.0
        self.stop_active = False
        self.abort_reason = ""

    def _cmd_callback(self, msg):
        self.last_cmd = msg
        self.last_cmd_received = time.monotonic()

    def _odom_callback(self, msg):
        self.last_odom = msg
        self.last_odom_received = time.monotonic()

    def _stop_callback(self, msg):
        self.stop_active = bool(msg.data)
        if self.stop_active:
            self.abort_reason = "/stop became true"

    def publish(self, value: float):
        msg = Twist()
        if self.axis == "linear":
            msg.linear.x = value
        else:
            msg.angular.z = value
        self.publisher.publish(msg)

    def publish_zero(self, seconds: float = 0.7):
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            self.publisher.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.05)

    def row(self, elapsed: float, phase: Phase):
        odom = self.last_odom
        if odom is None:
            return None
        v = odom.twist.twist.linear.x
        w = odom.twist.twist.angular.z
        half_track = TRACK_WIDTH / 2.0
        return {
            "t": elapsed,
            "phase": phase.name,
            "requested": phase.command,
            "cmd_linear": self.last_cmd.linear.x,
            "cmd_angular": self.last_cmd.angular.z,
            "odom_linear": v,
            "odom_angular": w,
            "wheel_left": v - half_track * w,
            "wheel_right": v + half_track * w,
            "x": odom.pose.pose.position.x,
            "y": odom.pose.pose.position.y,
            "stop": int(self.stop_active),
        }


def median(values):
    return statistics.median(values) if values else float("nan")


def analyze(rows, axis):
    cmd_key = "cmd_linear" if axis == "linear" else "cmd_angular"
    out_key = "odom_linear" if axis == "linear" else "odom_angular"
    phases = []
    names = []
    for row in rows:
        if row["phase"] not in names:
            names.append(row["phase"])
    for name in names:
        sample = [r for r in rows if r["phase"] == name]
        if not sample:
            continue
        end_t = sample[-1]["t"]
        steady = [r for r in sample if r["t"] >= end_t - 1.0]
        requested = sample[0]["requested"]
        cmd = median([r[cmd_key] for r in steady])
        output = median([r[out_key] for r in steady])
        left = median([r["wheel_left"] for r in steady])
        right = median([r["wheel_right"] for r in steady])
        phases.append({
            "name": name, "requested": requested, "actual_command": cmd,
            "steady_output": output,
            "gain": output / cmd if abs(cmd) > 1e-6 else None,
            "wheel_left": left, "wheel_right": right,
        })

    dynamics = []
    for index, phase in enumerate(phases):
        if abs(phase["actual_command"]) < 1e-6 or index == 0:
            continue
        phase_rows = [r for r in rows if r["phase"] == phase["name"]]
        start_t = phase_rows[0]["t"]
        pre = [r for r in rows if start_t - 0.5 <= r["t"] < start_t]
        y0 = median([r[out_key] for r in pre])
        yss = phase["steady_output"]
        delta = yss - y0
        if not math.isfinite(delta) or abs(delta) < 1e-4:
            continue
        crossings = {}
        for fraction in (0.10, 0.632):
            target = y0 + fraction * delta
            for row in phase_rows:
                reached = row[out_key] >= target if delta > 0 else row[out_key] <= target
                if reached:
                    crossings[fraction] = row["t"] - start_t
                    break
        if 0.10 in crossings and 0.632 in crossings:
            dynamics.append({
                "phase": phase["name"],
                "delay_s": crossings[0.10],
                "time_constant_s": max(0.0, crossings[0.632] - crossings[0.10]),
            })

    moving = [p for p in phases if abs(p["actual_command"]) > 1e-6]
    gains = [p["gain"] for p in moving if p["gain"] is not None]
    result = {
        "axis": axis,
        "model": "first-order plus dead time, measured at /cmd_vel -> /odom",
        "steady_gain_median": median(gains),
        "delay_s_median": median([d["delay_s"] for d in dynamics]),
        "time_constant_s_median": median([d["time_constant_s"] for d in dynamics]),
        "phases": phases,
        "dynamics": dynamics,
    }
    if axis == "linear":
        ratios = []
        for p in moving:
            denom = (abs(p["wheel_left"]) + abs(p["wheel_right"])) / 2.0
            if denom > 1e-4:
                ratios.append((abs(p["wheel_right"]) - abs(p["wheel_left"])) / denom)
        result["right_left_speed_asymmetry_median"] = median(ratios)
    return result


def write_results(rows, result, output_prefix):
    output_prefix.parent.mkdir(parents=True, exist_ok=True)
    csv_path = output_prefix.with_suffix(".csv")
    json_path = output_prefix.with_suffix(".json")
    with csv_path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    with json_path.open("w") as stream:
        json.dump(result, stream, indent=2, ensure_ascii=False, allow_nan=False)
    return csv_path, json_path


def finite_or_none(value):
    return value if isinstance(value, (int, float)) and math.isfinite(value) else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("axis", choices=PROFILES)
    parser.add_argument("--run", action="store_true", help="actually command the robot")
    parser.add_argument("--command-topic", default="/cmd_vel_direct")
    parser.add_argument("--output-dir", default="motor_id_results")
    args = parser.parse_args()
    duration = sum(p.duration for p in PROFILES[args.axis])
    if not args.run:
        print(f"DRY RUN: {args.axis}, {duration:.0f} s")
        for phase in PROFILES[args.axis]:
            print(f"  {phase.name:14s} {phase.duration:4.1f} s  command={phase.command:+.3f}")
        print("Add --run only after the area is clear and the physical E-stop is ready.")
        return 0

    rclpy.init()
    node = IdentificationNode(args.axis, args.command_topic)
    interrupted = False

    def request_stop(_signum, _frame):
        nonlocal interrupted
        interrupted = True
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    rows = []
    try:
        preflight_end = time.monotonic() + 10.0
        while time.monotonic() < preflight_end:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.last_odom is not None and node.publisher.get_subscription_count() > 0:
                break
        if node.last_odom is None:
            raise RuntimeError("no /odom received during preflight")
        if node.publisher.get_subscription_count() == 0:
            raise RuntimeError(f"{node.command_topic} has no subscriber")
        if node.stop_active:
            raise RuntimeError("/stop is active")

        started = time.monotonic()
        for phase in PROFILES[args.axis]:
            phase_started = time.monotonic()
            print(f"{phase.name}: {phase.command:+.3f}", flush=True)
            while time.monotonic() - phase_started < phase.duration:
                if interrupted:
                    raise RuntimeError("interrupted")
                for _ in range(6):
                    rclpy.spin_once(node, timeout_sec=0.0)
                if node.abort_reason:
                    raise RuntimeError(node.abort_reason)
                if time.monotonic() - node.last_odom_received > 0.5:
                    raise RuntimeError("/odom became stale")
                node.publish(phase.command)
                phase_age = time.monotonic() - phase_started
                if phase_age > 0.75 and args.command_topic != "/cmd_vel":
                    actual = (node.last_cmd.linear.x if args.axis == "linear"
                              else node.last_cmd.angular.z)
                    if time.monotonic() - node.last_cmd_received > 0.5:
                        raise RuntimeError("no fresh /cmd_vel from twist_mux")
                    if abs(actual - phase.command) > 0.03:
                        raise RuntimeError(
                            f"twist_mux output differs from request ({actual:+.3f} vs "
                            f"{phase.command:+.3f}); another higher-priority source may be active"
                        )
                row = node.row(time.monotonic() - started, phase)
                if row:
                    rows.append(row)
                time.sleep(1.0 / SAMPLE_HZ)
        node.publish_zero()
    except Exception as exc:
        node.publish_zero()
        print(f"ABORTED: {exc}", file=sys.stderr)
        return 2
    finally:
        node.publish_zero(0.3)
        node.destroy_node()
        rclpy.shutdown()

    if not rows:
        print("No samples recorded", file=sys.stderr)
        return 2
    result = analyze(rows, args.axis)
    for key in ("steady_gain_median", "delay_s_median", "time_constant_s_median",
                "right_left_speed_asymmetry_median"):
        if key in result:
            result[key] = finite_or_none(result[key])
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    prefix = Path(args.output_dir) / f"{timestamp}_{args.axis}"
    csv_path, json_path = write_results(rows, result, prefix)
    print(json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False))
    print(f"CSV: {csv_path}\nJSON: {json_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
