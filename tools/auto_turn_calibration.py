#!/usr/bin/env python3
"""Command one bounded turn using filtered odometry, and report both odometries."""

import argparse
import math
import signal
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


class TurnState:
    def __init__(self):
        self.previous = None
        self.total = 0.0
        self.twist = 0.0
        self.received = 0.0
        self.count = 0

    def update(self, msg):
        angle = yaw(msg.pose.pose.orientation)
        if self.previous is not None:
            self.total += math.atan2(math.sin(angle - self.previous),
                                     math.cos(angle - self.previous))
        self.previous = angle
        self.twist = msg.twist.twist.angular.z
        self.received = time.monotonic()
        self.count += 1


class AutoTurn(Node):
    def __init__(self):
        super().__init__("auto_turn_calibration")
        self.pub = self.create_publisher(Twist, "/cmd_vel_direct", 10)
        self.raw = TurnState()
        self.filtered = TurnState()
        self.create_subscription(Odometry, "/odom", self.raw.update, 50)
        self.create_subscription(Odometry, "/odom/filtered", self.filtered.update, 50)
        self.create_subscription(Twist, "/cmd_vel", self.on_command, 10)
        self.create_subscription(Bool, "/stop", self.on_stop, 10)
        self.command = None
        self.command_received = 0.0
        self.stop_active = False

    def on_command(self, msg):
        self.command = msg
        self.command_received = time.monotonic()

    def on_stop(self, msg):
        self.stop_active = bool(msg.data)

    def publish(self, angular):
        msg = Twist()
        msg.angular.z = angular
        self.pub.publish(msg)

    def stop(self):
        deadline = time.monotonic() + 1.0
        while rclpy.ok() and time.monotonic() < deadline:
            self.publish(0.0)
            rclpy.spin_once(self, timeout_sec=0.05)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("direction", choices=("left", "right"))
    parser.add_argument("--run", action="store_true", help="send motor commands")
    parser.add_argument("--speed", type=float, default=0.4,
                        help="angular speed command in rad/s, default 0.4")
    parser.add_argument("--max-duration", type=float, default=35.0)
    args = parser.parse_args()
    if not 0.2 <= args.speed <= 0.5:
        parser.error("speed must be between 0.2 and 0.5 rad/s")
    if not 15 <= args.max_duration <= 60:
        parser.error("max-duration must be between 15 and 60 seconds")
    if not args.run:
        print("DRY RUN: filtered odomで約360度の旋回を指令します。実行するには --run を付けてください。")
        return 0

    rclpy.init()
    node = AutoTurn()
    interrupted = False

    def request_stop(_signum, _frame):
        nonlocal interrupted
        interrupted = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    direction = 1 if args.direction == "left" else -1
    command = direction * args.speed
    try:
        deadline = time.monotonic() + 8.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.raw.count and node.filtered.count and node.pub.get_subscription_count():
                break
        if not node.raw.count or not node.filtered.count:
            raise RuntimeError("/odom または /odom/filtered が届きません")
        if node.pub.get_subscription_count() == 0:
            raise RuntimeError("/cmd_vel_direct に購読者がいません（twist_mux を確認）")
        if node.stop_active:
            raise RuntimeError("/stop が有効です")
        if abs(node.filtered.twist) > 0.05:
            raise RuntimeError("ロボットがまだ旋回しています。停止後に実行してください")

        # Reset integrated angles once both topics are flowing and the robot is still.
        node.raw.total = 0.0
        node.filtered.total = 0.0
        node.raw.count = 0
        node.filtered.count = 0
        print("3秒後に自動旋回を開始します。周囲を空けてください。Ctrl-Cで停止します。", flush=True)
        deadline = time.monotonic() + 3.0
        while rclpy.ok() and time.monotonic() < deadline:
            if interrupted or node.stop_active:
                raise RuntimeError("中止しました")
            rclpy.spin_once(node, timeout_sec=0.1)

        started = time.monotonic()
        while rclpy.ok():
            now = time.monotonic()
            if interrupted or node.stop_active:
                raise RuntimeError("中止しました")
            if now - started > args.max_duration:
                raise RuntimeError("制限時間を超えました")
            if now - node.filtered.received > 0.5 or now - node.raw.received > 0.5:
                raise RuntimeError("odom が更新されなくなりました")
            if now - started > 1.0:
                if now - node.command_received > 0.5 or node.command is None:
                    raise RuntimeError("twist_mux から /cmd_vel が届きません")
                if abs(node.command.angular.z - command) > 0.05 or abs(node.command.linear.x) > 0.05:
                    raise RuntimeError("/cmd_vel が指令と異なります。キーオペ等の優先入力を確認")

            progress = direction * node.filtered.total
            # Account for the robot continuing to rotate while the zero command takes effect.
            stopping_angle = max(math.radians(3), abs(node.filtered.twist) * 0.20)
            if progress >= 2 * math.pi - stopping_angle:
                break
            node.publish(command)
            rclpy.spin_once(node, timeout_sec=0.05)

        node.stop()
        settle_deadline = time.monotonic() + 1.5
        while rclpy.ok() and time.monotonic() < settle_deadline:
            if interrupted or node.stop_active:
                break
            rclpy.spin_once(node, timeout_sec=0.05)
        print(f"/odom          : {math.degrees(node.raw.total):+.2f} deg")
        print(f"/odom/filtered : {math.degrees(node.filtered.total):+.2f} deg")
        print("床の開始方向の印と最終方向を比べ、実際の角度も確認してください。")
    except (RuntimeError, KeyboardInterrupt) as exc:
        print(f"旋回中止: {exc}", file=sys.stderr)
        return 2
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
