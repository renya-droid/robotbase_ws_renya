#!/usr/bin/env python3
"""Exit successfully only after exactly one simulation clock advances and odom arrives."""
import argparse
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--timeout', type=float, default=45.0)
    args = parser.parse_args()
    rclpy.init(args=[])
    node = rclpy.create_node('koko_sim_ready_probe')
    first = latest = None
    clock_at = odom_at = float('-inf')
    def clock(msg):
        nonlocal first, latest, clock_at
        stamp = msg.clock.sec*1_000_000_000+msg.clock.nanosec
        if first is None or (latest is not None and stamp < latest):
            first = stamp
        latest = stamp
        clock_at = time.monotonic()
    def odom(_):
        nonlocal odom_at
        odom_at = time.monotonic()
    node.create_subscription(Clock, '/clock', clock, qos_profile_sensor_data)
    node.create_subscription(Odometry, '/odom', odom, qos_profile_sensor_data)
    try:
        until = time.monotonic()+args.timeout
        while rclpy.ok() and time.monotonic()<until:
            rclpy.spin_once(node, timeout_sec=.1)
            now = time.monotonic()
            if node.count_publishers('/clock') > 1:
                print('/clock配信元が複数あります。シミュレーション一式を終了してください。', flush=True)
                return 2
            if (first is not None and latest > first and now-clock_at < 1.0 and
                    now-odom_at < 1.0 and node.count_publishers('/clock') == 1):
                print('Gazebo準備完了：/clockの進行と/odomの受信を確認しました。', flush=True)
                return 0
        print('Gazeboが準備できないため後続の起動を中止しました。Gazebo端末のエラーを確認してください。', flush=True)
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
