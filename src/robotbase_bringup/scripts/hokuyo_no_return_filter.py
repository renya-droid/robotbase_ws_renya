#!/usr/bin/env python3
"""Republish Hokuyo scans with no-return values replaced by inf.

Hokuyoの無反射値をinfに置き換えて再配信する。

屋外ではHokuyo(urg_node2)の光線の多くが無反射になり、range_max(30m)を超える
値(65.533m)で配信される。Nav2のObstacleLayerはrange_maxを超える光線を捨てるため、
その方向はクリアされず、通り過ぎた歩行者や自転車の跡が残る。

この値をinfに置き換え、ObstacleLayer側で `inf_is_valid: true` にすると、
無反射の方向も raytrace_max_range までクリアされる。
range_max以下の値、range_min未満の値、NaNは変更しない。
"""

from array import array

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


def replace_no_return(ranges, range_max, threshold=0.0):
    """Return ranges with finite values above range_max (or threshold) set to inf.

    戻り値は (置換後の配列, 置換した数)。
    """
    values = np.asarray(ranges, dtype=np.float32)
    limit = threshold if threshold > 0.0 else range_max
    no_return = np.isfinite(values) & (values > limit)
    if not no_return.any():
        return values, 0
    values = values.copy()
    values[no_return] = np.inf
    return values, int(no_return.sum())


class HokuyoNoReturnFilter(Node):

    def __init__(self):
        super().__init__('hokuyo_no_return_filter')
        input_topic = self.declare_parameter('input_topic', '/hokuyo_scan').value
        output_topic = self.declare_parameter(
            'output_topic', '/hokuyo_scan_filtered').value
        # 0以下ならメッセージのrange_maxを使う。
        self.threshold = float(self.declare_parameter('threshold', 0.0).value)
        # 信頼性はreliableにして、best effortでもreliableでも購読できるようにする。
        self.publisher = self.create_publisher(
            LaserScan, output_topic,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.subscription = self.create_subscription(
            LaserScan, input_topic, self.callback, qos_profile_sensor_data)
        self.get_logger().info(
            f'{input_topic} -> {output_topic}: range_max超の無反射値をinfに置換')

    def callback(self, msg):
        values, _ = replace_no_return(msg.ranges, msg.range_max, self.threshold)
        msg.ranges = array('f', values.tobytes())
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = HokuyoNoReturnFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
