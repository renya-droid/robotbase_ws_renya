#!/usr/bin/env python3
"""Repeated full PoseArray snapshots. No IDs; positions is [x1,y1,x2,y2,...]."""
import math
import time

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


def validate(positions, frame_id):
    # Jazzy startup overrides can represent [] as NOT_SET; param set can use BYTE_ARRAY.
    if positions is None:
        positions = []
    if not isinstance(positions, (list, tuple, bytes, bytearray)) and not hasattr(positions, 'tolist'):
        raise ValueError('positions must be an array [x1,y1,x2,y2,...] or []')
    if len(positions) % 2 or any(isinstance(v, bool) or not math.isfinite(float(v)) for v in positions):
        raise ValueError('positions must contain finite x,y pairs')
    if not frame_id or frame_id.startswith('/') or any(c.isspace() for c in frame_id):
        raise ValueError('frame_id must be nonempty, without whitespace or a leading slash')


class ManualPeople(Node):
    def __init__(self):
        super().__init__('manual_external_camera_people')
        topic = self.declare_parameter('topic', '/external_camera/people',
            ParameterDescriptor(read_only=True)).value
        rate = self.declare_parameter('publish_rate', 5.0,
            ParameterDescriptor(read_only=True)).value
        self.declare_parameter('positions', [-2.0, -4.0], ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('frame_id', 'map')
        validate(self.get_parameter('positions').value, self.get_parameter('frame_id').value)
        if not math.isfinite(rate) or rate <= 0:
            raise ValueError('publish_rate must be finite and > 0')
        self.publisher = self.create_publisher(PoseArray, topic, QoSProfile(depth=1,
            reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE))
        self.add_on_set_parameters_callback(self.validate_parameters)
        self.timer = self.create_timer(1.0/rate, self.publish, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.last_warning = float('-inf')
        self.get_logger().info(f'Full snapshots on {topic} at {rate:g} Hz; stop the camera/video publisher first.')

    def validate_parameters(self, parameters):
        values = {key: self.get_parameter(key).value for key in ('positions', 'frame_id')}
        values.update({p.name: p.value for p in parameters if p.name in values})
        try:
            validate(values['positions'], values['frame_id'])
            return SetParametersResult(successful=True)
        except (TypeError, ValueError) as exc:
            return SetParametersResult(successful=False, reason=str(exc))

    def publish(self):
        msg = PoseArray()
        msg.header.frame_id = self.get_parameter('frame_id').value
        msg.header.stamp = self.get_clock().now().to_msg()
        values = self.get_parameter('positions').value or []
        for i in range(0, len(values), 2):
            pose = Pose()
            pose.position.x, pose.position.y = float(values[i]), float(values[i+1])
            pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.publisher.publish(msg)
        if self.count_publishers(self.publisher.topic_name) > 1 and time.monotonic()-self.last_warning > 5:
            self.get_logger().warning('Multiple publishers on people topic: full snapshots overwrite each other. Stop one.')
            self.last_warning = time.monotonic()


def main():
    rclpy.init()
    node = None
    try:
        node = ManualPeople()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
