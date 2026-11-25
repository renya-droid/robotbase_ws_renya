#!/usr/bin/env python3
"""Display configured cabin geometry only for the map currently served; no control."""
from pathlib import Path
import math
import time
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rcl_interfaces.srv import GetParameters
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


def geometry(config):
    p = config['elevator_entry_map_diff']['ros__parameters']
    left = (float(p['door_left_x']), float(p['door_left_y']))
    right = (float(p['door_right_x']), float(p['door_right_y']))
    if p.get('use_elevator_polygon'):
        cabin = list(zip(p['elevator_polygon_x'], p['elevator_polygon_y']))
        if len(cabin) < 3 or len(p['elevator_polygon_x']) != len(p['elevator_polygon_y']):
            raise ValueError('Invalid cabin polygon')
    else:
        dx, dy = right[0]-left[0], right[1]-left[1]
        length = math.hypot(dx, dy)
        if length <= 0:
            raise ValueError('Door has zero width')
        ex, ey = dx/length, dy/length
        sign = p['elevator_inside_sign']
        cx, cy = (left[0]+right[0])/2, (left[1]+right[1])/2
        cabin = [(cx+ex*x-ey*sign*y, cy+ey*x+ex*sign*y)
                 for x, y in [(p['elevator_local_x_min'], p['elevator_local_y_min']),
                              (p['elevator_local_x_max'], p['elevator_local_y_min']),
                              (p['elevator_local_x_max'], p['elevator_local_y_max']),
                              (p['elevator_local_x_min'], p['elevator_local_y_max'])]]
    if not all(math.isfinite(float(v)) for point in [left, right, *cabin] for v in point):
        raise ValueError('Nonfinite coordinates')
    return [(left, right), cabin+[cabin[0]]]


def config_for_map(active_map, current_path):
    """Show a preview only when exactly one saved profile owns the map."""
    if not active_map:
        return None
    active = Path(active_map).expanduser().resolve()
    paths = [current_path]
    if current_path.parent.name == 'config':
        paths.extend(sorted((current_path.parent / 'profiles').glob('*/entry.local.yaml')))
    matches = []
    for path in paths:
        if not path.is_file():
            continue
        if path != current_path:
            site = path.parent / 'site.yaml'
            if not site.is_file():
                continue
            readiness = yaml.safe_load(site.read_text()) or {}
            if not (readiness.get('measurements_reviewed') is True or
                    readiness.get('mock_geometry_reviewed') is True):
                continue
        config = yaml.safe_load(path.read_text())
        expected = config['elevator_entry_map_diff']['ros__parameters'].get('map_yaml_path')
        if expected and Path(expected).expanduser().resolve() == active:
            matches.append(config)
    return matches[0] if len(matches) == 1 else None


class Preview(Node):
    def __init__(self):
        super().__init__('elevator_geometry_preview')
        self.declare_parameter('config_file', str(Path.home()/'kuwa_ws2/config/entry.local.yaml'))
        self.path = Path(self.get_parameter('config_file').value).expanduser()
        self.client = self.create_client(GetParameters, '/map_server/get_parameters')
        self.pub = self.create_publisher(MarkerArray, '/elevator_geometry_preview_markers',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.pending = None
        self.started = 0.
        self.timer = self.create_timer(1., self.tick)

    def clear(self):
        # This topic is separate from control markers; delete only preview IDs.
        self.pub.publish(MarkerArray(markers=[Marker(ns='configured_elevator_preview', id=i,
                                                    action=Marker.DELETE) for i in range(2)]))

    def tick(self):
        if self.pending is not None:
            if self.pending.done():
                pending, self.pending = self.pending, None
                try:
                    reply = pending.result()
                    active = reply.values[0].string_value
                    config = config_for_map(active, self.path)
                    if config is None:
                        self.clear()
                        return
                    markers = []
                    for i, points in enumerate(geometry(config)):
                        m = Marker()
                        m.header.frame_id = 'map'
                        m.header.stamp = self.get_clock().now().to_msg()
                        m.ns, m.id = 'configured_elevator_preview', i
                        m.type, m.action = Marker.LINE_STRIP, Marker.ADD
                        m.pose.orientation.w = 1.
                        m.scale.x = .05
                        m.color.r, m.color.g, m.color.b, m.color.a = ((1., .6, 0., 1.) if i == 0 else (0., .8, 1., 1.))
                        m.points = [Point(x=float(x), y=float(y), z=.12) for x, y in points]
                        markers.append(m)
                    self.pub.publish(MarkerArray(markers=markers))
                except Exception as exc:
                    self.clear()
                    self.get_logger().warning(str(exc), throttle_duration_sec=10.)
            elif time.monotonic()-self.started > 3.:
                self.client.remove_pending_request(self.pending)
                self.pending = None
                self.clear()
            return
        if self.client.service_is_ready() and self.path.is_file():
            self.started = time.monotonic()
            self.pending = self.client.call_async(GetParameters.Request(names=['yaml_filename']))
        else:
            self.clear()


def main():
    rclpy.init()
    node = Preview()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.clear()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
