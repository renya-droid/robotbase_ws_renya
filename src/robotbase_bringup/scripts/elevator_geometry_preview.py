#!/usr/bin/env python3
"""Display configured cabin geometry only for the map currently served; no control."""
from pathlib import Path
from copy import deepcopy
import math
import sys
import time
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rcl_interfaces.srv import GetParameters
from geometry_msgs.msg import Point
from std_msgs.msg import String
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


def config_on_map(active_map, path):
    """Resolve departure or arrival geometry without changing saved parameters."""
    config = yaml.safe_load(path.read_text())
    entry = config['elevator_entry_map_diff']['ros__parameters']
    expected = entry.get('map_yaml_path')
    if expected and Path(expected).expanduser().resolve() == active_map:
        return config
    reference = (entry.get('arrival_geometry_file') if entry.get('switch_map_before_exit')
                 else entry.get('exit_map_geometry_file') if entry.get('switch_map_after_exit')
                 else None)
    if not reference:
        return None
    arrival_path = Path(reference).expanduser()
    if not arrival_path.is_absolute():
        arrival_path = path.parent / arrival_path
    arrival = yaml.safe_load(arrival_path.read_text())['arrival_geometry']
    expected = arrival.get('map_yaml_path')
    if not expected or Path(expected).expanduser().resolve() != active_map:
        return None
    # Departure-only bank data must not override the arrival outline.
    result = deepcopy(config)
    params = result['elevator_entry_map_diff']['ros__parameters']
    params.update(arrival)
    params.pop('elevator_bank_config', None)
    return result


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
        config = config_on_map(active, path)
        if config is not None:
            matches.append(config)
    return matches[0] if len(matches) == 1 else None


def selected_config_for_map(active_map, current_path):
    """Use the elevator GUI's explicit choice when maps share the same image."""
    selection = current_path.parent.parent / 'records/rviz_profile.txt'
    if not selection.is_file():
        return config_for_map(active_map, current_path), current_path
    name = selection.read_text().strip()
    if not name or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for c in name):
        return None, None
    path = (current_path if name == 'current' else
            current_path.parent / 'profiles' / name / 'entry.local.yaml')
    if not path.is_file():
        return None, None
    if not active_map:
        return None, None
    config = config_on_map(Path(active_map).expanduser().resolve(), path)
    if config is None:
        return None, None
    return config, path


def markers_for_config(config, path, stamp):
    p = config['elevator_entry_map_diff']['ros__parameters']
    bank_file = p.get('elevator_bank_config')
    if bank_file:
        bank_path = Path(bank_file).expanduser()
        if not bank_path.is_absolute():
            bank_path = path.parent / bank_path
        bank = yaml.safe_load(bank_path.read_text())
        source = next((parent / 'src/elevator_space_navigation' for parent in path.parents
                       if (parent / 'src/elevator_space_navigation').is_dir()), None)
        if source is None:
            raise ValueError('Elevator marker source workspace not found')
        if str(source) not in sys.path:
            sys.path.insert(0, str(source))
        from elevator_space_navigation.elevator_bank_ros import bank_geometry_markers
        door = config.get('elevator_door_node', {}).get('ros__parameters', {})
        return bank_geometry_markers(
            {e['id']: e for e in bank['elevators']}, None, stamp,
            door_detection_x_margin=door.get('door_detection_x_margin', 0.12),
            door_closed_band=door.get('door_closed_band', 0.15),
            door_open_depth=door.get('door_open_depth', 0.40))
    markers = []
    for i, points in enumerate(geometry(config)):
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = stamp
        m.ns, m.id = 'configured_elevator_preview', i
        m.type, m.action = Marker.LINE_STRIP, Marker.ADD
        m.pose.orientation.w = 1.
        m.scale.x = .05
        m.color.r, m.color.g, m.color.b, m.color.a = ((1., .6, 0., 1.) if i == 0 else (0., .8, 1., 1.))
        m.points = [Point(x=float(x), y=float(y), z=.12) for x, y in points]
        markers.append(m)
    return markers


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
        self.last_control_state_at = None
        self.create_subscription(String, '/elevator_entry_state',
                                 self.control_state, 10)
        self.timer = self.create_timer(1., self.tick)

    def control_state(self, _message):
        self.last_control_state_at = time.monotonic()

    def clear(self):
        # This topic has no other publishers; clear all previous preview IDs.
        self.pub.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))

    def tick(self):
        # The live controller publishes the geometry for its actual profile.
        if (getattr(self, 'last_control_state_at', None) is not None
                and time.monotonic() - self.last_control_state_at < 3.):
            self.clear()
            return
        if self.pending is not None:
            if self.pending.done():
                pending, self.pending = self.pending, None
                try:
                    reply = pending.result()
                    active = reply.values[0].string_value
                    config, config_path = selected_config_for_map(active, self.path)
                    if config is None:
                        self.clear()
                        return
                    markers = markers_for_config(config, config_path, self.get_clock().now().to_msg())
                    self.pub.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL), *markers]))
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
