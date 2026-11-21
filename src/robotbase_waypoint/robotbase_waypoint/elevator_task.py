"""Correlated elevator task; waypoint navigation stays idle during handoff."""
import json
import math
from pathlib import Path
from time import monotonic

from std_msgs.msg import String
from std_srvs.srv import Trigger

from .waypoint_io import load_waypoints
from .waypoint_visualize import route_markers


class ElevatorTaskLink:
    """Transfer route ownership until the correlated elevator exit completes."""

    def _configure_elevator(self):
        self.elevator_task = None
        if not any(w.elevator for w in self.waypoints):
            return
        if hasattr(self, 'elevator_client'):
            return
        self.declare_parameter('elevator_timeout_sec', 300.0)
        self.declare_parameter('elevator_arrival_map_file', '')
        self.declare_parameter('elevator_arrival_waypoint_file', '')
        self.elevator_timeout_sec = float(self.get_parameter('elevator_timeout_sec').value)
        if not math.isfinite(self.elevator_timeout_sec) or self.elevator_timeout_sec <= 0:
            raise ValueError('elevator_timeout_sec must be positive and finite')
        map_file = str(self.get_parameter('elevator_arrival_map_file').value)
        waypoint_file = str(self.get_parameter('elevator_arrival_waypoint_file').value)
        if bool(map_file) != bool(waypoint_file):
            raise ValueError('Both elevator arrival map and waypoint file are required')
        self.elevator_arrival_map_file = Path(map_file).expanduser().resolve() if map_file else None
        self.elevator_arrival_waypoints = None
        if self.elevator_arrival_map_file is not None:
            if not self.elevator_arrival_map_file.is_file():
                raise FileNotFoundError(self.elevator_arrival_map_file)
            self.elevator_arrival_waypoints = load_waypoints(waypoint_file)
            if any(w.elevator or w.change_map or w.person_area for w in self.elevator_arrival_waypoints):
                raise ValueError('Arrival waypoints must be ordinary navigation points')
        self.elevator_client = self.create_client(Trigger, '/elevator_cycle_start')
        self.elevator_cancel_publisher = self.create_publisher(String, '/elevator_manual_command', 10)
        self.elevator_state_subscription = self.create_subscription(
            String, '/elevator_entry_state', self._elevator_state, 10)

    def _start_elevator(self):
        if self._is_shutting_down() or getattr(self, 'elevator_task', None) is not None:
            return
        if any(not record.resolved for record in self.goal_records.values()):
            self._elevator_fault('Waypoint navigation has not terminated')
            return
        if not self._publish_stop(False):
            self._elevator_fault('Waypoint stop could not be released')
            return
        task = dict(stage='waiting_service', began=monotonic(), id=None, future=None,
                    session=None, sequence=-1, last_state_at=None)
        if self._commit_state(elevator_task=task, holding_stop=False):
            self.get_logger().info(f'[WP:{self.waypoints[self.index].number}] waiting for elevator cycle')

    def _continue_elevator(self):
        task = self.elevator_task
        if task['stage'] == 'fault' or self._is_shutting_down():
            return
        now = monotonic()
        elapsed = now - task['began'] - task.get('preboard_wait_total', 0.)
        if task.get('preboard_wait_since') is not None:
            elapsed -= now - task['preboard_wait_since']
        if elapsed > self.elevator_timeout_sec:
            self._elevator_fault('Elevator cycle timed out')
            return
        if task['stage'] == 'waiting_service':
            if self.elevator_client.service_is_ready():
                task['stage'] = 'request'
                task['request_at'] = now
                try:
                    self._commit_mission(self._send_elevator_request, task)
                except Exception as error:
                    self._elevator_fault(f'Elevator request failed: {error}')
            elif now - task['began'] > 8.0:
                self._elevator_fault('Start the elevator node with the waypoint profile first')
        elif task['stage'] == 'request':
            if task['future'] is not None and task['future'].done():
                try:
                    response = task['future'].result()
                    if not response.success or not response.message:
                        task['stage'] = 'refused'
                        self._elevator_fault(f'Elevator refused: {response.message}')
                        return
                    task.update(stage='cycle', id=response.message, last_state_at=now)
                except Exception as error:
                    self._elevator_fault(f'Elevator response failed: {error}')
            elif now - task['request_at'] > 8.0:
                self._elevator_fault('Elevator response timed out; request is not resent')
        elif task['stage'] == 'cycle' and now - task['last_state_at'] > 4.0:
            self._elevator_fault('Elevator state stream stopped')

    def _send_elevator_request(self, task):
        task['future'] = self.elevator_client.call_async(Trigger.Request())

    def _elevator_state(self, message):
        task = getattr(self, 'elevator_task', None)
        if self._is_shutting_down() or task is None or task['stage'] != 'cycle':
            return
        try:
            state = json.loads(message.data)
            if not isinstance(state, dict) or state.get('waypoint_cycle_id') != task['id']:
                return
            sequence, stamp = state['state_sequence'], state['stamp_ros']
            session = state['session_id']
            if (type(sequence) is not int or sequence <= task['sequence']
                    or type(stamp) not in (int, float) or not math.isfinite(stamp)
                    or not -.1 <= self.get_clock().now().nanoseconds / 1e9 - stamp <= 4.0
                    or not isinstance(session, str) or not session):
                return
        except (ValueError, KeyError, TypeError):
            return
        if task['session'] is not None and task['session'] != session:
            self._elevator_fault('Elevator node restarted during cycle')
            return
        now = monotonic()
        task.update(session=session, sequence=sequence, last_state_at=now)
        # Arrival/next-car waiting is unbounded while correlated state stays live.
        # Keep the 4 s state deadline and the active journey's time budget.
        preboard = (state.get('phase') in ('WAIT_START', 'WAIT_INITIAL_OPEN',
                                          'WAIT_SKIP_CLOSE', 'WAIT_SKIP_OPEN')
                    and state.get('finished') is False and state.get('nav_unresolved') is False)
        if preboard and task.get('preboard_wait_since') is None:
            task['preboard_wait_since'] = now
        elif not preboard and task.get('preboard_wait_since') is not None:
            task['preboard_wait_total'] = task.get('preboard_wait_total', 0.) + now-task.pop('preboard_wait_since')
        result = state.get('waypoint_cycle_result', '')
        if isinstance(result, str) and result.startswith('NG_'):
            self._elevator_fault(f'Elevator cycle failed: {result}')
            return
        safety = state.get('safety', {})
        if (result == 'GOAL_REACHED' and state.get('finished') is True
                and state.get('phase') == 'FINISHED' and state.get('nav_unresolved') is False
                and isinstance(safety, dict)
                # The reported door region still uses departure-map geometry after a map switch.
                and (state.get('map_switched') is True
                     or safety.get('doorway_region') == 'OUTSIDE')
                and state.get('normal_profile_ready') is True):
            if state.get('map_switched') is True:
                if self.elevator_arrival_waypoints is not None:
                    if (not state.get('exit_map_yaml_path')
                            or Path(state['exit_map_yaml_path']).resolve() != self.elevator_arrival_map_file
                            or state.get('shutdown_requested') is not False
                            or safety.get('stop_latched') is not False
                            or safety.get('cancel_in_progress') is not False):
                        self._elevator_fault('Arrival map or safety state does not match the selected route')
                        return
                    self._resume_on_arrival_map()
                    return
                if self._commit_state(elevator_task=None, finished=True):
                    self.get_logger().info('Elevator exit completed on a new map; select a route for this map')
                return
            if not self._commit_state(elevator_task=None):
                return
            self.get_logger().info('Elevator exit completed; continuing waypoints')
            if self.index == len(self.waypoints) - 1:
                self._complete_or_loop()
            else:
                self._advance()

    def _resume_on_arrival_map(self):
        if any(not record.resolved for record in self.goal_records.values()):
            self._elevator_fault('Old waypoint goal has not terminated')
            return
        try:
            markers = route_markers(self.elevator_arrival_waypoints, self.map_frame)
        except Exception as error:
            self._elevator_fault(f'Arrival route markers failed: {error}')
            return
        if not self._commit_state(
                elevator_task=None, waypoints=self.elevator_arrival_waypoints,
                index=0, hokuyo_reached_index=-1, loop=False,
                goal_serial=self.goal_serial + 1, goal_handle=None,
                goal_result_future=None, single_goal_recovery=False,
                waiting_until=None, waiting_finishes_route=False):
            return
        if not self._commit_mission(setattr, self.route_markers, 'message', markers):
            return
        if not self._commit_mission(self.route_markers.publish):
            return
        if not self._publish_stop(False):
            self._elevator_fault('Could not release waypoint stop after map handoff')
            return
        self.get_logger().info('Elevator destination map ready; starting arrival waypoints')
        self._send_current_goal()

    def _elevator_fault(self, reason):
        if self._is_shutting_down():
            return
        task = getattr(self, 'elevator_task', None)
        if task is None:
            task = {}
        if not self._commit_state(elevator_task={**task, 'stage': 'fault'}, finished=True):
            return
        if task.get('id') is not None or task.get('stage') == 'request':
            self._commit_mission(self.elevator_cancel_publisher.publish, String(data='stop'))
        self.get_logger().error(reason + '; no next waypoint will be sent')
