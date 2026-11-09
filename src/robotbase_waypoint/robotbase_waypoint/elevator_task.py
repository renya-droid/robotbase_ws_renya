"""Correlated elevator task; waypoint navigation stays idle during handoff."""
import json
import math
from time import monotonic

from std_msgs.msg import String
from std_srvs.srv import Trigger


class ElevatorTaskLink:
    """Transfer route ownership until the correlated elevator exit completes."""

    def _configure_elevator(self):
        self.elevator_task = None
        if not any(w.elevator for w in self.waypoints):
            return
        if hasattr(self, 'elevator_client'):
            return
        self.declare_parameter('elevator_timeout_sec', 300.0)
        self.elevator_timeout_sec = float(self.get_parameter('elevator_timeout_sec').value)
        if not math.isfinite(self.elevator_timeout_sec) or self.elevator_timeout_sec <= 0:
            raise ValueError('elevator_timeout_sec must be positive and finite')
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
        if now - task['began'] > self.elevator_timeout_sec:
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
        task.update(session=session, sequence=sequence, last_state_at=monotonic())
        result = state.get('waypoint_cycle_result', '')
        if isinstance(result, str) and result.startswith('NG_'):
            self._elevator_fault(f'Elevator cycle failed: {result}')
            return
        safety = state.get('safety', {})
        if (result == 'GOAL_REACHED' and state.get('finished') is True
                and state.get('phase') == 'FINISHED' and state.get('nav_unresolved') is False
                and isinstance(safety, dict) and safety.get('doorway_region') == 'OUTSIDE'
                and state.get('normal_profile_ready') is True):
            if not self._commit_state(elevator_task=None):
                return
            self.get_logger().info('Elevator exit completed; continuing waypoints')
            if self.index == len(self.waypoints) - 1:
                self._complete_or_loop()
            else:
                self._advance()

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
