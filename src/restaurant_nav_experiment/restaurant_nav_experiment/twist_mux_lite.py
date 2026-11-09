"""Minimal stand-in for twist_mux (not installed on this PC), simulation only.

Reads the same parameters as params/sim/twist_mux.yaml: topics.<name>.{topic,
timeout, priority} and locks.<name>.{topic, timeout, priority}. Publishes the
highest-priority input received within its timeout on cmd_vel_out; a lock
(std_msgs/Bool true, timeout 0 = no expiry) blocks inputs with lower priority.
Unlike twist_mux it does not publish when no input is active.
"""
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool


class TwistMuxLite(Node):
    def __init__(self):
        super().__init__('twist_mux', allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)
        self.inputs, self.locks = {}, {}
        params = self.get_parameters_by_prefix('')
        for key, value in params.items():
            parts = key.split('.')
            if len(parts) == 3 and parts[0] in ('topics', 'locks'):
                group = self.inputs if parts[0] == 'topics' else self.locks
                group.setdefault(parts[1], {})[parts[2]] = value.value
        self.pub = self.create_publisher(Twist, 'cmd_vel_out', 10)
        self.state = {}
        for name, cfg in self.inputs.items():
            self.create_subscription(Twist, cfg['topic'], lambda m, n=name: self.on_twist(n, m), 10)
        for name, cfg in self.locks.items():
            self.create_subscription(Bool, cfg['topic'], lambda m, n=name: self.on_lock(n, m), 10)
        self.get_logger().info(f'inputs {sorted(self.inputs)} locks {sorted(self.locks)}')

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def lock_priority(self):
        t, best = self.now(), -1
        for name, cfg in self.locks.items():
            on, stamp = self.state.get(('lock', name), (False, None))
            if on and (cfg.get('timeout', 0.0) == 0.0 or t - stamp <= cfg['timeout']):
                best = max(best, int(cfg['priority']))
        return best

    def on_lock(self, name, msg):
        self.state[('lock', name)] = (msg.data, self.now())

    def on_twist(self, name, msg):
        t = self.now()
        self.state[name] = (msg, t)
        prio = int(self.inputs[name]['priority'])
        if prio <= self.lock_priority():
            return
        for other, cfg in self.inputs.items():
            if other == name or int(cfg['priority']) <= prio:
                continue
            got = self.state.get(other)
            if got and t - got[1] <= float(cfg['timeout']):
                return  # a higher-priority input is still active
        self.pub.publish(msg)


def main():
    rclpy.init(); node = TwistMuxLite()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
