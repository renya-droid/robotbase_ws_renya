"""Ground-truth odom -> base_footprint TF from Gazebo (simulation only).

Static people models are teleported, so a person can be placed against the
robot; wheel slip then corrupts DiffDrive odometry and, with Nav2's
localization:=static (map->odom identity), the robot plans from a wrong pose.
This node replaces the DiffDrive TF with the model's true pose relative to the
spawn pose, so map == spawn-relative ground truth and the experiment compares
planning, not localisation. /odom (velocities) still comes from DiffDrive.
"""
import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster


class GroundTruthTF(Node):
    def __init__(self):
        super().__init__('restaurant_ground_truth_tf')
        for name, default in (('model', 'robot'), ('spawn_x', 0.0), ('spawn_y', 0.0),
                              ('odom_frame', 'robot/odom'), ('base_frame', 'robot/base_footprint')):
            self.declare_parameter(name, default)
        g = lambda k: self.get_parameter(k).value
        self.model, self.spawn = g('model'), (g('spawn_x'), g('spawn_y'))
        self.odom, self.base = g('odom_frame'), g('base_frame')
        self.br = TransformBroadcaster(self)
        self.create_subscription(TFMessage, '/restaurant/robot_truth', self.on_poses, 10)
        self.warned = False

    def pick(self, msg):
        """The robot model's world pose.

        ros_gz_bridge leaves child_frame_id empty for dynamic_pose/info, so the
        name cannot be matched. Link poses there are relative to their model
        (within 0.3 m of 0) while the model pose is in world coordinates and the
        robot is always inside the restaurant (>1 m from the world origin), so
        the entry farthest from the origin is the model."""
        named = [t for t in msg.transforms if t.child_frame_id == self.model]
        if named:
            return named[0]
        far = max(msg.transforms, key=lambda t: math.hypot(t.transform.translation.x, t.transform.translation.y),
                  default=None)
        if far is not None and math.hypot(far.transform.translation.x, far.transform.translation.y) > 1.0:
            return far
        return None

    def on_poses(self, msg):
        for t in [self.pick(msg)]:
            if t is None:
                continue
            out = TransformStamped()
            stamp = t.header.stamp
            out.header.stamp = stamp if (stamp.sec or stamp.nanosec) else self.get_clock().now().to_msg()
            out.header.frame_id, out.child_frame_id = self.odom, self.base
            out.transform.translation.x = t.transform.translation.x - self.spawn[0]
            out.transform.translation.y = t.transform.translation.y - self.spawn[1]
            q = t.transform.rotation
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
            out.transform.rotation.z, out.transform.rotation.w = math.sin(yaw/2), math.cos(yaw/2)
            self.br.sendTransform(out)
            return
        if not self.warned:
            self.warned = True
            self.get_logger().warning(f'No robot pose in /restaurant/robot_truth ({len(msg.transforms)} entries)')


def main():
    rclpy.init(); node = GroundTruthTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
