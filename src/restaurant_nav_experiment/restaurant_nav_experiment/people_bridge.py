"""Ground-truth "external camera": restaurant snapshot -> PoseArrays for Nav2.

Publishes in the Nav2 map frame (map = world - base):
  /external_camera/people          every active person    (condition B1)
  /external_camera/people_walking  everyone not SEATED    (condition P)
  /external_camera/people_seated   SEATED people          (condition P)
and feeds the robot pose back to the people controller:
  /restaurant/robot_odom (nav_msgs/Odometry, frame world)
Positions are the simulator's internal state, i.e. a perfect observer; this
stage measures the upper bound of the benefit, not camera accuracy.
"""
import json
import math

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException


def split_agents(snapshot, base):
    """Return (all, walking, seated) lists of map-frame (x, y, yaw)."""
    everyone, walking, seated = [], [], []
    for a in snapshot['agents']:
        if not a['active']:
            continue
        p = (a['position']['x'] - base[0], a['position']['y'] - base[1], a.get('yaw', 0.0))
        everyone.append(p)
        (seated if a['posture'] == 'SEATED' else walking).append(p)
    return everyone, walking, seated


class PeopleBridge(Node):
    def __init__(self):
        super().__init__('restaurant_people_bridge')
        self.declare_parameter('base_x', 0.0)
        self.declare_parameter('base_y', 0.0)
        self.declare_parameter('robot_base_frame', 'robot/base_footprint')
        self.base = (self.get_parameter('base_x').value, self.get_parameter('base_y').value)
        self.frame = self.get_parameter('robot_base_frame').value
        # Same QoS as external_camera_people_layer: reliable, volatile, depth 1.
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.pubs = {k: self.create_publisher(PoseArray, f'/external_camera/{k}', qos)
                     for k in ('people', 'people_walking', 'people_seated')}
        self.odom_pub = self.create_publisher(Odometry, '/restaurant/robot_odom', 10)
        self.create_subscription(String, '/restaurant/agents_snapshot', self.on_snapshot, 10)
        self.tf = Buffer(); self.listener = TransformListener(self.tf, self)
        self.create_timer(0.1, self.publish_robot)

    def on_snapshot(self, msg):
        everyone, walking, seated = split_agents(json.loads(msg.data), self.base)
        stamp = self.get_clock().now().to_msg()
        for key, people in (('people', everyone), ('people_walking', walking), ('people_seated', seated)):
            out = PoseArray(); out.header.frame_id = 'map'; out.header.stamp = stamp
            for x, y, yaw in people:
                p = Pose(); p.position.x, p.position.y = x, y
                p.orientation.z, p.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
                out.poses.append(p)
            self.pubs[key].publish(out)

    def publish_robot(self):
        try:
            t = self.tf.lookup_transform('map', self.frame, rclpy.time.Time())
        except TransformException:
            return
        o = Odometry(); o.header.stamp = t.header.stamp; o.header.frame_id = 'world'
        o.child_frame_id = self.frame
        o.pose.pose.position.x = t.transform.translation.x + self.base[0]
        o.pose.pose.position.y = t.transform.translation.y + self.base[1]
        o.pose.pose.orientation = t.transform.rotation
        self.odom_pub.publish(o)


def main():
    rclpy.init(); node = PeopleBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
