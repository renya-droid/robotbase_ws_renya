#!/usr/bin/env python3
"""Visualize which VLP-16 returns /scan3 adopts and where Nav2 can clear.

The node re-implements the /scan3 selection of velodyne_laserscan.cpp
(per ring/bin "last point wins", then the nearest point within each ring's
max distance) so every point can be labelled as adopted or excluded.  It
compares its own result with the real /scan3 and reports any mismatch.

Published topics (all in the lidar frame unless noted):
  ~/rings            MarkerArray: per ring truncated cone (adopted range in
                     ring colour, excluded range grey) with rims, generator
                     lines, ground-contact curve, labels, and a side-view
                     profile (namespace 断面図); namespaces are per ring
  ~/adopted_points   PointCloud2: selected-ring points within max distance
  ~/excluded_points  PointCloud2: selected-ring points beyond max distance
  ~/scan3_points     PointCloud2: the 3D point that produced each scan3 bin
  ~/clearing_rays    MarkerArray: green = finite scan3 bin (Nav2 clears),
                     red = +Inf bin (no clearing with inf_is_valid=false)
  ~/clear_candidate  LaserScan: per bin max raw range over selected rings
                     (preview of a clearing-only scan; not used by Nav2)
  ~/status           std_msgs/String JSON summary
"""

import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import ColorRGBA, Header, String
from visualization_msgs.msg import Marker, MarkerArray

RING_COLORS = [
    (0.90, 0.10, 0.10), (0.95, 0.50, 0.05), (0.90, 0.85, 0.05), (0.20, 0.80, 0.20),
    (0.05, 0.75, 0.85), (0.20, 0.35, 0.95), (0.65, 0.25, 0.90), (0.95, 0.30, 0.65),
]


def rpy_matrix(roll, pitch):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    # R = Ry(pitch) * Rx(roll): lidar frame -> gravity-aligned frame (yaw ignored)
    return np.array([
        [cp, sp * sr, sp * cr],
        [0.0, cr, -sr],
        [-sp, cp * sr, cp * cr],
    ])


def color(rgb, alpha):
    return ColorRGBA(r=float(rgb[0]), g=float(rgb[1]), b=float(rgb[2]), a=float(alpha))


class VelodyneCoverageViz(Node):

    def __init__(self):
        super().__init__('velodyne_coverage_viz')
        p = self.declare_parameter
        self.ring_ids = [int(r) for r in p('ring_ids', [1, 2, 3, 4, 5, 6, 8]).value]
        self.ring_max = [float(d) for d in p(
            'ring_max_distances', [1.25, 1.50, 1.75, 2.00, 2.50, 3.50, 100.0]).value]
        self.resolution = float(p('resolution', 0.008726646).value)
        self.num_rings = int(p('num_rings', 16).value)
        self.vmin = math.radians(float(p('vertical_min_deg', -15.0).value))
        self.vmax = math.radians(float(p('vertical_max_deg', 15.0).value))
        self.sensor_height = float(p('sensor_height', 0.72).value)
        self.sensor_max_range = float(p('sensor_max_range', 30.0).value)
        self.cone_draw_range = float(p('cone_draw_range', 16.0).value)
        self.ray_stride = max(1, int(p('ray_stride', 4).value))
        self.inf_ray_length = float(p('inf_ray_length', 4.0).value)
        self.use_imu_tilt = bool(p('use_imu_tilt', False).value)
        self.raytrace_max_range = float(p('raytrace_max_range', 20.0).value)
        self.sensor_min_range = float(p('sensor_min_range', 0.20).value)
        # side-view profile plane (deg, lidar frame); +90 = robot's left
        self.profile_azimuth_deg = float(p('profile_azimuth_deg', 90.0).value)

        n = min(len(self.ring_ids), len(self.ring_max))
        # Same rule as std::map in the C++ node: later duplicates overwrite.
        self.ring_limit = {}
        for ring, dist in zip(self.ring_ids[:n], self.ring_max[:n]):
            self.ring_limit[ring] = dist
        self.kSize = int(round(2.0 * math.pi / self.resolution))
        step = (self.vmax - self.vmin) / max(self.num_rings - 1, 1)
        self.elev = [self.vmin + i * step for i in range(self.num_rings)]

        self.roll = 0.0
        self.pitch = 0.0
        self.computed = {}
        self.compared = 0
        self.unmatched_scan3 = 0
        self.last_mismatch = None
        self.max_mismatch = 0
        self.last_status = {}
        self.ring_frame = None

        self.pub_rings = self.create_publisher(MarkerArray, '~/rings', 1)
        self.pub_adopted = self.create_publisher(PointCloud2, '~/adopted_points', 2)
        self.pub_excluded = self.create_publisher(PointCloud2, '~/excluded_points', 2)
        self.pub_scan3_pts = self.create_publisher(PointCloud2, '~/scan3_points', 2)
        self.pub_rays = self.create_publisher(MarkerArray, '~/clearing_rays', 2)
        self.pub_candidate = self.create_publisher(LaserScan, '~/clear_candidate', 2)
        self.pub_status = self.create_publisher(String, '~/status', 2)

        self.create_subscription(
            PointCloud2, 'velodyne_points', self.on_cloud, qos_profile_sensor_data)
        self.create_subscription(LaserScan, 'scan3', self.on_scan3, 10)
        if self.use_imu_tilt:
            self.create_subscription(Imu, 'imu', self.on_imu, qos_profile_sensor_data)
        self.create_timer(1.0, self.publish_rings)
        self.create_timer(5.0, self.log_status)

        self.get_logger().info(
            'rings=%s max=%s height=%.2f m imu_tilt=%s' % (
                list(self.ring_limit), list(self.ring_limit.values()),
                self.sensor_height, self.use_imu_tilt))
        for ring, dist in self.ring_limit.items():
            e = self.elev[ring]
            ground = self.sensor_height / math.tan(-e) if e < 0 else math.inf
            beam_z = self.sensor_height + min(dist, 1e3) * math.tan(e)
            self.get_logger().info(
                'ring %d: elev %+.1f deg, ground contact %s, max %.2f m, '
                'beam height at max %.2f m' % (
                    ring, math.degrees(e),
                    '%.2f m' % ground if math.isfinite(ground) else 'none',
                    dist, beam_z))

    # ------------------------------------------------------------------ inputs
    def on_imu(self, msg):
        q = msg.orientation
        sinr = 2.0 * (q.w * q.x + q.y * q.z)
        cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        self.roll = math.atan2(sinr, cosr)
        sinp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
        self.pitch = math.asin(sinp)

    def on_scan3(self, msg):
        key = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        mine = self.computed.pop(key, None)
        if mine is None or len(msg.ranges) != self.kSize:
            self.unmatched_scan3 += 1
            return
        real = np.asarray(msg.ranges, dtype=np.float32)
        both_inf = np.isinf(real) & np.isinf(mine)
        close = np.isclose(real, mine, atol=1e-4)
        self.last_mismatch = int(np.sum(~(both_inf | close)))
        self.max_mismatch = max(self.max_mismatch, self.last_mismatch)
        self.compared += 1

    def on_cloud(self, msg):
        names = [f.name for f in msg.fields]
        if 'ring' not in names:
            self.get_logger().error('PointCloud2 has no ring field', throttle_duration_sec=5.0)
            return
        self.ring_frame = msg.header.frame_id
        cloud = point_cloud2.read_points(msg, field_names=('x', 'y', 'z', 'ring'), skip_nans=False)
        x = np.asarray(cloud['x'], dtype=np.float32)
        y = np.asarray(cloud['y'], dtype=np.float32)
        z = np.asarray(cloud['z'], dtype=np.float32)
        ring = np.asarray(cloud['ring'], dtype=np.int64)
        valid_ring = ring < 16
        x, y, z, ring = x[valid_ring], y[valid_ring], z[valid_ring], ring[valid_ring]

        with np.errstate(invalid='ignore'):
            dist = np.sqrt(x * x + y * y)
            bins = np.floor((np.arctan2(y, x) + np.float32(math.pi)) / np.float32(self.resolution))
        in_range = np.isfinite(bins) & (bins >= 0) & (bins < self.kSize)
        x, y, z, ring, dist = x[in_range], y[in_range], z[in_range], ring[in_range], dist[in_range]
        bins = bins[in_range].astype(np.int64)

        # Per ring/bin the C++ node keeps the LAST point (plain assignment).
        key = ring * self.kSize + bins
        rev_key = key[::-1]
        _, first_rev = np.unique(rev_key, return_index=True)
        keep = len(key) - 1 - first_rev
        raw = np.full((16, self.kSize), np.inf, dtype=np.float32)
        owner = np.full((16, self.kSize), -1, dtype=np.int64)
        raw[ring[keep], bins[keep]] = dist[keep]
        owner[ring[keep], bins[keep]] = keep

        scan3 = np.full(self.kSize, np.inf, dtype=np.float32)
        scan3_owner = np.full(self.kSize, -1, dtype=np.int64)
        candidate = np.full(self.kSize, -np.inf, dtype=np.float32)
        adopted_mask = np.zeros(len(x), dtype=bool)
        excluded_mask = np.zeros(len(x), dtype=bool)
        for r, max_d in self.ring_limit.items():
            if not 0 <= r < 16:
                continue
            row = raw[r]
            ok = row <= np.float32(max_d)
            better = ok & (row < scan3)
            scan3[better] = row[better]
            scan3_owner[better] = owner[r][better]
            finite = np.isfinite(row)
            candidate[finite] = np.maximum(candidate[finite], row[finite])
            sel = ring == r
            finite_pt = np.isfinite(dist)
            adopted_mask |= sel & finite_pt & (dist <= max_d)
            excluded_mask |= sel & finite_pt & (dist > max_d)
        candidate[~np.isfinite(candidate)] = np.inf

        header = msg.header
        self.pub_adopted.publish(self.make_cloud(header, x[adopted_mask], y[adopted_mask],
                                                 z[adopted_mask], ring[adopted_mask]))
        self.pub_excluded.publish(self.make_cloud(header, x[excluded_mask], y[excluded_mask],
                                                  z[excluded_mask], ring[excluded_mask]))
        idx = scan3_owner[scan3_owner >= 0]
        self.pub_scan3_pts.publish(self.make_cloud(header, x[idx], y[idx], z[idx], ring[idx]))
        self.publish_candidate(header, candidate)
        self.publish_rays(header, scan3)

        finite_scan3 = np.isfinite(scan3)
        status = {
            'stamp': header.stamp.sec + header.stamp.nanosec * 1e-9,
            'bins': int(self.kSize),
            'scan3_inf_ratio': float(1.0 - finite_scan3.mean()),
            'candidate_inf_ratio': float(1.0 - np.isfinite(candidate).mean()),
            'adopted_points': int(adopted_mask.sum()),
            'excluded_points': int(excluded_mask.sum()),
            'scan3_winner_by_ring': {
                str(r): int(np.sum(ring[idx] == r)) for r in self.ring_limit},
        }
        self.computed[(header.stamp.sec, header.stamp.nanosec)] = scan3
        while len(self.computed) > 20:
            self.computed.pop(next(iter(self.computed)))
        status['scan3_mismatch_bins_last'] = self.last_mismatch
        status['scan3_mismatch_bins_max'] = self.max_mismatch
        status['scan3_compared_frames'] = self.compared
        self.last_status = status
        self.pub_status.publish(String(data=json.dumps(status)))

    # ----------------------------------------------------------------- outputs
    def make_cloud(self, header, x, y, z, ring):
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='ring', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        data = np.ascontiguousarray(
            np.stack([x, y, z, ring.astype(np.float32)], axis=1), dtype=np.float32)
        msg = PointCloud2(header=header, height=1, width=int(data.shape[0]), fields=fields,
                          is_bigendian=False, point_step=16, row_step=16 * int(data.shape[0]),
                          is_dense=True)
        msg.data = data.tobytes()
        return msg

    def publish_candidate(self, header, candidate):
        scan = LaserScan()
        scan.header = header
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = self.resolution
        scan.range_min = 0.0
        scan.range_max = 200.0
        scan.ranges = candidate.tolist()
        self.pub_candidate.publish(scan)

    def publish_rays(self, header, scan3):
        z = -self.sensor_height + 0.03
        clear = Marker(header=header, ns='scan3_clearing', id=0, type=Marker.LINE_LIST)
        clear.scale.x = 0.012
        clear.color = color((0.1, 0.85, 0.2), 0.55)
        noclear = Marker(header=header, ns='scan3_inf_no_clearing', id=1, type=Marker.LINE_LIST)
        noclear.scale.x = 0.012
        noclear.color = color((0.95, 0.1, 0.1), 0.45)
        for i in range(0, self.kSize, self.ray_stride):
            a = -math.pi + (i + 0.5) * self.resolution
            c, s = math.cos(a), math.sin(a)
            r = float(scan3[i])
            if math.isfinite(r):
                rr = min(r, self.raytrace_max_range)
                clear.points += [Point(x=0.0, y=0.0, z=z), Point(x=rr * c, y=rr * s, z=z)]
            else:
                rr = self.inf_ray_length
                noclear.points += [Point(x=0.0, y=0.0, z=z), Point(x=rr * c, y=rr * s, z=z)]
        for m in (clear, noclear):
            if not m.points:
                m.action = Marker.DELETE
        self.pub_rays.publish(MarkerArray(markers=[clear, noclear]))

    def beam_hit(self, elev, az, rot):
        """Return (t, point) where the beam meets the ground, or (inf, None)."""
        d = np.array([math.cos(elev) * math.cos(az), math.cos(elev) * math.sin(az), math.sin(elev)])
        dz = float((rot @ d)[2])
        if dz >= -1e-6:
            return math.inf, None
        t = self.sensor_height / -dz
        return t, d * t

    def publish_rings(self):
        """Draw each ring as a truncated cone (frustum) plus a side-view profile.

        Every ring is a cone around the sensor's vertical axis whose surface
        is tilted by the ring elevation.  The adopted part (sensor_min_range
        .. ring max distance) is drawn as a coloured frustum with near/far
        rims and generator lines; the excluded part (max distance .. ground
        contact) as a grey frustum.  Namespaces are per ring so each ring can
        be toggled in RViz (MarkerArray > Namespaces).
        """
        if self.ring_frame is None:
            return
        header = Header(frame_id=self.ring_frame)
        rot = rpy_matrix(self.roll, self.pitch) if self.use_imu_tilt else np.eye(3)
        markers = []
        n_az = 120
        azs = [-math.pi + 2.0 * math.pi * k / n_az for k in range(n_az + 1)]
        gen_every = max(1, n_az // 12)  # generator line every 30 deg
        prof_az = math.radians(self.profile_azimuth_deg)
        profile = Marker(header=header, ns='断面図', id=0, type=Marker.LINE_LIST)
        profile.scale.x = 0.04
        profile.color = color((1.0, 1.0, 1.0), 1.0)
        profile.pose.orientation.w = 1.0
        prof_grey = Marker(header=header, ns='断面図', id=1, type=Marker.LINE_LIST)
        prof_grey.scale.x = 0.015
        prof_grey.color = color((0.75, 0.75, 0.75), 0.8)
        prof_grey.pose.orientation.w = 1.0
        cp, sp = math.cos(prof_az), math.sin(prof_az)
        zg = -self.sensor_height
        # ground line and sensor mast in the profile plane
        prof_grey.points += [Point(x=0.0, y=0.0, z=zg),
                             Point(x=self.cone_draw_range * cp, y=self.cone_draw_range * sp, z=zg),
                             Point(x=0.0, y=0.0, z=zg), Point(x=0.0, y=0.0, z=0.0)]
        for d in range(1, int(self.cone_draw_range) + 1):  # 1 m ticks on the ground line
            prof_grey.points += [Point(x=d * cp, y=d * sp, z=zg),
                                 Point(x=d * cp, y=d * sp, z=zg + (0.08 if d % 5 else 0.18))]

        for idx, (ring, max_d) in enumerate(self.ring_limit.items()):
            if not 0 <= ring < self.num_rings:
                continue
            e = self.elev[ring]
            rgb = RING_COLORS[idx % len(RING_COLORS)]
            ns = 'R%d %+.0f°' % (ring, math.degrees(e))
            tan_e = math.tan(e)

            def at(horiz, az, tan_e=tan_e):
                return Point(x=horiz * math.cos(az), y=horiz * math.sin(az), z=horiz * tan_e)

            def mk(kind, mid, mtype, rgba, width=1.0):
                m = Marker(header=header, ns=f'{ns} {kind}', id=mid, type=mtype)
                m.pose.orientation.w = 1.0
                m.scale.x = width
                if mtype == Marker.TRIANGLE_LIST:
                    m.scale.y = m.scale.z = 1.0
                m.color = color(rgba[:3], rgba[3])
                return m

            adopted = mk('採用', 0, Marker.TRIANGLE_LIST, (*rgb, 0.35))
            excluded = mk('除外', 0, Marker.TRIANGLE_LIST, (0.6, 0.6, 0.6, 0.12))
            rims = mk('輪郭', 0, Marker.LINE_LIST, (*rgb, 1.0), 0.03)
            gens = mk('輪郭', 1, Marker.LINE_LIST, (*rgb, 0.9), 0.012)
            gens_ex = mk('輪郭', 2, Marker.LINE_LIST, (0.7, 0.7, 0.7, 0.6), 0.008)
            ground = mk('地面交点', 0, Marker.LINE_STRIP, (*rgb, 0.7), 0.025)

            near = self.sensor_min_range * math.cos(e)
            ground_h, far_h, end_h = [], [], []
            for az in azs:
                t, _ = self.beam_hit(e, az, rot)
                g = min(t * math.cos(e), self.sensor_max_range)
                ground_h.append(g)
                end = min(g, self.cone_draw_range)
                end_h.append(end)
                far_h.append(min(max_d, end))
            for k in range(n_az):
                a0, a1 = azs[k], azs[k + 1]
                n0, n1 = at(near, a0), at(near, a1)
                f0, f1 = at(far_h[k], a0), at(far_h[k + 1], a1)
                adopted.points += [n0, f0, f1, n0, f1, n1]
                if end_h[k] > far_h[k] or end_h[k + 1] > far_h[k + 1]:
                    e0, e1 = at(end_h[k], a0), at(end_h[k + 1], a1)
                    excluded.points += [f0, e0, e1, f0, e1, f1]
                rims.points += [n0, n1, f0, f1]
                if ground_h[k] < self.sensor_max_range:
                    ground.points.append(at(ground_h[k], a0))
                if k % gen_every == 0:
                    gens.points += [Point(), f0]
                    if end_h[k] > far_h[k]:
                        gens_ex.points += [f0, at(end_h[k], a0)]
            if ground.points:
                ground.points.append(ground.points[0])
            markers += [adopted, rims, gens]
            for m, kind in ((excluded, '除外'), (gens_ex, '輪郭'), (ground, '地面交点')):
                if len(m.points) >= 2:
                    markers.append(m)
                else:
                    markers.append(Marker(header=header, ns=f'{ns} {kind}', id=m.id,
                                          action=Marker.DELETE))

            # side-view profile in the plane at profile_azimuth_deg
            t_g, _ = self.beam_hit(e, prof_az, rot)
            g_p = min(t_g * math.cos(e), self.sensor_max_range)
            f_p = min(max_d, g_p, self.cone_draw_range)
            profile.points += [Point(), at(f_p, prof_az)]
            profile.colors += [color(rgb, 1.0), color(rgb, 1.0)]
            if g_p < self.sensor_max_range:
                prof_grey.points += [at(f_p, prof_az), at(g_p, prof_az)]

            label = Marker(header=header, ns=f'{ns} ラベル', id=0, type=Marker.TEXT_VIEW_FACING)
            p = at(f_p, prof_az)
            label.pose.position.x, label.pose.position.y = p.x, p.y
            label.pose.position.z = p.z + 0.10 + (0.12 if idx % 2 else 0.0)
            label.pose.orientation.w = 1.0
            label.scale.z = 0.14
            label.color = color(rgb, 1.0)
            gtxt = '%.2fm' % g_p if g_p < self.sensor_max_range else 'なし'
            label.text = 'R%d %+.0f° 上限%.2fm(高さ%.2fm) 地面%s' % (
                ring, math.degrees(e), max_d, self.sensor_height + f_p * tan_e, gtxt)
            markers.append(label)
        markers += [profile, prof_grey]
        self.pub_rings.publish(MarkerArray(markers=markers))

    def log_status(self):
        s = self.last_status
        if not s:
            self.get_logger().warn('no /velodyne_points received yet')
            return
        self.get_logger().info(
            'scan3 +Inf bins %.1f%% (clear_candidate %.1f%%), adopted %d / excluded %d pts, '
            '/scan3 reproduction: %d frames compared, max mismatch %d bins' % (
                100 * s['scan3_inf_ratio'], 100 * s['candidate_inf_ratio'],
                s['adopted_points'], s['excluded_points'], self.compared, self.max_mismatch))


def main(args=None):
    rclpy.init(args=args)
    node = VelodyneCoverageViz()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
