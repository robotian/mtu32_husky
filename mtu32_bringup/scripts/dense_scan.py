#!/usr/bin/env python3
"""The front camera's obstacles by point density, as a LaserScan for Nav2 (sensors/camera_0/scan_dense).

depthimage_to_laserscan keeps the nearest point of every image column, so in a lavender lane the scan is the tips of
the stems sticking out of the rows, ~0.3 m from a robot on the lane centre: Nav2 then sees the lanes narrower than
the a300 (0.70 m wide in 0.67-0.77 m lanes) and MPPI won't drive them. A plant's body is dense where its stems are
sparse, and anything the robot must not brush (a person, a post, another robot) is a solid surface: this node keeps
what is dense.

Every depth pixel (every `stride`-th) is projected into base_link; those in a height band (`z_min`..`z_max`, in
base_link: above the soil, mounds and slopes included, so per platform) go into `cell`-sized top-down cells, each point weighted by the
surface it covers (pixel footprint at its distance: (r/fx)(r/fy)), so a cell's sum is the surface seen in it, the
same for near and far objects. A cell is dense when the surface in its 3x3 neighbourhood reaches `min_area` (cm^2).
Measured in the sim's lavender (a300 on a lane centre, 5 cm cells): stems reaching into the lane 0.6-0.7 cm^2 per
cell (median), the plant bodies 8-10, a solid surface 5 cm wide over a 1 m band ~500.

The scan (in `scan_frame`, rays every `angle_increment` over +-90 deg): the nearest point of a dense cell; +inf
where the camera saw only sparse points in the band (free: clears with the costmap source's inf_is_valid); NaN where
it saw nothing. Self-check: dense_scan.py --selftest.
"""
import math
import sys

import numpy as np

STEMS_CM2 = 0.7  # sim lavender stems per 5 cm cell (median), for the self-check


def dense_cells(x, y, area, cell, x_max, y_max, min_area_cm2):
    """Top-down cells (x ahead 0..x_max, y -y_max..y_max) summing `area` (m^2) per point, and which are dense
    (3x3 neighbourhood >= min_area_cm2). Returns (dense mask per point, the grid, the dense grid)."""
    nx, ny = int(math.ceil(x_max / cell)), int(math.ceil(2 * y_max / cell))
    ix = np.floor(x / cell).astype(np.int64)
    iy = np.floor((y + y_max) / cell).astype(np.int64)
    inside = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
    grid = np.bincount(ix[inside] * ny + iy[inside], weights=area[inside], minlength=nx * ny).reshape(nx, ny)
    pad = np.pad(grid, 1)
    nb = sum(pad[1 + dx:1 + dx + nx, 1 + dy:1 + dy + ny] for dx in (-1, 0, 1) for dy in (-1, 0, 1))
    dense = (nb * 1e4 >= min_area_cm2) & (grid > 0)
    mask = np.zeros(len(x), dtype=bool)
    mask[inside] = dense[ix[inside], iy[inside]]
    return mask, grid, dense


def scan_ranges(px, py, dense_mask, inc, n_rays):
    """Rays over -90..+90 deg from the frame's origin: nearest dense point, +inf where only sparse points, NaN
    where none."""
    ang = np.arctan2(py, px)
    rng = np.hypot(px, py)
    b = np.round((ang + math.pi / 2) / inc).astype(np.int64)
    ok = (b >= 0) & (b < n_rays)
    ranges = np.full(n_rays, np.nan, dtype=np.float32)
    seen = np.zeros(n_rays, dtype=bool)
    seen[b[ok]] = True
    ranges[seen] = np.inf
    d = ok & dense_mask
    best = np.full(n_rays, np.inf)
    np.minimum.at(best, b[d], rng[d])
    hit = np.isfinite(best)
    ranges[hit] = best[hit]
    return ranges


def selftest():
    """A lane: stem tips 0.3 m to each side (sparse), plant bodies 0.6 m out (dense), a solid post 2 m ahead."""
    rng = np.random.default_rng(0)
    xs, ys, areas = [], [], []
    cell = 0.05
    for x0 in np.arange(1.0, 4.0, cell):  # plant bodies: ~10 cm^2 per cell, over 0.55-0.9 m each side
        for side in (-1, 1):
            for y0 in np.arange(0.55, 0.9, cell):
                xs.append(x0 + cell / 2); ys.append(side * (y0 + cell / 2)); areas.append(10e-4)
    for _ in range(200):  # stem tips: isolated points 0.25-0.45 m out, each a fraction of a cm^2
        xs.append(rng.uniform(1.0, 4.0)); ys.append(rng.choice((-1, 1)) * rng.uniform(0.25, 0.45))
        areas.append(STEMS_CM2 * 1e-4)
    for y0 in np.arange(-0.05, 0.05, 0.01):  # a post 10 cm wide, 2 m ahead: ~500 cm^2 per cell
        xs.append(2.0); ys.append(y0); areas.append(250e-4)
    x, y, a = np.array(xs), np.array(ys), np.array(areas)
    mask, _, _ = dense_cells(x, y, a, cell, 5.0, 5.0, 25.0)
    stem = (np.abs(y) > 0.24) & (np.abs(y) < 0.46)
    body = np.abs(y) >= 0.55
    post = np.abs(y) < 0.06
    assert not mask[stem].any(), f"{mask[stem].sum()} stem points kept"
    assert mask[body].mean() > 0.95, f"only {mask[body].mean():.2f} of the plant bodies kept"
    assert mask[post].all(), "the post was dropped"
    inc = math.radians(0.25)
    r = scan_ranges(x, y, mask, inc, int(round(math.pi / inc)) + 1)
    centre = int(round((math.pi / 2) / inc))
    assert abs(r[centre] - 2.0) < 0.06, f"post at {r[centre]:.2f} m, not 2.0"
    side = int(round((math.pi / 2 + math.atan2(0.35, 1.5)) / inc))  # through the stems at 0.35 m, 1.5 m ahead
    assert r[side] > 0.6 or not np.isfinite(r[side]), f"stem kept on the side ray ({r[side]:.2f} m)"
    print("selftest ok: stems dropped, plant bodies and the post kept, post at "
          f"{r[centre]:.2f} m, side ray {r[side]:.2f}")


def main():
    import rclpy
    import tf2_ros
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from sensor_msgs.msg import CameraInfo, Image, LaserScan

    class DenseScan(Node):
        def __init__(self):
            super().__init__('dense_scan')
            p = lambda name, default: self.declare_parameter(name, default).value  # noqa: E731
            self.rate = p('rate', 10.0)
            self.cell = p('cell', 0.05)
            self.min_area = p('min_area', 25.0)  # cm^2 in a cell's 3x3 neighbourhood
            self.z_min, self.z_max = p('z_min', 0.0), p('z_max', 0.85)  # m, in scan_frame (base_link)
            self.range_min, self.range_max = p('range_min', 0.1), p('range_max', 5.0)
            self.stride = int(p('stride', 2))
            self.inc = math.radians(p('angle_increment_deg', 0.25))
            self.frame = p('scan_frame', 'base_link')
            self.n_rays = int(round(math.pi / self.inc)) + 1
            self.info = None
            self.cam = None  # (R, t) camera optical frame -> scan_frame
            self.last = 0.0
            self.tf = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf, self)
            self.pub = self.create_publisher(LaserScan, 'scan', 5)
            self.create_subscription(CameraInfo, 'depth_camera_info', self.on_info, 5)
            self.create_subscription(Image, 'depth', self.on_depth, 5)

        def on_info(self, m):
            self.info = m

        def camera_pose(self, frame):
            try:
                tr = self.tf.lookup_transform(self.frame, frame, rclpy.time.Time())
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f'no transform {self.frame} <- {frame} yet: {e}', throttle_duration_sec=5.0)
                return None
            q, t = tr.transform.rotation, tr.transform.translation
            x, y, z, w = q.x, q.y, q.z, q.w
            R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                          [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                          [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
            self.get_logger().info(f'camera {frame} in {self.frame}: t=({t.x:.3f}, {t.y:.3f}, {t.z:.3f})')
            return R, np.array([t.x, t.y, t.z])

        def on_depth(self, m):
            now = self.get_clock().now().nanoseconds * 1e-9
            if self.info is None or now - self.last < 1.0 / self.rate:
                return
            if self.cam is None:
                self.cam = self.camera_pose(m.header.frame_id)
                if self.cam is None:
                    return
            self.last = now
            if m.encoding == '32FC1':
                depth = np.frombuffer(bytes(m.data), dtype=np.float32).reshape(m.height, m.width)
            elif m.encoding == '16UC1':
                depth = np.frombuffer(bytes(m.data), dtype=np.uint16).reshape(m.height, m.width) * 1e-3
            else:
                self.get_logger().error(f'depth encoding {m.encoding} not handled', throttle_duration_sec=10.0)
                return
            s = self.stride
            k = self.info.k
            fx, fy, cx, cy = k[0], k[4], k[2], k[5]
            v, u = np.mgrid[0:m.height:s, 0:m.width:s]
            z = depth[0:m.height:s, 0:m.width:s].astype(np.float64)
            ok = np.isfinite(z) & (z > self.range_min) & (z < self.range_max)
            u, v, z = u[ok], v[ok], z[ok]
            p = np.stack([(u - cx) / fx * z, (v - cy) / fy * z, z])  # optical frame
            R, t = self.cam
            q = R @ p + t[:, None]
            band = (q[2] > self.z_min) & (q[2] < self.z_max)
            r = np.linalg.norm(p[:, band], axis=0)
            area = (r / fx) * (r / fy) * s * s
            qx, qy = q[0, band], q[1, band]
            mask, _, _ = dense_cells(qx, qy, area, self.cell, self.range_max, self.range_max, self.min_area)
            out = LaserScan()
            out.header.stamp = m.header.stamp
            out.header.frame_id = self.frame
            out.angle_min, out.angle_max, out.angle_increment = -math.pi / 2, math.pi / 2, self.inc
            out.scan_time = 1.0 / self.rate
            out.range_min, out.range_max = self.range_min, self.range_max
            out.ranges = scan_ranges(qx, qy, mask, self.inc, self.n_rays).tolist()
            self.pub.publish(out)

    rclpy.init()
    node = DenseScan()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    if '--selftest' in sys.argv:
        selftest()
    else:
        main()
