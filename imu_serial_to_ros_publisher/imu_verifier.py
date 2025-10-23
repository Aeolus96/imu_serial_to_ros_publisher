# tools/imu_verifier.py
# Strict stationary verifier with dwell, stability, and percentile-based covariance checks.
# Passes if: rate > 90 Hz, no quaternion/gravity/gyro-bias errors, and <=2% covariance ratio outliers during strictly-still windows.

#!/usr/bin/env python3
import math, time, statistics
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Imu


def var_diag(samples):
    n = len(samples)
    if n < 2:
        return (math.nan, math.nan, math.nan)
    sx = sy = sz = 0.0
    sxx = syy = szz = 0.0
    for x, y, z in samples:
        sx += x
        sy += y
        sz += z
        sxx += x * x
        syy += y * y
        szz += z * z
    mx, my, mz = sx / n, sy / n, sz / n
    vx = (sxx - n * mx * mx) / (n - 1)
    vy = (syy - n * my * my) / (n - 1)
    vz = (szz - n * mz * mz) / (n - 1)
    return (max(0.0, vx), max(0.0, vy), max(0.0, vz))


def mean_std(vals):
    n = len(vals)
    if n == 0:
        return (math.nan, math.nan)
    m = sum(vals) / n
    if n < 2:
        return (m, math.nan)
    s2 = sum((v - m) * (v - m) for v in vals) / (n - 1)
    return (m, math.sqrt(max(0.0, s2)))


def norm3(x, y, z):
    return math.sqrt(x * x + y * y + z * z)


class ImuVerifier(Node):
    def __init__(self):
        super().__init__("imu_verifier")
        # Tunables (match firmware: 100 Hz, window ~2 s)
        self.window = 200
        self.gyro_thr = 0.015  # rad/s mean magnitude threshold for stillness
        self.amag_std_thr = 0.06  # m/s^2 std threshold for stillness
        self.amag_expect = 9.80665  # expected |a| at rest
        self.dwell_windows = 3  # require N consecutive still windows
        self.stability_eps = 0.25  # 25% change tolerance between tx cov windows
        self.ratio_ok_low = 0.5  # acceptable ratio band for per-window check
        self.ratio_ok_high = 2.0
        self.max_outlier_frac = 0.02  # allow up to 2% outlier windows

        qos = QoSProfile(
            depth=50,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.sub = self.create_subscription(Imu, "imu/data", self.cb, qos)

        self.frames = 0
        self.start = time.time()
        self.gyros = deque(maxlen=self.window)
        self.accs = deque(maxlen=self.window)
        self.amags = deque(maxlen=self.window)

        self.quat_bad = 0
        self.grav_bad = 0
        self.gyro_bias_bad = 0
        self.ratios = []  # store ratios only for valid still, stable windows
        self.moving_windows = 0
        self.still_streak = 0
        self.prev_tx_cov = None

    def cb(self, msg: Imu):
        self.frames += 1
        gx, gy, gz = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
        ax, ay, az = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
        self.gyros.append((gx, gy, gz))
        self.accs.append((ax, ay, az))
        self.amags.append(norm3(ax, ay, az))

        # Quaternion norm check only if provided
        if msg.orientation_covariance[0] != -1.0:
            qw, qx, qy, qz = msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z
            qn = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
            if not (0.995 <= qn <= 1.005):
                self.quat_bad += 1  # [web:27]

        if len(self.gyros) < self.gyros.maxlen:
            return

        gmag_mean = sum(norm3(*g) for g in self.gyros) / len(self.gyros)
        _, amag_std = mean_std(list(self.amags))
        stationary = (gmag_mean < self.gyro_thr) and (amag_std is not None and amag_std < self.amag_std_thr)

        # Extract transmitted diagonals
        cov_gx = msg.angular_velocity_covariance[0]
        cov_gy = msg.angular_velocity_covariance[4]
        cov_gz = msg.angular_velocity_covariance[8]
        cov_ax = msg.linear_acceleration_covariance[0]
        cov_ay = msg.linear_acceleration_covariance[4]
        cov_az = msg.linear_acceleration_covariance[8]
        tx_cov = (cov_gx, cov_gy, cov_gz, cov_ax, cov_ay, cov_az)

        if stationary:
            self.still_streak += 1

            # Stability gating on tx cov
            if self.prev_tx_cov is not None:
                stable = True
                for prev, curr in zip(self.prev_tx_cov, tx_cov):
                    denom = max(1e-12, prev)
                    if abs(curr - prev) / denom > self.stability_eps:
                        stable = False
                        break
                if not stable:
                    self.prev_tx_cov = tx_cov
                    return

            if self.still_streak < self.dwell_windows:
                self.prev_tx_cov = tx_cov
                return

            # Physics checks at rest
            amag_mean, _ = mean_std(list(self.amags))
            if not (self.amag_expect - 1.0 <= amag_mean <= self.amag_expect + 1.0):
                self.grav_bad += 1  # [web:18]
            gx_mean = sum(g[0] for g in self.gyros) / len(self.gyros)
            gy_mean = sum(g[1] for g in self.gyros) / len(self.gyros)
            gz_mean = sum(g[2] for g in self.gyros) / len(self.gyros)
            if max(abs(gx_mean), abs(gy_mean), abs(gz_mean)) > 0.05:
                self.gyro_bias_bad += 1

            # Variance agreement
            vg = var_diag(list(self.gyros))
            va = var_diag(list(self.accs))
            for comp, tx in [
                (vg[0], cov_gx),
                (vg[1], cov_gy),
                (vg[2], cov_gz),
                (va[0], cov_ax),
                (va[1], cov_ay),
                (va[2], cov_az),
            ]:
                ratio = max(1e-12, comp) / max(1e-12, tx)
                self.ratios.append(ratio)
            self.prev_tx_cov = tx_cov
        else:
            self.moving_windows += 1
            self.still_streak = 0
            self.prev_tx_cov = tx_cov


def main():
    rclpy.init()
    node = ImuVerifier()
    try:
        end = time.time() + 10.0
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    elapsed = max(1e-6, time.time() - node.start)
    rate = node.frames / elapsed

    # Percentile analysis
    ratios = node.ratios
    p50 = statistics.median(ratios) if ratios else float("nan")
    p95 = statistics.quantiles(ratios, n=100)[94] if len(ratios) >= 100 else (max(ratios) if ratios else float("nan"))
    outliers = sum(1 for r in ratios if not (node.ratio_ok_low <= r <= node.ratio_ok_high))
    outlier_frac = (outliers / len(ratios)) if ratios else 0.0

    print(f"Frames: {node.frames}, Rate: {rate:.1f} Hz")
    print(
        f"ratios_n: {len(ratios)}, ratio_p50: {p50:.2f}, ratio_p95: {p95:.2f}, outliers: {outliers} ({outlier_frac * 100:.1f}%)"
    )
    print(
        f"quat_bad: {node.quat_bad}, grav_bad: {node.grav_bad}, gyro_bias_bad: {node.gyro_bias_bad}, moving_windows: {node.moving_windows}"
    )

    ok = (
        node.frames > 500
        and node.quat_bad == 0
        and node.grav_bad == 0
        and node.gyro_bias_bad == 0
        and rate > 90.0
        and ratios
        and outlier_frac <= node.max_outlier_frac
    )
    raise SystemExit(0 if ok else 2)


if __name__ == "__main__":
    main()
