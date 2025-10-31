# imu_serial_to_ros_publisher/imu_publisher_node.py (with accel magnitude watchdog)
#!/usr/bin/env python3
import struct, threading, time, math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Imu
import serial

PKT_IMU_V1 = 0x31
EXPECTED_LEN = 1 + 2 + 1 + 1 + 4 + 16 + 12 + 12 + 12 + 12 + 12 + 2


def crc16_ccitt(data: bytes, crc: int = 0xFFFF) -> int:
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc


def cobs_decode(data: bytes) -> bytes:
    out = bytearray()
    i = 0
    n = len(data)
    while i < n:
        code = data[i]
        i += 1
        if code == 0:
            raise ValueError("Zero code")
        end = i + code - 1
        if end > n:
            raise ValueError("Overrun")
        out.extend(data[i:end])
        i = end
        if code < 0xFF and i < n:
            out.append(0)
    return bytes(out)


class ImuCobsPublisher(Node):
    def __init__(self):
        super().__init__("imu_serial_publisher")
        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("topic", "imu/data_raw")
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("reconnect_interval_seconds", 1.0)
        self.declare_parameter("no_telemetry_warn_seconds", 3.0)
        self.declare_parameter("assert_dtr", True)
        self.declare_parameter("accel_mag_warn_tolerance", 4.0)  # m/s^2 deviation allowed when stationary
        self.declare_parameter("stationary_gyro_threshold", 0.02)  # rad/s magnitude

        self.port = self.get_parameter("serial_port").get_parameter_value().string_value
        self.baud = int(self.get_parameter("baud_rate").get_parameter_value().integer_value)
        self.topic = self.get_parameter("topic").get_parameter_value().string_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.reconnect_s = float(self.get_parameter("reconnect_interval_seconds").get_parameter_value().double_value)
        self.no_tel_s = float(self.get_parameter("no_telemetry_warn_seconds").get_parameter_value().double_value)
        self.assert_dtr = bool(self.get_parameter("assert_dtr").get_parameter_value().bool_value)
        self.accel_tol = float(self.get_parameter("accel_mag_warn_tolerance").get_parameter_value().double_value)
        self.gyro_thr = float(self.get_parameter("stationary_gyro_threshold").get_parameter_value().double_value)

        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.pub = self.create_publisher(Imu, self.topic, qos)

        self.ser_lock = threading.Lock()
        self.ser: Optional[serial.Serial] = None
        self.stop_evt = threading.Event()
        self.last_tel_t = time.monotonic()
        self.last_tel_warn = 0.0
        self.prev_seq = None
        self.last_mag_warn = 0.0

        threading.Thread(target=self.worker, daemon=True).start()
        self.get_logger().info(
            f"IMU COBS bridge on {self.port}@{self.baud}, publishing {self.topic} frame_id={self.frame_id}"
        )  # [attached_file:3]

    def worker(self):
        while not self.stop_evt.is_set():
            if not self._ensure_open():
                time.sleep(self.reconnect_s)
                continue
            try:
                with self.ser_lock:
                    if not self.ser:
                        continue
                    raw = self.ser.read_until(b"\x00")
                if not raw:
                    now = time.monotonic()
                    if (now - self.last_tel_t) > self.no_tel_s and (now - self.last_tel_warn) > self.no_tel_s:
                        self.get_logger().warn("Serial connected but no IMU telemetry lately")
                        self.last_tel_warn = now
                    time.sleep(0.02)
                    continue
                if raw.endswith(b"\x00"):
                    raw = raw[:-1]
                try:
                    payload = cobs_decode(raw)
                except Exception:
                    continue
                if len(payload) != EXPECTED_LEN or payload[0] != PKT_IMU_V1:
                    continue
                crc_rx = struct.unpack_from("<H", payload, len(payload) - 2)[0]
                if crc_rx != crc16_ccitt(payload[:-2]):
                    continue
                self.last_tel_t = time.monotonic()
                self.handle_imu_v1(payload)
            except Exception as e:
                self.get_logger().warn(f"Serial read error: {e}")
                with self.ser_lock:
                    self._close_locked()
                time.sleep(self.reconnect_s)

    def handle_imu_v1(self, b: bytes):
        off = 0
        (_t, seq, sensor_id, flags, t_ms) = struct.unpack_from("<B H B B I", b, off)
        off += 1 + 2 + 1 + 1 + 4
        (qw, qx, qy, qz) = struct.unpack_from("<4f", b, off)
        off += 16
        (gx, gy, gz) = struct.unpack_from("<3f", b, off)
        off += 12
        (ax, ay, az) = struct.unpack_from("<3f", b, off)
        off += 12
        (cov_ox, cov_oy, cov_oz) = struct.unpack_from("<3f", b, off)
        off += 12
        (cov_gx, cov_gy, cov_gz) = struct.unpack_from("<3f", b, off)
        off += 12
        (cov_ax, cov_ay, cov_az) = struct.unpack_from("<3f", b, off)
        off += 12

        if self.prev_seq is not None and ((self.prev_seq + 1) & 0xFFFF) != seq:
            self.get_logger().warn(f"IMU seq gap: prev={self.prev_seq} curr={seq}")
        self.prev_seq = seq

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        orientation_valid = (flags & 0x01) != 0
        if orientation_valid:
            msg.orientation.w = float(qw)
            msg.orientation.x = float(qx)
            msg.orientation.y = float(qy)
            msg.orientation.z = float(qz)
            msg.orientation_covariance = [float(cov_ox), 0.0, 0.0, 0.0, float(cov_oy), 0.0, 0.0, 0.0, float(cov_oz)]
        else:
            msg.orientation_covariance = [-1.0] + [0.0] * 8  # Imu contract sentinel [web:27]

        msg.angular_velocity.x = float(gx)
        msg.angular_velocity.y = float(gy)
        msg.angular_velocity.z = float(gz)
        msg.angular_velocity_covariance = [float(cov_gx), 0.0, 0.0, 0.0, float(cov_gy), 0.0, 0.0, 0.0, float(cov_gz)]

        msg.linear_acceleration.x = float(ax)
        msg.linear_acceleration.y = float(ay)
        msg.linear_acceleration.z = float(az)
        msg.linear_acceleration_covariance = [float(cov_ax), 0.0, 0.0, 0.0, float(cov_ay), 0.0, 0.0, 0.0, float(cov_az)]

        # Watchdog: if nearly stationary but |a| far from g, log a warning for bring-up
        now = time.monotonic()
        gmag = math.sqrt(gx * gx + gy * gy + gz * gz)
        amag = math.sqrt(ax * ax + ay * ay + az * az)
        if gmag < self.gyro_thr:
            if abs(amag - 9.80665) > self.accel_tol and (now - self.last_mag_warn) > 1.0:
                self.get_logger().warn(
                    f"Accel magnitude {amag:.2f} m/s^2 at rest; expected ~9.81. Check axis mapping/scale."
                )
                self.last_mag_warn = now  # [web:18][web:27]

        self.pub.publish(msg)

    def _ensure_open(self) -> bool:
        with self.ser_lock:
            if self.ser:
                return True
            try:
                self.ser = serial.Serial(self.port, self.baud, timeout=0.2, write_timeout=0.2)
                try:
                    if bool(self.get_parameter("assert_dtr").get_parameter_value().bool_value):
                        self.ser.setDTR(True)
                except Exception:
                    pass
                time.sleep(0.2)
                try:
                    self.ser.reset_input_buffer()
                    self.ser.reset_output_buffer()
                except Exception:
                    pass
                self.last_tel_t = time.monotonic()
                self.last_tel_warn = 0.0
                self.get_logger().info(f"Connected to {self.port}@{self.baud}")
                return True
            except Exception as e:
                self.ser = None
                self.get_logger().warn(f"Open failed for {self.port}: {e}")
                return False

    def _close_locked(self):
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass
        finally:
            self.ser = None

    def destroy_node(self):
        self.stop_evt.set()
        time.sleep(0.05)
        with self.ser_lock:
            self._close_locked()
        super().destroy_node()


def main():
    rclpy.init()
    node = ImuCobsPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
