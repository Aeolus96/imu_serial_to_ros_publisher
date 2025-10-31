# imu_serial_to_ros_publisher/imu_publisher_node.py (robust parse + sensor-aware accel watchdog)
#!/usr/bin/env python3
import struct, threading, time, math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Imu
import serial

PKT_IMU_V1 = 0x31
EXPECTED_LEN = 1 + 2 + 1 + 1 + 4 + 16 + 12 + 12 + 12 + 12 + 12 + 2  # 87 bytes

# Sensor IDs must match firmware
SID_NONE = 0
SID_LSM6DSOX = 1
SID_BMI088 = 2
SID_BNO085 = 3


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
        # Default False: many boards reset on DTR, causing reconnect loops
        self.declare_parameter("assert_dtr", False)
        self.declare_parameter("accel_mag_warn_tolerance", 4.0)  # m/s^2 deviation allowed when stationary
        self.declare_parameter("stationary_gyro_threshold", 0.02)  # rad/s magnitude
        # Optional override expected |a| at rest per sensor; None means auto
        self.declare_parameter("expected_rest_amag_bno085", 0.0)
        self.declare_parameter("expected_rest_amag_default", 9.80665)

        self.port = self.get_parameter("serial_port").get_parameter_value().string_value
        self.baud = int(self.get_parameter("baud_rate").get_parameter_value().integer_value)
        self.topic = self.get_parameter("topic").get_parameter_value().string_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.reconnect_s = float(self.get_parameter("reconnect_interval_seconds").get_parameter_value().double_value)
        self.no_tel_s = float(self.get_parameter("no_telemetry_warn_seconds").get_parameter_value().double_value)
        self.assert_dtr = bool(self.get_parameter("assert_dtr").get_parameter_value().bool_value)
        self.accel_tol = float(self.get_parameter("accel_mag_warn_tolerance").get_parameter_value().double_value)
        self.gyro_thr = float(self.get_parameter("stationary_gyro_threshold").get_parameter_value().double_value)
        self.rest_bno = float(self.get_parameter("expected_rest_amag_bno085").get_parameter_value().double_value)
        self.rest_def = float(self.get_parameter("expected_rest_amag_default").get_parameter_value().double_value)

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
        )

    def _parse_payload(self, payload: bytes):
        # Validate size, type, CRC
        if len(payload) != EXPECTED_LEN or payload[0] != PKT_IMU_V1:
            return None
        if struct.unpack_from("<H", payload, EXPECTED_LEN - 2)[0] != crc16_ccitt(payload[:-2]):
            return None
        off = 0
        (pkt_type, seq, sensor_id, flags, t_ms) = struct.unpack_from("<B H B B I", payload, off)
        off += 1 + 2 + 1 + 1 + 4
        (qw, qx, qy, qz) = struct.unpack_from("<4f", payload, off)
        off += 16
        (gx, gy, gz) = struct.unpack_from("<3f", payload, off)
        off += 12
        (ax, ay, az) = struct.unpack_from("<3f", payload, off)
        off += 12
        (cov_ori_x, cov_ori_y, cov_ori_z) = struct.unpack_from("<3f", payload, off)
        off += 12
        (cov_gx, cov_gy, cov_gz) = struct.unpack_from("<3f", payload, off)
        off += 12
        (cov_ax, cov_ay, cov_az) = struct.unpack_from("<3f", payload, off)
        off += 12
        return {
            "seq": seq,
            "sid": sensor_id,
            "flags": flags,
            "t_ms": t_ms,
            "quat": (qw, qx, qy, qz),
            "gyro": (gx, gy, gz),
            "acc": (ax, ay, az),
            "cov_o": (cov_ori_x, cov_ori_y, cov_ori_z),
            "cov_g": (cov_gx, cov_gy, cov_gz),
            "cov_a": (cov_ax, cov_ay, cov_az),
        }

    def worker(self):
        # Rolling buffers for stillness detection
        gyro_mag_buf = []
        acc_mag_buf = []
        maxlen = 200

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

                parsed = self._parse_payload(payload)
                if not parsed:
                    continue

                # Update telemetry time and detect drops
                self.last_tel_t = time.monotonic()
                if self.prev_seq is not None and ((self.prev_seq + 1) & 0xFFFF) != parsed["seq"]:
                    self.get_logger().warn(f"Dropped frames: prev={self.prev_seq} curr={parsed['seq']}")
                self.prev_seq = parsed["seq"]

                # Build Imu msg
                msg = Imu()
                msg.header.frame_id = self.frame_id
                msg.header.stamp = self.get_clock().now().to_msg()

                gx, gy, gz = parsed["gyro"]
                ax, ay, az = parsed["acc"]
                msg.angular_velocity.x = gx
                msg.angular_velocity.y = gy
                msg.angular_velocity.z = gz
                msg.linear_acceleration.x = ax
                msg.linear_acceleration.y = ay
                msg.linear_acceleration.z = az

                # Orientation if valid (flags bit0 = 1)
                orientation_valid = (parsed["flags"] & 0x01) != 0
                if orientation_valid:
                    qw, qx, qy, qz = parsed["quat"]
                    # Normalize defensively
                    qn = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
                    if qn > 0.0:
                        qw, qx, qy, qz = (qw / qn, qx / qn, qy / qn, qz / qn)
                    msg.orientation.w = qw
                    msg.orientation.x = qx
                    msg.orientation.y = qy
                    msg.orientation.z = qz
                    # Fill diagonal orientation covariance
                    msg.orientation_covariance[0] = max(0.0, parsed["cov_o"][0])
                    msg.orientation_covariance[4] = max(0.0, parsed["cov_o"][1])
                    msg.orientation_covariance[8] = max(0.0, parsed["cov_o"][2])
                    msg.orientation_covariance[1] = msg.orientation_covariance[3] = 0.0
                    msg.orientation_covariance[2] = msg.orientation_covariance[6] = 0.0
                    msg.orientation_covariance[5] = msg.orientation_covariance[7] = 0.0
                else:
                    # REP 145: -1 in [0] to indicate unknown orientation
                    msg.orientation_covariance[0] = -1.0
                    msg.orientation_covariance[1] = msg.orientation_covariance[2] = 0.0
                    msg.orientation_covariance[3] = msg.orientation_covariance[4] = 0.0
                    msg.orientation_covariance[5] = 0.0
                    msg.orientation_covariance[6] = msg.orientation_covariance[7] = msg.orientation_covariance[8] = 0.0

                # Diagonal covariances (clamp negatives)
                cgx, cgy, cgz = parsed["cov_g"]
                cax, cay, caz = parsed["cov_a"]
                msg.angular_velocity_covariance[0] = max(0.0, cgx)
                msg.angular_velocity_covariance[4] = max(0.0, cgy)
                msg.angular_velocity_covariance[8] = max(0.0, cgz)
                msg.angular_velocity_covariance[1] = msg.angular_velocity_covariance[2] = 0.0
                msg.angular_velocity_covariance[3] = msg.angular_velocity_covariance[5] = 0.0
                msg.angular_velocity_covariance[6] = msg.angular_velocity_covariance[7] = 0.0

                msg.linear_acceleration_covariance[0] = max(0.0, cax)
                msg.linear_acceleration_covariance[4] = max(0.0, cay)
                msg.linear_acceleration_covariance[8] = max(0.0, caz)
                msg.linear_acceleration_covariance[1] = msg.linear_acceleration_covariance[2] = 0.0
                msg.linear_acceleration_covariance[3] = msg.linear_acceleration_covariance[5] = 0.0
                msg.linear_acceleration_covariance[6] = msg.linear_acceleration_covariance[7] = 0.0

                # Stationary watchdog: choose expected |a| by sensor_id
                # BNO085 publishes SH2_LINEAR_ACCELERATION (gravity removed) => expect ~0 at rest
                sid = parsed["sid"]
                rest_expect = self.rest_bno if sid == SID_BNO085 else self.rest_def

                # Update rolling windows
                gmag = math.sqrt(gx * gx + gy * gy + gz * gz)
                amag = math.sqrt(ax * ax + ay * ay + az * az)
                gyro_mag_buf.append(gmag)
                acc_mag_buf.append(amag)
                if len(gyro_mag_buf) > maxlen:
                    gyro_mag_buf.pop(0)
                    acc_mag_buf.pop(0)

                if len(gyro_mag_buf) == maxlen:
                    still = (sum(gyro_mag_buf) / len(gyro_mag_buf)) < self.gyro_thr
                    if still:
                        now = time.monotonic()
                        amag_mean = sum(acc_mag_buf) / len(acc_mag_buf)
                        if abs(amag_mean - rest_expect) > self.accel_tol and (now - self.last_mag_warn) > 1.0:
                            self.get_logger().warn(
                                f"Accel magnitude {amag_mean:.2f} m/s^2 at rest; expected ~{rest_expect:.2f} for sid={sid}"
                            )
                            self.last_mag_warn = now

                self.pub.publish(msg)

            except Exception as e:
                # Close and retry on any I/O error
                self.get_logger().warn(f"Serial read error: {e}")
                with self.ser_lock:
                    self._close_locked()

    def _ensure_open(self) -> bool:
        with self.ser_lock:
            if self.ser:
                return True
            try:
                # Disable HW/SW flow control; leave DTR toggling configurable
                self.ser = serial.Serial(
                    self.port, self.baud, timeout=0.2, write_timeout=0.2, rtscts=False, dsrdtr=False, xonxoff=False
                )
                try:
                    if self.assert_dtr:
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
