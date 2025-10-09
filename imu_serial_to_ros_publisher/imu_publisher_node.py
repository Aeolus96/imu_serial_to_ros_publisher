
"""IMU serial to ROS2 publisher node.

This node reads newline-delimited JSON from a serial port. Each JSON object is
expected to contain at least the following structure (example):

{
  "header": {"stamp": {"secs": 123, "nsecs": 456}},
  "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
  "orientation_covariance": [..9 numbers..],
  "linear_acceleration": {"x": 0.0, "y": 0.0, "z": 0.0},
  "linear_acceleration_covariance": [..9 numbers..],
  "angular_velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
  "angular_velocity_covariance": [..9 numbers..]
}

This module adds robust handling for JSON errors, serial disconnects and
invalid fields. It logs warnings rather than crashing when it encounters bad
input from the serial device.
"""

import json
import time
import traceback
from typing import Any, List, Optional

import rclpy
import serial
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuSerialPublisher(Node):
    """ROS2 node that publishes sensor_msgs/Imu from newline JSON over serial.

    Notes:
    - The serial port is opened at construction time. If opening fails, the
      node logs an error and will periodically attempt to reopen the port.
    - The node tolerates malformed JSON and missing fields and logs warnings
      instead of crashing.
    """

    def __init__(self):
        super().__init__("imu_serial_publisher")

        # Parameters with sane defaults; parameters can be overridden via ROS2
        # parameter APIs or launch files.
        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("topic", "imu/data")
        self.declare_parameter("frame_id", "imu_link")

        serial_port = self.get_parameter("serial_port").get_parameter_value().string_value
        baud_rate = self.get_parameter("baud_rate").get_parameter_value().integer_value
        topic_name = self.get_parameter("topic").get_parameter_value().string_value
        frame_id = self.get_parameter("frame_id").get_parameter_value().string_value

        self.get_logger().info(f"Starting serial port: {serial_port} at {baud_rate}")
        self.get_logger().info(f"Publishing on topic: {topic_name}")
        self.get_logger().info(f"Using frame_id: {frame_id}")

        # Serial object is initially None; _open_serial will try to open it and
        # set `self.ser` if successful. We keep a timestamp of the last attempt
        # to avoid tight reopen loops.
        self.ser: Optional[serial.Serial] = None
        self._serial_port = serial_port
        self._baud_rate = int(baud_rate)
        self._last_open_attempt = 0.0
        self._open_retry_interval = 5.0  # seconds between open attempts

        # Try to open immediately (non-fatal)
        self._open_serial()

        # IMU publisher
        self.imu_pub = self.create_publisher(Imu, topic_name, 10)
        self._frame_id = frame_id

        # Timer to poll serial input; keep this fairly fast for IMU data.
        self.timer = self.create_timer(0.02, self.timer_callback)  # 50 Hz

    # --- Serial management
    def _open_serial(self) -> bool:
        """Attempt to open the serial port if not already open.

        Returns True on success, False otherwise. On failure the error is logged
        and the node will retry later.
        """
        now = time.time()
        if self.ser is not None and getattr(self.ser, "is_open", False):
            return True
        # avoid retrying too often
        if now - self._last_open_attempt < self._open_retry_interval:
            return False
        self._last_open_attempt = now
        try:
            self.ser = serial.Serial(self._serial_port, self._baud_rate, timeout=0.1)
            self.get_logger().info(f"Opened serial port {self._serial_port} at {self._baud_rate}")
            return True
        except serial.SerialException as e:
            self.get_logger().error(f"Could not open serial port {self._serial_port}: {e}")
            self.ser = None
            return False

    # --- Helpers
    @staticmethod
    def _safe_float(value: Any, default: float = 0.0) -> float:
        """Convert value to float with fallback to default on failure."""
        try:
            # handle None and empty strings
            if value is None:
                return default
            return float(value)
        except (ValueError, TypeError):
            return default

    @staticmethod
    def _fill_covariance(raw: Any) -> List[float]:
        """Return a list of 9 floats from a raw covariance value.

        If raw is not a sequence or has fewer than 9 values the remainder is
        filled with zeros. Invalid entries are coerced to 0.0.
        """
        cov = [0.0] * 9
        if not isinstance(raw, (list, tuple)):
            return cov
        for i in range(min(9, len(raw))):
            cov[i] = ImuSerialPublisher._safe_float(raw[i], 0.0)
        return cov

    # --- Main loop
    def timer_callback(self) -> None:
        """Called on a fixed timer; reads one or more full lines from serial if
        available and processes them.
        """
        # Ensure serial is open (will retry periodically)
        if not self._open_serial():
            return

        assert self.ser is not None
        try:
            # Use readline to simplify newline handling; decode with replacement
            # to avoid exceptions from invalid bytes.
            while self.ser.in_waiting > 0:
                raw = self.ser.readline()
                try:
                    line = raw.decode("utf-8", errors="replace").strip()
                except Exception:
                    # Fallback: if decode somehow fails (shouldn't with errors='replace')
                    line = ""
                if line:
                    self.process_line(line)
        except serial.SerialException as e:
            # Serial device disconnected or other I/O error; close and schedule
            # a reopen.
            self.get_logger().error(f"Serial error while reading: {e}")
            try:
                if self.ser is not None:
                    self.ser.close()
            except Exception:
                pass
            self.ser = None
        except Exception as e:
            # Catch-all: don't let a single bad read crash the node.
            self.get_logger().error(f"Unexpected error in timer_callback: {e}")
            self.get_logger().debug(f"{traceback.format_exc()}")

    def process_line(self, line: str) -> None:
        """Parse a single newline-delimited JSON line and publish an Imu msg.

        This method is defensive: malformed JSON, missing fields, or invalid
        numeric values are handled by logging warnings and skipping publish.
        """
        try:
            data = json.loads(line)
        except json.JSONDecodeError:
            # Serial devices often print non-JSON diagnostic lines at startup
            # (model info, baud, addresses, etc.). Treat those lines as
            # non-fatal and keep them quiet at debug level so logs are not
            # flooded during device boot.
            self.get_logger().debug(f"Non-JSON serial line (ignored): '{line[:200]}'")
            return

        # Basic structural validation
        if not isinstance(data, dict):
            self.get_logger().warning(f"JSON root is not an object: {type(data)!r}")
            return

        header = data.get("header")
        lin_acc = data.get("linear_acceleration")
        ang_vel = data.get("angular_velocity")
        if header is None or lin_acc is None or ang_vel is None:
            self.get_logger().warning(f"Incomplete IMU data (header/acc/gyro missing): {line[:200]}")
            return

        imu_msg = Imu()

        # Header: frame. Use the ROS clock for message timestamp because
        # IMU devices typically do not provide a synchronized clock. If the
        # incoming JSON *also* provides a stamp and you want to use it, that
        # would be a policy decision; here we prefer ROS time for consistency.
        imu_msg.header.frame_id = self._frame_id
        try:
            now = self.get_clock().now().to_msg()
            imu_msg.header.stamp.sec = int(now.sec)
            imu_msg.header.stamp.nanosec = int(now.nanosec)
        except Exception:
            # Fall back to zeros if ROS time isn't available for some reason.
            self.get_logger().warning(f"Unable to read ROS clock for timestamp; using zeros: {line[:200]}")

        # Orientation (defaults: identity quaternion)
        o = data.get("orientation") or {}
        imu_msg.orientation.x = self._safe_float(o.get("x") if isinstance(o, dict) else None, 0.0)
        imu_msg.orientation.y = self._safe_float(o.get("y") if isinstance(o, dict) else None, 0.0)
        imu_msg.orientation.z = self._safe_float(o.get("z") if isinstance(o, dict) else None, 0.0)
        imu_msg.orientation.w = self._safe_float(o.get("w") if isinstance(o, dict) else None, 1.0)

        # Orientation covariance
        ori_cov = self._fill_covariance(data.get("orientation_covariance"))
        for i in range(9):
            imu_msg.orientation_covariance[i] = ori_cov[i]

        # Linear acceleration
        a = lin_acc if isinstance(lin_acc, dict) else {}
        imu_msg.linear_acceleration.x = self._safe_float(a.get("x"), 0.0)
        imu_msg.linear_acceleration.y = self._safe_float(a.get("y"), 0.0)
        imu_msg.linear_acceleration.z = self._safe_float(a.get("z"), 0.0)

        # Linear acceleration covariance
        la_cov = self._fill_covariance(data.get("linear_acceleration_covariance"))
        for i in range(9):
            imu_msg.linear_acceleration_covariance[i] = la_cov[i]

        # Angular velocity
        g = ang_vel if isinstance(ang_vel, dict) else {}
        imu_msg.angular_velocity.x = self._safe_float(g.get("x"), 0.0)
        imu_msg.angular_velocity.y = self._safe_float(g.get("y"), 0.0)
        imu_msg.angular_velocity.z = self._safe_float(g.get("z"), 0.0)

        # Angular velocity covariance
        av_cov = self._fill_covariance(data.get("angular_velocity_covariance"))
        for i in range(9):
            imu_msg.angular_velocity_covariance[i] = av_cov[i]

        try:
            self.imu_pub.publish(imu_msg)
            self.get_logger().debug(f"Published IMU message {self._frame_id}")
        except Exception as e:
            self.get_logger().error(f"Failed to publish IMU message: {e}")
            self.get_logger().debug(traceback.format_exc())


def main(args=None):
    rclpy.init(args=args)
    node = ImuSerialPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
