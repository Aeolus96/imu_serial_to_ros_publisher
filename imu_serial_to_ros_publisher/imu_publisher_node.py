import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
import json


class ImuSerialPublisher(Node):
    def __init__(self):
        super().__init__("imu_serial_publisher")

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

        # Setup serial
        try:
            self.ser = serial.Serial(serial_port, baud_rate, timeout=0.1)
        except serial.SerialException as e:
            self.get_logger().error(f"Could not open serial port: {e}")
            raise

        # IMU publisher with parameterized topic
        self.imu_pub = self.create_publisher(Imu, topic_name, 10)
        self._frame_id = frame_id

        # Timer to check serial
        self.timer = self.create_timer(0.02, self.timer_callback)  # 50 Hz
        self.buffer = ""

    def timer_callback(self):
        while self.ser.in_waiting > 0:
            try:
                char = self.ser.read().decode("utf-8")
            except UnicodeDecodeError:
                continue
            if char == "\n":
                self.process_line(self.buffer)
                self.buffer = ""
            else:
                self.buffer += char

    def process_line(self, line):
        try:
            data = json.loads(line)
        except json.JSONDecodeError:
            self.get_logger().warn(f"Invalid JSON: {line}")
            return

        # Required fields check
        if "header" not in data or "linear_acceleration" not in data or "angular_velocity" not in data:
            self.get_logger().warn(f"Incomplete IMU data: {line}")
            return

        imu_msg = Imu()

        # Header
        imu_msg.header.frame_id = self._frame_id
        secs = data["header"].get("stamp", {}).get("secs", 0)
        nsecs = data["header"].get("stamp", {}).get("nsecs", 0)
        imu_msg.header.stamp.sec = int(secs)
        imu_msg.header.stamp.nanosec = int(nsecs)

        # Orientation quaternion
        o = data.get("orientation", {})
        imu_msg.orientation.x = float(o.get("x", 0.0) or 0.0)
        imu_msg.orientation.y = float(o.get("y", 0.0) or 0.0)
        imu_msg.orientation.z = float(o.get("z", 0.0) or 0.0)
        imu_msg.orientation.w = float(o.get("w", 1.0) or 1.0)

        # Orientation covariance
        ori_cov = data.get("orientation_covariance", [0.0] * 9)
        for i in range(9):
            imu_msg.orientation_covariance[i] = float(ori_cov[i] if i < len(ori_cov) else 0.0)

        # Linear acceleration
        a = data.get("linear_acceleration", {})
        imu_msg.linear_acceleration.x = float(a.get("x", 0.0) or 0.0)
        imu_msg.linear_acceleration.y = float(a.get("y", 0.0) or 0.0)
        imu_msg.linear_acceleration.z = float(a.get("z", 0.0) or 0.0)

        # Linear acceleration covariance
        la_cov = data.get("linear_acceleration_covariance", [0.0] * 9)
        for i in range(9):
            imu_msg.linear_acceleration_covariance[i] = float(la_cov[i] if i < len(la_cov) else 0.0)

        # Angular velocity
        g = data.get("angular_velocity", {})
        imu_msg.angular_velocity.x = float(g.get("x", 0.0) or 0.0)
        imu_msg.angular_velocity.y = float(g.get("y", 0.0) or 0.0)
        imu_msg.angular_velocity.z = float(g.get("z", 0.0) or 0.0)

        # Angular velocity covariance
        av_cov = data.get("angular_velocity_covariance", [0.0] * 9)
        for i in range(9):
            imu_msg.angular_velocity_covariance[i] = float(av_cov[i] if i < len(av_cov) else 0.0)

        self.imu_pub.publish(imu_msg)
        self.get_logger().debug("Published IMU message")


def main(args=None):
    rclpy.init(args=args)
    node = ImuSerialPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
