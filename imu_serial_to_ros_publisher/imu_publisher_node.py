import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
import json


class ImuSerialPublisher(Node):
    def __init__(self):
        super().__init__("imu_serial_publisher")

        # Params: serial port and baudrate
        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 115200)
        serial_port = self.get_parameter("serial_port").get_parameter_value().string_value
        baud_rate = self.get_parameter("baud_rate").get_parameter_value().integer_value

        self.get_logger().info(f"Starting serial port: {serial_port} at {baud_rate}")

        # Setup serial
        try:
            self.ser = serial.Serial(serial_port, baud_rate, timeout=0.1)
        except serial.SerialException as e:
            self.get_logger().error(f"Could not open serial port: {e}")
            raise

        # IMU publisher
        self.imu_pub = self.create_publisher(Imu, "imu/data", 10)

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

        imu_msg = Imu()

        # Header timestamp
        secs = data.get("header", {}).get("stamp", {}).get("secs", 0)
        nsecs = data.get("header", {}).get("stamp", {}).get("nsecs", 0)
        imu_msg.header.stamp.sec = int(secs)
        imu_msg.header.stamp.nanosec = int(nsecs)

        # Orientation quaternion
        o = data.get("orientation", {})
        imu_msg.orientation.x = o.get("x", 0.0)
        imu_msg.orientation.y = o.get("y", 0.0)
        imu_msg.orientation.z = o.get("z", 0.0)
        imu_msg.orientation.w = o.get("w", 1.0)

        # Orientation covariance
        ori_cov = data.get("orientation_covariance", [])
        for i in range(min(len(ori_cov), 9)):
            imu_msg.orientation_covariance[i] = float(ori_cov[i])

        # Linear acceleration
        a = data.get("linear_acceleration", {})
        imu_msg.linear_acceleration.x = a.get("x", 0.0)
        imu_msg.linear_acceleration.y = a.get("y", 0.0)
        imu_msg.linear_acceleration.z = a.get("z", 0.0)

        # Linear acceleration covariance
        la_cov = data.get("linear_acceleration_covariance", [])
        for i in range(min(len(la_cov), 9)):
            imu_msg.linear_acceleration_covariance[i] = float(la_cov[i])

        # Angular velocity
        g = data.get("angular_velocity", {})
        imu_msg.angular_velocity.x = g.get("x", 0.0)
        imu_msg.angular_velocity.y = g.get("y", 0.0)
        imu_msg.angular_velocity.z = g.get("z", 0.0)

        # Angular velocity covariance
        av_cov = data.get("angular_velocity_covariance", [])
        for i in range(min(len(av_cov), 9)):
            imu_msg.angular_velocity_covariance[i] = float(av_cov[i])

        self.imu_pub.publish(imu_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuSerialPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
