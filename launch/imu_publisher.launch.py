# launch/imu_publisher.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    serial_port = LaunchConfiguration("serial_port", default="/dev/ttyACM0")
    baud_rate = LaunchConfiguration("baud_rate", default="115200")
    namespace = LaunchConfiguration("namespace", default="")
    topic = LaunchConfiguration("topic", default="imu/data")
    frame_id = LaunchConfiguration("frame_id", default="imu_link")
    reconnect = LaunchConfiguration("reconnect_interval_seconds", default="2.0")
    no_tel = LaunchConfiguration("no_telemetry_warn_seconds", default="3.0")

    imu_node = Node(
        package="imu_serial_to_ros_publisher",
        executable="imu_publisher_node",
        namespace=namespace,
        output="screen",
        parameters=[
            {"serial_port": serial_port},
            {"baud_rate": baud_rate},
            {"topic": topic},
            {"frame_id": frame_id},
            {"reconnect_interval_seconds": reconnect},
            {"no_telemetry_warn_seconds": no_tel},
        ],
    )
    return LaunchDescription([imu_node])
