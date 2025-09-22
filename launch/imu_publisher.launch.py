from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    serial_port = LaunchConfiguration("serial_port", default="/dev/ttyACM0")
    baud_rate = LaunchConfiguration("baud_rate", default="115200")
    namespace = LaunchConfiguration("namespace", default="")
    topic = LaunchConfiguration("topic", default="imu/data")
    frame_id = LaunchConfiguration("frame_id", default="imu_link")

    imu_node = Node(
        package="imu_serial_to_ros_publisher",
        executable="imu_publisher_node",
        namespace=namespace,
        output="screen",
        parameters=[
            {"serial_port": serial_port},
            {"baud_rate": int(baud_rate)},
            {"topic": topic},
            {"frame_id": frame_id},
        ],
    )

    return LaunchDescription([imu_node])
