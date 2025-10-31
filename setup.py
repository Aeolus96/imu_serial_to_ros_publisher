from glob import glob

from setuptools import setup

package_name = "imu_serial_to_ros_publisher"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="Devson Butani",
    maintainer_email="dbutani@ltu.edu",
    description="ROS2 node publishing IMU msgs from serial JSON input",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "imu_publisher_node = imu_serial_to_ros_publisher.imu_publisher_node:main",
            "imu_verifier = imu_serial_to_ros_publisher.imu_verifier:main",
            "imu_facecheck = imu_serial_to_ros_publisher.imu_facecheck:main",
            "imu_orientation_check = imu_serial_to_ros_publisher.imu_orientation_check:main",
        ],
    },
)
