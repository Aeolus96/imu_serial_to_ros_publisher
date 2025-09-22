from setuptools import setup

package_name = "imu_serial_to_ros_publisher"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="User",
    maintainer_email="user@example.com",
    description="ROS2 node publishing IMU msgs from serial JSON input",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["imu_publisher_node = imu_serial_to_ros_publisher.imu_publisher_node:main"],
    },
)
