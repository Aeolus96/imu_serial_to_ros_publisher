# IMU Serial to ROS Publisher

ROS2 package for publishing IMU data from a serial device in ROS sensor_msgs/Imu format. This package is designed to work with various IMU sensors that communicate over serial, including LSM6DSOX, BMI088, and BNO085.

## Prerequisites

- Ubuntu 24.04 (Noble Numbat)
- ROS2 Jazzy Jellyfish (Desktop Install)

  ```bash
  # If you haven't already installed ROS2, follow these steps:
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
  
  sudo apt update
  sudo apt install ros-jazzy-desktop
  ```

## Installation

1. Install development tools and ROS2 dependencies:

```bash
sudo apt update
sudo apt install -y python3-pip python3-rosdep python3-colcon-common-extensions git
```

2. Initialize rosdep if you haven't already:

```bash
sudo rosdep init
rosdep update
```

3. Create a ROS2 workspace:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

4. Clone this repository:

```bash
git clone https://github.com/Aeolus96/imu_serial_to_ros_publisher.git
```

5. Install all dependencies automatically:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

This will automatically install:

- Required ROS2 packages
- Python dependencies (pyserial)

4. Build the workspace:

```bash
colcon build --symlink-install
```

5. Source the workspace:

```bash
source ~/ros2_ws/install/setup.bash
```

## Usage

### Basic IMU Publisher Launch

Launch the IMU publisher node with default settings:

```bash
ros2 launch imu_serial_to_ros_publisher imu_publisher.launch.py
```

### Launch with Custom Parameters

You can specify custom parameters when launching:

```bash
ros2 launch imu_serial_to_ros_publisher imu_publisher.launch.py \
    serial_port:=/dev/ttyACM0 \
    baud_rate:=115200 \
    topic:=imu/data_raw \
    frame_id:=imu_link
```

### Available Parameters

- `serial_port`: Serial port device path (default: `/dev/ttyACM0`)
- `baud_rate`: Serial baud rate (default: 115200)
- `topic`: ROS topic to publish on (default: `imu/data_raw`)
- `frame_id`: TF frame ID for the IMU (default: `imu_link`)
- `reconnect_interval_seconds`: Time between reconnection attempts (default: 2.0)
- `no_telemetry_warn_seconds`: Time before warning about no telemetry (default: 3.0)

## Available Tools

### IMU Orientation Check

Tool to verify correct IMU orientation and axis alignment:

```bash
ros2 run imu_serial_to_ros_publisher imu_orientation_check --ros-args -p topic:=/imu/data_raw
```

### IMU Verifier

Tool to verify IMU data quality and calibration:

```bash
ros2 run imu_serial_to_ros_publisher imu_verifier
```

## Published Topics

- `imu/data_raw` ([sensor_msgs/Imu](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html))
  - Raw IMU data including:
    - Linear acceleration (m/s²)
    - Angular velocity (rad/s)
    - Orientation quaternion (if available from sensor)
    - Covariance matrices

## License

This project is licensed under the MIT License - see the LICENSE file for details.
