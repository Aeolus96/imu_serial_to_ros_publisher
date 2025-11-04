
# IMU Serial to ROS Publisher

This package publishes IMU sensor data from a serial device as `sensor_msgs/Imu`. It supports common IMUs such as LSM6DSOX, BMI088, and BNO085 via the [imu_i2c_to_serial_publisher](https://github.com/Aeolus96/imu_i2c_to_serial_publisher) firmware.

## Quick build requirements

- Target OS: Ubuntu 24.04 (Noble Numbat)
- Target ROS 2: Jazzy Jellyfish (desktop)
- Python: system Python 3.12

## Installation (development setup)

Source the ROS 2 setup in every new shell (or add to your shell rc):

```bash
source /opt/ros/jazzy/setup.bash
```

1. Initialize rosdep (first time only):

```bash
sudo rosdep init
rosdep update
```

1. Create a workspace and clone the repo:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Aeolus96/imu_serial_to_ros_publisher.git
```

1. Install dependencies with rosdep:

```bash
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

1. Install Python dependencies (if needed):

```bash
python3 -m pip install --user -r src/imu_serial_to_ros_publisher/requirements.txt
```

1. Build and source the workspace:

```bash
colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
```

## Serial device and stable linking

For reliable device naming across reboots, create a stable symlink such as `/dev/my_imu`.

To create a manual udev rule under `/etc/udev/rules.d/99-imu.rules` (use attributes from `udevadm info --attribute-walk --name /dev/ttyACM0`). Then reload udev rules and trigger (may require replugging device to take effect):

```bash
sudo udevadm control --reload
sudo udevadm trigger
```

Verify the symlink:

```bash
ls -l /dev/edubot_imu
```

Add your user to the dialout group so you can access serial devices:

```bash
sudo usermod -aG dialout $USER
# then log out and log in again
```

## Usage

Start the IMU publisher with the provided launch file:

```bash
ros2 launch imu_serial_to_ros_publisher imu_publisher.launch.py
```

Run with custom parameters:

```bash
ros2 launch imu_serial_to_ros_publisher imu_publisher.launch.py \
  serial_port:=/dev/edubot_imu \
  baud_rate:=115200 \
  topic:=imu/data_raw \
  frame_id:=imu_link
```

### Common node parameters

- `serial_port` (string) — default `/dev/ttyACM0`
- `baud_rate` (int) — default `115200`
- `topic` (string) — default `imu/data_raw`
- `frame_id` (string) — default `imu_link`
- `reconnect_interval_seconds` (float) — default `1.0`
- `no_telemetry_warn_seconds` (float) — default `3.0`
- `assert_dtr` (bool) — default `false`

## Tools

After launching the IMU publisher, run orientation check (interactive sign check):

```bash
ros2 run imu_serial_to_ros_publisher imu_orientation_check --ros-args -p topic:=/imu/data_raw
```

Data quality verifier:

```bash
ros2 run imu_serial_to_ros_publisher imu_verifier
```

## Published topics

- `imu/data_raw` (`sensor_msgs/Imu`): accelerations, angular velocities, optional orientation, covariance fields.

## License and contributing

Created by [Devson Butani](https://github.com/Aeolus96), 2025

MIT License. See `LICENSE` file for details.

Contributing bug fixes and new features? Submit a pull request!
