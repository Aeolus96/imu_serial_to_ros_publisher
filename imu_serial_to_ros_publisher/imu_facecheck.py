# tools/ros_imu_facecheck.py
#!/usr/bin/env python3
import time, math
from collections import deque
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy


def mean(vs):
    return sum(vs) / len(vs) if vs else 0.0


def norm3(x, y, z):
    return math.sqrt(x * x + y * y + z * z)


class FaceCheck(Node):
    def __init__(self):
        super().__init__("ros_imu_facecheck")
        qos = QoSProfile(
            depth=50,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.sub = self.create_subscription(Imu, "imu/data", self.cb, qos)
        self.gx = deque(maxlen=200)
        self.gy = deque(maxlen=200)
        self.gz = deque(maxlen=200)
        self.ax = deque(maxlen=200)
        self.ay = deque(maxlen=200)
        self.az = deque(maxlen=200)
        self.last_report = 0.0

    def cb(self, msg: Imu):
        self.gx.append(msg.angular_velocity.x)
        self.gy.append(msg.angular_velocity.y)
        self.gz.append(msg.angular_velocity.z)
        self.ax.append(msg.linear_acceleration.x)
        self.ay.append(msg.linear_acceleration.y)
        self.az.append(msg.linear_acceleration.z)
        if len(self.gx) < self.gx.maxlen:
            return
        gmag = sum(norm3(self.gx[i], self.gy[i], self.gz[i]) for i in range(len(self.gx))) / len(self.gx)
        if gmag > 0.02:
            return  # not stationary
        if time.time() - self.last_report < 1.0:
            return
        self.last_report = time.time()
        mx, my, mz = mean(self.ax), mean(self.ay), mean(self.az)
        print(f"Stationary means ax={mx:.3f}, ay={my:.3f}, az={mz:.3f}, |a|={norm3(mx, my, mz):.3f}")
        # Expect one near ±9.81 and the other two near 0 on each face
        near = lambda v: abs(v - 9.80665) < 1.0 or abs(v + 9.80665) < 1.0
        zeros = lambda v: abs(v) < 1.0
        flags = (near(mx), near(my), near(mz), zeros(mx), zeros(my), zeros(mz))
        if not (
            (flags[0] and zeros(my) and zeros(mz))
            or (flags[1] and zeros(mx) and zeros(mz))
            or (flags[2] and zeros(mx) and zeros(my))
        ):
            print("WARN: Axis mapping/scale anomaly; confirm driver scaling and mapping.")


def main():
    rclpy.init()
    node = FaceCheck()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
