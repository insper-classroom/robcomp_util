import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from sensor_msgs.msg import LaserScan

class Laser():
    def __init__(self):
        self.opening = 5
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        rclpy.spin_once(self)

    def scan_callback(self, robcompehlegal: LaserScan):
        self.laser_msg = np.array(robcompehlegal.ranges).round(decimals=2)
        self.laser_msg[self.laser_msg == 0] = np.inf
        self.laser_msg = self.laser_msg.tolist()

        self.left = self.laser_msg[90-self.opening:90+self.opening]
        self.right = self.laser_msg[270-self.opening:270+self.opening]
        self.back = self.laser_msg[180-self.opening:180+self.opening]
        self.front = self.laser_msg[-self.opening:0] + self.laser_msg[0:self.opening]

