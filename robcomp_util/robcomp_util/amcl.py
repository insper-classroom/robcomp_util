import math

import cv2
from rclpy.qos import ReliabilityPolicy, QoSProfile
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry, OccupancyGrid
from tf2_msgs.msg import TFMessage
import numpy as np
import rclpy

class AMCL(): # Mude o nome da classe

    def __init__(self):
        # Subscribers
        self.odom_ready = False

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )
        # odom
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile)
        # tf
        self.subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            qos_profile)
        
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            qos_profile)  # Using the custom QoS profile
        self.subscription  # prevent unused variable warning
        
        
        while not self.odom_ready:
            rclpy.spin_once(self)
            print("odom: retrying")
        print("Odom ready")

    def euler_from_quaternion(self, quaternion : list):
            """
            Converts quaternion (w in last place) to euler roll, pitch, yaw
            quaternion = [x, y, z, w]
            Below should be replaced when porting for ROS2 Python tf_conversions is done.
            """
            x = quaternion[0]
            y = quaternion[1]
            z = quaternion[2]
            w = quaternion[3]

            sinr_cosp = 2 * (w * x + y * z)
            cosr_cosp = 1 - 2 * (x * x + y * y)
            roll = np.arctan2(sinr_cosp, cosr_cosp)

            sinp = 2 * (w * y - z * x)
            pitch = np.arcsin(sinp)

            siny_cosp = 2 * (w * z + x * y)
            cosy_cosp = 1 - 2 * (y * y + z * z)
            yaw = np.arctan2(siny_cosp, cosy_cosp)

            return roll, pitch, yaw

    def odom_callback(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        # Get the yaw (rotation around z-axis) from the quaternion
        _, _, self.odom_yaw = self.euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

    def tf_callback(self, msg):
        # frame id needs to be /map
        for tf in msg.transforms:
            if tf.child_frame_id == 'odom':
                try:
                    # Translation
                    x_map = tf.transform.translation.x
                    y_map = tf.transform.translation.y

                    # Rotation (orientation in the map frame)
                    orientation_q = tf.transform.rotation
                    _, _, yaw_map = self.euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

                    # Find the estimated robot pose in the map frame using tf and odom
                    delta_x = self.odom_x * math.cos(yaw_map) - self.odom_y * math.sin(yaw_map)
                    delta_y = self.odom_x * math.sin(yaw_map) + self.odom_y * math.cos(yaw_map)

                    self.x = x_map + delta_x
                    self.y = y_map + delta_y
                    # self.get_logger().info(f'Current pose: ({self.x:.2f}, {self.y:.2f})')

                    self.start = self.world_to_map(self.x, self.y)
                    self.odom_ready = True
                    self.yaw = yaw_map + self.odom_yaw
                    self.yaw = np.arctan2(np.sin(self.yaw), np.cos(self.yaw))
                except Exception as e:
                    self.get_logger().error(f"Error in tf_callback: {e}")
    
    def process_map(self, map_array):
        # print(np.unique(map_array))
        map_array = map_array.astype(np.float32)
        map_array[map_array == -1] = 128  # Gray for unknown
        map_array[map_array < 60] = 255   # White for free
        map_array[(map_array >= 60.) & (map_array != 128.) & (map_array != 255.)] = 0  # Black for obstacles
        # print(np.unique(map_array))
        map_array = map_array.astype(np.uint8)
        # small closing operation to remove small holes
        kernel = np.ones((3, 3), np.uint8)
        map_array = cv2.morphologyEx(map_array, cv2.MORPH_OPEN, kernel)
        map_array = np.flipud(map_array) # TODO - um deles precisa ser removido
        map_array = np.pad(map_array, ((0, 200), (0, 200)), 'constant', constant_values=128)



        return map_array  
    
    def map_callback(self, msg):
        self.resolution = msg.info.resolution  # Map resolution (meters per cell)
        origin = msg.info.origin.position  # Origin of the map in the real world
        self.origin_x = origin.x
        self.origin_y = origin.y
        self.height = msg.info.height
        self.width = msg.info.width
        # self.start = self.world_to_map(0, 0)
        # print(self.start)

        # self.get_logger().info(f'Received map with resolution: {self.resolution} m/cell, width: {msg.info.width}, height: {msg.info.height}')
        # self.get_logger().info(f'Map origin: ({self.origin_x}, {self.origin_y})')

        # Convert map data to a NumPy array
        map_array = np.array(msg.data).reshape((self.height, self.width))

        self.map_array = self.process_map(map_array)
        # print(np.unique(map_array))
        # print(np.unique(self.map_array))
        # cv2.imshow('Map', cv2.resize(self.map_array, (self.width * 10, self.height * 10), interpolation=cv2.INTER_NEAREST))
        # cv2.waitKey(1)

        # self.display_map_pixel_space()

    def map_to_world(self, map_x, map_y):
        # Convert map coordinates (pixels) to world coordinates (meters)
        world_x = map_x * self.resolution + self.origin_x
        world_y = (self.height - map_y) * self.resolution + self.origin_y
        return world_x, world_y

    def world_to_map(self, world_x, world_y):
        # Convert world coordinates (meters) to map coordinates (pixels)
        map_x = int((world_x - self.origin_x) / self.resolution)
        map_y = self.height - int((world_y - self.origin_y) / self.resolution)
        return map_y, map_x