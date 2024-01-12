import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry

import message_filters

import numpy as np
from pathlib import Path
import math



class RecorderNode(Node):
    def __init__(self):
        super().__init__("my_node")
        self.get_logger().info("Initializing my_node")

        # Create the subscribers and synchronize the messages
        self.odom_sub = message_filters.Subscriber(self, Odometry, "/diff_drive_controller/odom")
        self.filter_sub = message_filters.Subscriber(self, Odometry, "/odometry/filtered")
        self.ground_truth_sub = message_filters.Subscriber(self, Odometry, "/ground_truth")
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.odom_sub, self.filter_sub, self.ground_truth_sub], 10, 0.01
        )
        self.ts.registerCallback(self.store_data)

        self.odom = []
        self.filter = []
        self.ground_truth = []

    def yaw_calc(self, quaternion):
        # Convert quaternion to yaw (rotation around the Z-axis)
        t3 = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        t4 = 1.0 - 2.0 * (quaternion.y**2 + quaternion.z**2)
        yaw = math.atan2(t3, t4)
        return yaw

    def store_data(self, odom, filter, ground_truth):
        self.get_logger().info(
            f"Storing data... Current lenght {len(self.odom)}", throttle_duration_sec=5.0
        )

        self.odom.append([odom.pose.pose.position.x, odom.pose.pose.position.y, self.yaw_calc(odom.pose.pose.orientation)])
        self.filter.append([filter.pose.pose.position.x, filter.pose.pose.position.y, self.yaw_calc(filter.pose.pose.orientation)])
        self.ground_truth.append([ground_truth.pose.pose.position.x, ground_truth.pose.pose.position.y, self.yaw_calc(ground_truth.pose.pose.orientation)])

    def save_data(self):
        # adds '.npy' files to a 'stored_data' folder that was previously created. Remember to import 'Path' from 'pathlib'.
        odom_path = Path.cwd() / 'src/lab05_custom/stored_data/odom.npy'
        filter_path = Path.cwd() / 'src/lab05_custom/stored_data/filter.npy'
        gt_path = Path.cwd() / 'src/lab05_custom/stored_data/ground_truth.npy'
        np.save(odom_path, np.array(self.odom))
        np.save(filter_path, np.array(self.filter))
        np.save(gt_path, np.array(self.ground_truth))
    

def main(args=None):
    rclpy.init(args=args)
    node = RecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_data()

        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
