import rclpy
from rclpy.node import Node
import tf_transformations
from nav_msgs.msg import Odometry
import message_filters

import numpy as np
import math
from pathlib import Path

class RecorderAltNode(Node):
    def __init__(self):
        super().__init__("my_node")
        self.get_logger().info("Initializing my_node")
        
        # Create the subscribers and synchronize the messages
        self.odom_sub = message_filters.Subscriber(self, Odometry, "/diff_drive_controller/odom")
        #self.filter_sub = message_filters.Subscriber(self, Odometry, "/ekf")
        self.ground_truth_sub = message_filters.Subscriber(self, Odometry, "/ground_truth")
        self.ts = message_filters.ApproximateTimeSynchronizer(
            #[self.odom_sub, self.filter_sub, self.ground_truth_sub], 10, 0.02
            [self.odom_sub, self.ground_truth_sub], 10, 0.02
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

    #def store_data(self, odom, filter, ground_truth):
    def store_data(self, odom, ground_truth):
        self.get_logger().info(
            f"Storing data... Current lenght {len(self.odom)}", throttle_duration_sec=5.0
        )
        self.odom.append([odom.pose.pose.position.x, odom.pose.pose.position.y, self.yaw_calc(odom.pose.pose.orientation)])
        #self.filter.append([filter.pose.pose.position.x, filter.pose.pose.position.y, filter.pose.pose.orientation.z])
        self.ground_truth.append([ground_truth.pose.pose.position.x, ground_truth.pose.pose.position.y, self.yaw_calc(ground_truth.pose.pose.orientation)])
        # Convert incoming data to NumPy arrays
        #odom_array = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, self.yaw_calc(odom.pose.pose.orientation)])
        #filter_array = np.array([filter.pose.pose.position.x, filter.pose.pose.position.y, filter.pose.pose.orientation.z])
        #ground_truth_array = np.array([ground_truth.pose.pose.position.x, ground_truth.pose.pose.position.y, self.yaw_calc(ground_truth.pose.pose.orientation)])

        # Append data to existing arrays
        #self.odom = np.vstack([self.odom, odom_array]) if self.odom else odom_array
        #self.filter = np.vstack([self.filter, filter_array]) if self.filter else filter_array
        #self.ground_truth = np.vstack([self.ground_truth, ground_truth_array]) if self.ground_truth else ground_truth_array
        
        
    def save_data(self):
        self.get_logger().info('Saving the data...')
        odom_path = Path('src/localization_project/stored_data/odom.npy').resolve()
        #filter_path = Path('src/localization_project/stored_data/filter.npy').resolve()
        gt_path = Path('src/localization_project/stored_data/ground_truth.npy').resolve()

        np.save(odom_path, np.array(self.odom))
        #np.save(filter_path, np.array(self.filter))
        np.save(gt_path, np.array(self.ground_truth))

    
def main(args=None):
    rclpy.init(args=args)
    node = RecorderAltNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_data()
        print(node.odom)
        #print(node.filter)
        print(node.ground_truth)
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
