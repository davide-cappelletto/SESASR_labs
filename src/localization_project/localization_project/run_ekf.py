from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import tf_transformations
from rclpy.qos import qos_profile_sensor_data
import math
import time
from ekf import RobotEKF
from motion_models import velocity_motion_model, get_odometry_input, odometry_motion_model
from measurement_model import range_and_bearing, z_landmark, residual

class robot_localization_kalman_filter(Node):
    def __init__(self):
        super().__init__('robot_localization_kalman_filter')
        self.ground_truth_sub = self.create_subscription(Odometry, '/ground_truth', qos_profile_sensor_data)
        self.odom_sub = self.create_subscription(Odometry, '/diff_drive_controller/odom', qos_profile_sensor_data)
        self.ekf_pub = self.create_publisher(Odometry, '/ekf', 10)

        self.ekf_period_s = 0.0
        self.initial_pose = [...]
        self.initial_covariance = [...]
        self.landmarks = [...]
        self.std_rot1 = [...]
        self.std_trasl = [...]
        self.std_rot2 = [...]
        self.std_lin_vel = [...]
        self.std_ang_vel = [...]
        self.std_rng = [...]
        self.std_brg = [...]
        self.max_range = (...)
        self.fov_deg = [...]
