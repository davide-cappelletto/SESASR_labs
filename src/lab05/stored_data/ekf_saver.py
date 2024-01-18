import numpy as np
import matplotlib.pyplot as plt
class ErrorMetricsCalculator:
    def __init__(self, odom_data, filter_data, ground_truth_data):
        self.odom_data = odom_data
        self.filter_data = filter_data
        self.ground_truth_data = ground_truth_data

    def odom_metrics(self):
        odom = self.odom_data
        ground = self.ground_truth_data

        odom_xy = odom[:, :2]
        odom_yaw = odom[:, -1]

        ground_xy = ground[:, :2]
        ground_yaw = ground[:, -1]

        N = len(odom_xy)

        e_xy = np.linalg.norm(odom_xy-ground_xy, axis=-1)
        se_xy = np.square(e_xy)
        mse_xy = np.sum(se_xy)/N
        rmse_xy = np.sqrt(mse_xy)

        e_yaw = odom_yaw-ground_yaw
        se_yaw = np.square(e_yaw)
        mse_yaw = np.sum(se_yaw)/N
        rmse_yaw = np.sqrt(mse_yaw)

        RMSE_odom = [rmse_xy, rmse_yaw]
        MAE_odom = [np.max(e_xy), np.max(np.abs(e_yaw))]
        TCE_odom = [np.sum(e_xy), np.sum(np.abs(e_yaw))]

        odom_x = odom_xy[:, 0]
        odom_y = odom_xy[:, -1]

        # Initialize total distance
        TDT_odom = 0.0

    # Iterate through points and calculate distances
        for i in range(0, N-1):

            delta_x = odom_x[i+1] - odom_x[i]
            delta_y = odom_y[i+1] - odom_y[i]
            distance = np.sqrt(delta_x**2 + delta_y**2)
            TDT_odom += distance

        e_p_odom = TCE_odom[0]/TDT_odom

        self.odom_metrics_results = [RMSE_odom, MAE_odom, TCE_odom, e_p_odom]
        return self.odom_metrics_results
        # return self.odom_metrics_results

    def filter_metrics(self):
        filter = self.filter_data
        ground = self.ground_truth_data

        filter_xy = filter[:, :2]
        filter_yaw = filter[:, -1]

        ground_xy = ground[:, :2]
        ground_yaw = ground[:, -1]

        N = len(filter_xy)

        e_xy = np.linalg.norm(filter_xy-ground_xy, axis=-1)
        se_xy = np.square(e_xy)
        mse_xy = np.sum(se_xy)/N
        rmse_xy = np.sqrt(mse_xy)

        e_yaw = filter_yaw-ground_yaw
        se_yaw = np.square(e_yaw)
        mse_yaw = np.sum(se_yaw)/N
        rmse_yaw = np.sqrt(mse_yaw)

        RMSE_filter = [rmse_xy, rmse_yaw]
        MAE_filter = [np.max(e_xy), np.max(np.abs(e_yaw))]
        TCE_filter = [np.sum(e_xy), np.sum(np.abs(e_yaw))]

        filter_x = filter_xy[:, 0]
        filter_y = filter_xy[:, -1]
        # Initialize total distance
        TDT_filter = 0.0

    # Iterate through points and calculate distances
        for i in range(0, N-1):

            delta_x = filter_x[i+1] - filter_x[i]
            delta_y = filter_y[i+1] - filter_y[i]
            distance = np.sqrt(delta_x**2 + delta_y**2)
            TDT_filter += distance

        print("Total Distance Traveled filter:", TDT_filter)

        e_p_filter = TCE_filter[0]/TDT_filter

        self.filter_metrics_results = [
            RMSE_filter, MAE_filter, TCE_filter, e_p_filter]
        return self.filter_metrics_results

        # return RMSE_filter, MAE_filter, TCE_filter, e_p_filter

    def show(self):
        print(f"\n\n% ERROR METRICS FOR /diff_drive_controller/odom AND /odometry/filtererd %\n [xy position errors, yaw errors]\n"
              f"odom_metric:\n\n> RMSE_odom = {self.odom_metrics_results[0]}\n> MAE_odom = {self.odom_metrics_results[1]}\n> Total cumulative error odom (TCE_odom) = {self.odom_metrics_results[2]}\n> %err final pos w.r.t. the total distance = {self.odom_metrics_results[3]}\n\n"
              f"filter_metric:\n\n> RMSE_filter = {self.filter_metrics_results[0]}\n> MAE_filter = {self.filter_metrics_results[1]}\n> Total cumulative error filter (TCE_filter) = {self.filter_metrics_results[2]}\n> %err final pos w.r.t. the total distance = {self.filter_metrics_results[3]}\n")


odom_data = np.load("stored_data/odom.npy")
filter_data = np.load("stored_data/filter.npy")
ground_truth_data = np.load("stored_data/ground_truth.npy")
calculator = ErrorMetricsCalculator(odom_data, filter_data, ground_truth_data)
calculator.odom_metrics()
calculator.odom_metrics_results
calculator.filter_metrics()
calculator.filter_metrics_results
calculator.show()
