import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
class ErrorMetricsCalculator:
    def __init__(self, odom_data, filter_data, ground_truth_data):
        self.odom_data = odom_data
        self.filter_data = filter_data
        self.ground_truth_data = ground_truth_data

    def odom_metrics(self):
        odom = self.odom_data
        ground = self.ground_truth_data
        print("odom shape", odom.shape)
        odom_list = odom[:, :2]
        N = len(odom_list)
        odom_arr = np.array(odom_list)
        arr = np.zeros((N,2))
        arr[:,0] = -2
        odom_corrected  = odom_arr + arr
        odom_xy = odom_corrected #.tolist()
        odom_yaw = odom[:, -1]
        ground_xy = ground[:, :2]
        ground_yaw = ground[:, -1]

        e_xy = np.linalg.norm(odom_xy-ground_xy, axis=-1)
        se_xy = np.square(e_xy)
        mse_xy = np.sum(se_xy)/N
        rmse_xy = np.sqrt(mse_xy)
        print("_____________________odom_xy_____________________", odom_xy)
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
        final_position = e_xy[-1]
    # Iterate through points and calculate distances
        for i in range(0, N-1):

            delta_x = odom_x[i+1] - odom_x[i]
            delta_y = odom_y[i+1] - odom_y[i]
            distance = np.sqrt(delta_x**2 + delta_y**2)
            TDT_odom += distance

        e_p_odom = final_position/TDT_odom

        self.e_odom = np.array([e_xy, e_yaw])
        self.odom_metrics_results = [RMSE_odom, MAE_odom, TCE_odom, e_p_odom]
        return self.odom_metrics_results, self.e_odom
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
        final_position = e_xy[-1]
    # Iterate through points and calculate distances
        for i in range(0, N-1):

            delta_x = filter_x[i+1] - filter_x[i]
            delta_y = filter_y[i+1] - filter_y[i]
            distance = np.sqrt(delta_x**2 + delta_y**2)
            TDT_filter += distance

        print("Total Distance Traveled filter:", TDT_filter)

        e_p_filter = final_position/TDT_filter

        self.filter_metrics_results = [  
            RMSE_filter, MAE_filter, TCE_filter, e_p_filter]
        self.e_filter = np.array([e_xy, e_yaw])
        return self.filter_metrics_results, self.e_filter

        # return RMSE_filter, MAE_filter, TCE_filter, e_p_filter

    def show(self):
        print(f"\n\n% ERROR METRICS FOR /diff_drive_controller/odom AND /odometry/filtererd %\n [xy position errors, yaw errors]\n"
              f"odom_metric:\n\n> RMSE_odom = {self.odom_metrics_results[0]}\n> MAE_odom = {self.odom_metrics_results[1]}\n> Total cumulative error odom (TCE_odom) = {self.odom_metrics_results[2]}\n> %err final pos w.r.t. the total distance = {self.odom_metrics_results[3]}\n\n"
              f"filter_metric:\n\n> RMSE_filter = {self.filter_metrics_results[0]}\n> MAE_filter = {self.filter_metrics_results[1]}\n> Total cumulative error filter (TCE_filter) = {self.filter_metrics_results[2]}\n> %err final pos w.r.t. the total distance = {self.filter_metrics_results[3]}\n")

    def plot_data(self, data1, data2, title):
        plt.figure(figsize=(10, 5))
        time = np.arange(len(data1[1]))

        plt.subplot(2,1,1)
        plt.plot(time, data1[0,:], label='odom error', linestyle='-', color = 'b')
        plt.plot(time, data2[0,:], label='filter error', linestyle='-', color = 'r')
        plt.title(f'{title} - XY positions')
        plt.xlabel('Time')
        plt.ylabel('XY position err')
        plt.legend()

        plt.subplot(2,1,2)
        plt.plot(time, data1[1,:], label='odom error yaw', linestyle='-', color = 'b')
        plt.plot(time, data2[1,:], label='filter error yaw', linestyle='-', color = 'r')  
        plt.title(f'{title} - Yaw errors')
        plt.xlabel('Time')
        plt.ylabel('Yaw err')   
        plt.legend() 
        plt.tight_layout()
        plt.show()

script_path = Path(__file__).resolve()

script_directory = script_path.parent

odom_file = script_directory / "odom.npy"
filter_file = script_directory / "filter.npy"
ground_truth_file = script_directory / "ground_truth.npy"

odom_data = np.load(odom_file)
filter_data = np.load(filter_file)
ground_truth_data = np.load(ground_truth_file)

calculator = ErrorMetricsCalculator(odom_data, filter_data, ground_truth_data)

calculator.odom_metrics()
odom_metrics_results = calculator.odom_metrics_results
e_xy_odom = calculator.e_odom

calculator.filter_metrics()
filter_metrics_results = calculator.filter_metrics_results
e_xy_filter = calculator.e_filter

calculator.filter_metrics_results
calculator.e_filter
calculator.show()
#plots
_, e_odom = calculator.odom_metrics()
_, e_filter = calculator.filter_metrics()
#print(e_odom[0,:])
calculator.plot_data(e_odom, e_filter, title="/diff_drive_controller/odom position errors")
#calculator.plot_data(e_odom[0], e_filter[0], title="/diff_drive_controller/odom position errors")
#calculator.plot_data(e_odom[1], e_filter[1], title="/odometry/filtered position errors")