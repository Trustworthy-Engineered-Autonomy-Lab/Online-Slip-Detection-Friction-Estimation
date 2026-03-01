import rclpy
import os
from rclpy.node import Node
from sensor_msgs.msg import Imu
from vesc_msgs.msg import VescImuStamped
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String, Float64
import numpy as np
import pandas as pd
from queue import Queue
from sklearn.preprocessing import StandardScaler
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

class DriftDetector(Node):
    def __init__(self):
        super().__init__('drifting_detector')
        
        self.wheelbase = 0.32  # 32 cm in meters
        self.tirewidth = 0.04445
        self.mass = 3.333
        self.gravity = 9.81
        self.force_of_gravity = self.mass * self.gravity

        self.mus = []

        self.ackermann_callback = self.create_subscription(
            AckermannDriveStamped,
            '/ackermann_cmd',
            self.ackermann_callback,
            10
        )

        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.imu_subscriber = self.create_subscription(
            Imu,
            '/sensors/imu/raw',
            self.imu_callback,
            10
        )
        self.odomfil_subscriber = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odomfil_callback,
            10
        )

        self.drifting_publisher = self.create_publisher(Bool, 'is_drifting', 10)

        self.throttle = float('inf')
        self.steering_angle = float('inf')
        self.turning_radius = float('inf')
        self.theor_ang_vel = float('inf')
        self.twist_angular_z = float('inf')
        self.odom_linear_x = float('inf')
        self.odom_linear_y = float('inf')
        self.odomfil_linear_x = float('inf')
        self.odomfil_linear_y = float('inf')
        self.linear_acceleration_x = float('inf')
        self.linear_acceleration_y = float('inf')
        self.linear_acceleration_z = float('inf')
        self.timestamp = 0.0
        self.initial_timestamp = -1

        self.REMOVE_DRIFT_MUS = []
        self.REMOVE_DRIFT_TIMES = []

        self.linear_difference_vals = []
        self.linear_difference_timestamps = []

        self.drifting = False
        self.linear = False
        self.drifting_timestamp = 0.0
        self.drift_length = 1.5

        # ----------------------------
        # Threshold handling (PARAM > FILE > INF)
        # ----------------------------
        self.linear_threshold = float('inf')

        # Declare ROS2 parameter (lets you do: --ros-args -p linear_threshold:=0.123)
        self.declare_parameter('linear_threshold', float('inf'))
        param_thresh = float(self.get_parameter('linear_threshold').value)

        # Track whether threshold came from param (useful so we don't overwrite files during k-fold)
        self.threshold_from_param = np.isfinite(param_thresh)

        if self.threshold_from_param:
            self.linear_threshold = param_thresh
            self.get_logger().info(f"Using linear_threshold from parameter: {self.linear_threshold}")
        else:
            thresh_path = '/home/<YOUR USER>/f1tenth_ws/src/drift_detector/drift_detector/thresholds.txt'
            if os.path.exists(thresh_path):
                with open(thresh_path, 'r') as f:
                    line = f.readline().strip()
                    if line:
                        self.linear_threshold = float(line)
                        self.get_logger().info(f"Using linear_threshold from file: {self.linear_threshold}")
                    else:
                        self.get_logger().warn("thresholds.txt is empty; leaving linear_threshold=inf")
            else:
                self.get_logger().warn("No parameter and no thresholds.txt found; linear_threshold=inf")

        # --- accel-from-odom state ---
        self.prev_odomfil_t = None
        self.prev_odomfil_vx = None
        self.prev_odomfil_vy = None

        self.ax_f = 0.0
        self.ay_f = 0.0
        self.alpha = 0.6  # 0.85-0.95 range; higher = smoother

        # self.create_timer(0.02, self.check_drifting)

        # plt.ion()
        # self.fig, self.ax = plt.subplots()
        # self.line, = self.ax.plot([], [], 'r-')
        # self.mu_times = []
        # self.mu_vals = []
        
        # self.ax.set_title("Live Mu Values During Drift")
        # self.ax.set_xlabel("Time (s)")
        # self.ax.set_ylabel("Mu")

    def odom_callback(self, msg):
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        odom_linear_x = msg.twist.twist.linear.x
        odom_linear_y = msg.twist.twist.linear.y

        self.timestamp = timestamp
        if self.initial_timestamp == -1:
        	self.initial_timestamp = self.timestamp
        self.odom_linear_x = odom_linear_x
        self.odom_linear_y = odom_linear_y

        self.check_drifting()
    
    def ackermann_callback(self, msg):
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        throttle = msg.drive.speed
        steering_angle = msg.drive.steering_angle
        turning_radius = self.wheelbase / np.tan(steering_angle) if steering_angle != 0 else float('inf')
        
        self.timestamp = timestamp
        if self.initial_timestamp == -1:
        	self.initial_timestamp = self.timestamp
        self.throttle = throttle
        self.steering_angle = steering_angle
        self.turning_radius = turning_radius
        self.theor_ang_vel = throttle / turning_radius if turning_radius != 0 else float('inf')

        self.check_drifting()

    def imu_callback(self, msg):
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        angular_velocity = msg.angular_velocity.z
        linear_acceleration_x = msg.linear_acceleration.x
        linear_acceleration_y = msg.linear_acceleration.y
        linear_acceleration_z = msg.linear_acceleration.z

        self.timestamp = timestamp
        if self.initial_timestamp == -1:
        	self.initial_timestamp = self.timestamp
        self.twist_angular_z = angular_velocity
        self.linear_acceleration_x = linear_acceleration_x
        self.linear_acceleration_y = linear_acceleration_y
        self.linear_acceleration_z = linear_acceleration_z

        self.check_drifting()

    def odomfil_callback(self, msg):
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        odom_linear_x = msg.twist.twist.linear.x
        odom_linear_y = msg.twist.twist.linear.y
        wz = msg.twist.twist.angular.z

        self.linear = False

        self.timestamp = timestamp
        if self.initial_timestamp == -1:
        	self.initial_timestamp = self.timestamp

        self.odomfil_linear_x = odom_linear_x
        self.odomfil_linear_y = odom_linear_y

        # -------- estimate accel from odom filtered twist --------
        if self.prev_odomfil_t is not None:
            dt = timestamp - self.prev_odomfil_t
            if dt > 1e-4:
                dvx_dt = (odom_linear_x - self.prev_odomfil_vx) / dt
                dvy_dt = (odom_linear_y - self.prev_odomfil_vy) / dt

                # body-frame planar accel with rigid-body correction
                ax = dvx_dt
                ay = dvy_dt

                ax = self.alpha * self.ax_f + (1.0 - self.alpha) * ax
                ay = self.alpha * self.ay_f + (1.0 - self.alpha) * ay
                self.ax_f, self.ay_f = ax, ay

                # self.linear_acceleration_x = ax
                # self.linear_acceleration_y = ay

                # # keep your existing mu formula happy:
                # # use gravity magnitude as "z accel"
                # self.linear_acceleration_z = self.gravity

        self.prev_odomfil_t = timestamp
        self.prev_odomfil_vx = odom_linear_x
        self.prev_odomfil_vy = odom_linear_y

        self.check_drifting()

    def check_drifting(self):
        if self.throttle == float('inf') or self.steering_angle == float('inf') or self.turning_radius == float('inf'):
            return
        if self.theor_ang_vel == float('inf') or self.twist_angular_z == float('inf') or self.odomfil_linear_x == float('inf') or self.odomfil_linear_y == float('inf'):
            return
        if self.linear_acceleration_x == float('inf') or self.odom_linear_x == float('inf'):
            return
        
        odomfil_comb = 2 * np.sqrt(self.odomfil_linear_x**2 + self.odomfil_linear_y**2)
        odom_comb = 2 * np.sqrt(self.odom_linear_x**2 + self.odom_linear_y**2)
        throttle = self.throttle

        # linear_drift_estimate acts as a slip value...dividing by odom_comb yields slip ratio
        linear_drift_estimate = abs(odomfil_comb - odom_comb)
        self.linear_difference_vals.append(linear_drift_estimate)
        self.linear_difference_timestamps.append(self.timestamp)

        if linear_drift_estimate > self.linear_threshold:  # Threshold for linear drift
            drifting_msg = Bool()
            drifting_msg.data = True
            self.drifting_publisher.publish(drifting_msg)
            if self.timestamp - self.drifting_timestamp > self.drift_length:
                self.drifting = True
                self.drifting_timestamp = self.timestamp
                print(self.drifting_timestamp)
                self.linear = True
        else:
            if self.timestamp - self.drifting_timestamp > self.drift_length:
                self.drifting = False
                if self.mus:
                    print(f"Maximum mu: {np.max(self.mus)} Timestamp: {self.drifting_timestamp}")
                    self.REMOVE_DRIFT_MUS.append(np.max(self.mus))
                    self.REMOVE_DRIFT_TIMES.append(self.drifting_timestamp)
                    self.mus.clear()

        if self.timestamp - self.drifting_timestamp < self.drift_length and self.drifting:
            mu_val = np.sqrt(self.linear_acceleration_x**2 + self.linear_acceleration_y**2) / self.linear_acceleration_z
            # mu_val = np.sqrt(self.linear_acceleration_x**2 + self.linear_acceleration_y**2) / 1
            # mu_val = np.sqrt(((self.linear_acceleration_x * 1.1419258429120134)-0.005057554075893866)**2 + ((self.linear_acceleration_y*1.2509757938552863)-0.0818827085476114)**2) / ((self.linear_acceleration_z*1.2810814105218842)-0.3238420944339122)
            # z = ((self.linear_acceleration_z - 1.0078125) / 0.07470703125)
            # y = (np.tanh(z) + 1) / 2.0 * (1.20166015625 - 0.81787109375) + 0.81787109375 + -0.04115622745183933
            # y = np.clip(y, 0.81787109375, 1.20166015625)
            # mu_val = np.sqrt(self.linear_acceleration_x**2 + self.linear_acceleration_y**2) / y
            self.mus.append(mu_val)

        # mu_val = np.sqrt(self.linear_acceleration_x**2 + self.linear_acceleration_y**2) / self.linear_acceleration_z
        # # append new point
        # self.mu_times.append(self.timestamp - self.initial_timestamp)
        # self.mu_vals.append(mu_val)

        # # sort by time
        # sorted_pairs = sorted(zip(self.mu_times, self.mu_vals), key=lambda x: x[0])
        # self.mu_times, self.mu_vals = map(list, zip(*sorted_pairs))

        # self.line.set_xdata(self.mu_times)
        # self.line.set_ydata(self.mu_vals)
        # self.ax.relim()
        # self.ax.autoscale_view()
        # plt.draw()
        # plt.pause(0.001)

    def on_shutdown(self):

        if self.mus:
            max_mu = np.nanmax(self.mus)
            print(f"[shutdown] Maximum mu: {max_mu} Timestamp: {self.drifting_timestamp}")
            self.REMOVE_DRIFT_MUS.append(max_mu)
            self.REMOVE_DRIFT_TIMES.append(self.drifting_timestamp)
            self.mus.clear()
            
        print('Times:', self.REMOVE_DRIFT_TIMES)
        print('Mus:', self.REMOVE_DRIFT_MUS)
        if self.linear_difference_vals:
            plt.plot(self.linear_difference_vals)
            print(f"Sum of linear mean and standard deviation: {np.mean(self.linear_difference_vals) + 2*np.std(self.linear_difference_vals)}")
            plt.title('Linear Drift Estimates')
            plt.xlabel('Time Steps')
            plt.ylabel('Linear Drift Estimate')
            linear = pd.DataFrame()
            linear['timestamps'] = self.linear_difference_timestamps
            linear['vals'] = self.linear_difference_vals
            linear.to_csv('/home/<YOUR USER>/f1tenth_ws/src/drift_detector/drift_detector/linaer.csv')
            plt.savefig('/home/<YOUR USER>/f1tenth_ws/src/drift_detector/drift_detector/linear_drift_estimates.png')
            plt.close()

        thresh_path = '/home/<YOUR USER>/f1tenth_ws/src/drift_detector/drift_detector/thresholds.txt'
        if (not self.threshold_from_param) and self.linear_difference_vals and (not os.path.exists(thresh_path)):
            with open(thresh_path, 'w') as f:
                f.write(f"{np.mean(self.linear_difference_vals) + 2*np.std(self.linear_difference_vals)}\n")

      
def main(args=None):
    rclpy.init(args=args)
    drift_detector = DriftDetector()
    try:
        rclpy.spin(drift_detector)
    except KeyboardInterrupt:
        pass
    finally:
        drift_detector.on_shutdown()
        drift_detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
