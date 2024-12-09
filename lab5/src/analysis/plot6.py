#!/usr/bin/env python
import rosbag
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import cumtrapz
from scipy.ndimage import gaussian_filter1d

bag_path = "/home/chauhan-anu/catkin_ws/src/lab5/src/data/data_driving.bag"

gps_x, gps_y, time_stamps_gps = [], [], []
imu_acceleration, imu_yaw_rate, time_stamps_imu = [], [], []

def extract_gps_data(bag_file):
    gps_x, gps_y, time_stamps_gps = [], [], []
    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages():
            if "gps" in topic:  
                gps_x.append(msg.utm_easting)
                gps_y.append(msg.utm_northing)
                time_stamps_gps.append(msg.header.stamp.to_sec())
    return np.array(gps_x), np.array(gps_y), np.array(time_stamps_gps)

def extract_imu_data(bag_file):
    imu_acceleration, imu_yaw_rate, time_stamps_imu = [], [], []
    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages():
            if "imu" in topic:  
                imu_acceleration.append(msg.imu.linear_acceleration.x)  
                imu_yaw_rate.append(msg.imu.angular_velocity.z)  
                time_stamps_imu.append(msg.header.stamp.to_sec())
    return (
        np.array(imu_acceleration),
        np.array(imu_yaw_rate),
        np.array(time_stamps_imu),
    )

def compute_imu_trajectory(acceleration, yaw_rate, time_stamps):
    yaw_rate_smooth = gaussian_filter1d(yaw_rate, sigma=2)
    acceleration_smooth = gaussian_filter1d(acceleration, sigma=2)
    velocity = cumtrapz(acceleration_smooth, time_stamps, initial=0)
    print(f"Velocity range: min={np.min(velocity)}, max={np.max(velocity)}")
    yaw = cumtrapz(yaw_rate_smooth, time_stamps, initial=0)
    yaw = np.mod(yaw, 2 * np.pi)  
    print(f"Yaw range: min={np.min(yaw)}, max={np.max(yaw)}")

    dx = velocity * np.cos(yaw)
    dy = velocity * np.sin(yaw)
    x = cumtrapz(dx, time_stamps, initial=0)
    y = cumtrapz(dy, time_stamps, initial=0)

    return x, y

def main():
    print("Extracting GPS data...")
    gps_x, gps_y, time_stamps_gps = extract_gps_data(bag_path)
    print("Extracting IMU data...")
    imu_acceleration, imu_yaw_rate, time_stamps_imu = extract_imu_data(bag_path)
    print("Computing IMU trajectory...")
    imu_x, imu_y = compute_imu_trajectory(imu_acceleration, imu_yaw_rate, time_stamps_imu)

    print("Applying scaling factors...")
    if (np.max(imu_x) - np.min(imu_x)) > 0 and (np.max(imu_y) - np.min(imu_y)) > 0:
        scaling_factor_x = (np.max(gps_x) - np.min(gps_x)) / (np.max(imu_x) - np.min(imu_x))
        scaling_factor_y = (np.max(gps_y) - np.min(gps_y)) / (np.max(imu_y) - np.min(imu_y))
        print(f"Scaling Factor X: {scaling_factor_x}, Scaling Factor Y: {scaling_factor_y}")
        imu_x = imu_x * scaling_factor_x + (gps_x[0] - imu_x[0])
        imu_y = imu_y * scaling_factor_y + (gps_y[0] - imu_y[0])
    else:
        print("IMU trajectory range is zero. Cannot compute scaling factors.")

    plt.figure(figsize=(14, 12))
    plt.subplot(2, 1, 1)
    plt.plot(gps_x, gps_y, label="GPS Trajectory", color="blue")
    plt.title("Trajectory from GPS Data")
    plt.xlabel("UTM Easting (m)")
    plt.ylabel("UTM Northing (m)")
    plt.legend()
    plt.grid()

    plt.subplot(2, 1, 2)
    plt.plot(imu_x, imu_y, label="IMU Trajectory", color="orange")
    plt.title("Trajectory from IMU Data")
    plt.xlabel("X Position (m)")
    plt.ylabel("Y Position (m)")
    plt.legend()
    plt.grid()

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
