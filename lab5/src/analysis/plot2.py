#!/usr/bin/env python
import rosbag
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import cumtrapz

bag_path = "/home/chauhan-anu/catkin_ws/src/lab5/src/data/data_driving.bag"
gyro_z, time_stamps = [], []

def extract_gyro_data(bag_file):
    global gyro_z, time_stamps
    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages():
            if "imu" in topic: 
                gyro_z.append(msg.imu.angular_velocity.z)  
                time_stamps.append(msg.header.stamp.to_sec())
    return np.array(gyro_z), np.array(time_stamps)

def main():
    global gyro_z, time_stamps
    print("Extracting gyroscope data from bag...")
    gyro_z, time_stamps = extract_gyro_data(bag_path)
    print(f"Number of Data Points: {len(gyro_z)}")
    print(f"Gyroscope Sample (Z): {gyro_z[:5]}")
    print(f"Time Stamps Sample: {time_stamps[:5]}")
    gyro_yaw = cumtrapz(gyro_z, time_stamps, initial=0)
    print(f"Gyro Yaw (Radians): {gyro_yaw[:5]}")

    plt.figure(figsize=(10, 6))
    plt.plot(time_stamps, np.degrees(gyro_yaw), label="Gyroscope Yaw", color="orange")
    plt.title("Gyroscope Yaw Estimation vs. Time")
    plt.xlabel("Time (s)")
    plt.ylabel("Yaw (degrees)")
    plt.legend()
    plt.grid()
    plt.show()

if __name__ == "__main__":
    main()
