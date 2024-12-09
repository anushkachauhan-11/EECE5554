#!/usr/bin/env python
import rosbag
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt
from scipy.integrate import cumtrapz

bag_path = "/home/chauhan-anu/catkin_ws/src/lab5/src/data/data_driving.bag"

accel_x, time_stamps = [], []

def extract_accelerometer_data(bag_file):
    global accel_x, time_stamps
    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages():
            if "imu" in topic:  
                accel_x.append(msg.imu.linear_acceleration.x)  
                time_stamps.append(msg.header.stamp.to_sec())
    return np.array(accel_x), np.array(time_stamps)

def low_pass_filter(data, cutoff, fs, order=2):
    nyquist = 0.5 * fs
    normal_cutoff = cutoff / nyquist
    b, a = butter(order, normal_cutoff, btype="low", analog=False)
    return filtfilt(b, a, data)

def main():
    global accel_x, time_stamps
    print("Extracting accelerometer data from bag...")
    accel_x, time_stamps = extract_accelerometer_data(bag_path)
    print(f"Number of Data Points: {len(accel_x)}")
    print(f"Accelerometer Sample (X): {accel_x[:5]}")
    print(f"Time Stamps Sample: {time_stamps[:5]}")
    raw_velocity = cumtrapz(accel_x, time_stamps, initial=0)
    fs = 50  
    cutoff = 2.0  
    accel_x_filtered = low_pass_filter(accel_x, cutoff, fs)
    adjusted_velocity = cumtrapz(accel_x_filtered, time_stamps, initial=0)
    print(f"Raw Velocity (m/s): {raw_velocity[:5]}")
    print(f"Adjusted Velocity (m/s): {adjusted_velocity[:5]}")

    plt.figure(figsize=(12, 6))
    plt.plot(time_stamps, raw_velocity, label="Raw Velocity (Before Adjustment)", color="red", linestyle="--")
    plt.plot(time_stamps, adjusted_velocity, label="Adjusted Velocity (After Adjustment)", color="blue")
    plt.title("Forward Velocity from Accelerometer")
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (m/s)")
    plt.legend()
    plt.grid()
    plt.show()

if __name__ == "__main__":
    main()
