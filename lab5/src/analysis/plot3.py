#!/usr/bin/env python
import rosbag
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt
from scipy.integrate import cumtrapz

bag_path = "/home/chauhan-anu/catkin_ws/src/lab5/src/data/data_driving.bag"
mag_x, mag_y, gyro_z, time_stamps = [], [], [], []

def extract_data(bag_file):
    global mag_x, mag_y, gyro_z, time_stamps
    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages():
            if "imu" in topic:  
                mag_x.append(msg.mag_field.magnetic_field.x)
                mag_y.append(msg.mag_field.magnetic_field.y)
                gyro_z.append(msg.imu.angular_velocity.z)
                time_stamps.append(msg.header.stamp.to_sec())
    return (
        np.array(mag_x),
        np.array(mag_y),
        np.array(gyro_z),
        np.array(time_stamps),
    )

def low_pass_filter(data, cutoff, fs, order=2):
    nyquist = 0.5 * fs
    normal_cutoff = cutoff / nyquist
    b, a = butter(order, normal_cutoff, btype="low", analog=False)
    return filtfilt(b, a, data)

def high_pass_filter(data, cutoff, fs, order=2):
    nyquist = 0.5 * fs
    normal_cutoff = cutoff / nyquist
    b, a = butter(order, normal_cutoff, btype="high", analog=False)
    return filtfilt(b, a, data)

def complementary_filter(mag_yaw, gyro_yaw, alpha=0.98):
    return alpha * gyro_yaw + (1 - alpha) * mag_yaw

def main():
    global mag_x, mag_y, gyro_z, time_stamps
    print("Extracting data from bag...")
    mag_x, mag_y, gyro_z, time_stamps = extract_data(bag_path)

    print(f"Number of Data Points: {len(mag_x)}")
    print(f"Magnetometer Sample (X, Y): {mag_x[:5]}, {mag_y[:5]}")
    print(f"Gyroscope Sample (Z): {gyro_z[:5]}")
    print(f"Time Stamps Sample: {time_stamps[:5]}")

    mag_yaw = np.arctan2(mag_y, mag_x)
    gyro_yaw = cumtrapz(gyro_z, time_stamps, initial=0)
    fs = 50  
    low_cutoff = 1.0 
    high_cutoff = 0.1  

    mag_yaw_filtered = low_pass_filter(mag_yaw, low_cutoff, fs)
    gyro_yaw_filtered = high_pass_filter(gyro_yaw, high_cutoff, fs)
    fused_yaw = complementary_filter(mag_yaw_filtered, gyro_yaw_filtered)

    print(f"Magnetometer Yaw (Filtered): {mag_yaw_filtered[:5]}")
    print(f"Gyroscope Yaw (Filtered): {gyro_yaw_filtered[:5]}")
    print(f"Fused Yaw (Complementary): {fused_yaw[:5]}")

    plt.figure(figsize=(14, 12))
    plt.subplot(4, 1, 1)
    plt.plot(time_stamps, np.degrees(mag_yaw_filtered), color="blue", label="Low-pass Filtered")
    plt.title("Low-pass Filtered Magnetometer Yaw")
    plt.ylabel("Yaw (degrees)")
    plt.grid()
    plt.legend()

    plt.subplot(4, 1, 2)
    plt.plot(time_stamps, np.degrees(gyro_yaw_filtered), color="orange", label="High-pass Filtered")
    plt.title("High-pass Filtered Gyroscope Yaw")
    plt.ylabel("Yaw (degrees)")
    plt.grid()
    plt.legend()

    plt.subplot(4, 1, 3)
    plt.plot(time_stamps, np.degrees(fused_yaw), color="green", label="Complementary Filter Output")
    plt.title("Complementary Filter Output")
    plt.ylabel("Yaw (degrees)")
    plt.grid()
    plt.legend()

    plt.subplot(4, 1, 4)
    plt.plot(time_stamps, np.degrees(fused_yaw), color="purple", label="IMU Heading Estimate")
    plt.title("IMU Heading Estimate")
    plt.xlabel("Time (s)")
    plt.ylabel("Yaw (degrees)")
    plt.grid()
    plt.legend()

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
