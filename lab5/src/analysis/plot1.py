#!/usr/bin/env python
import rosbag
import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import lstsq
from scipy.signal import butter, filtfilt
from scipy.integrate import cumtrapz

bag_path = "/home/chauhan-anu/catkin_ws/src/lab5/src/data/data_going_in_circles.bag"
mag_x, mag_y, time_stamps = [], [], []

def extract_magnetometer_data(bag_file):
    global mag_x, mag_y, time_stamps
    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages():
            if "imu" in topic:  
                mag_x.append(msg.mag_field.magnetic_field.x)
                mag_y.append(msg.mag_field.magnetic_field.y)
                time_stamps.append(msg.header.stamp.to_sec())
    return np.array(mag_x), np.array(mag_y), np.array(time_stamps)

def calibrate_magnetometer(x, y):
    x_centered = x - np.mean(x)
    y_centered = y - np.mean(y)
    D = np.array([x_centered**2, x_centered * y_centered, y_centered**2]).T
    b = np.ones_like(x_centered)
    coeffs, _, _, _ = lstsq(D, b)
    a, b, c = coeffs
    scale_x = np.sqrt(1 / abs(a))
    scale_y = np.sqrt(1 / abs(c))
    x_corrected = x_centered * scale_x
    y_corrected = y_centered * scale_y

    return x_corrected, y_corrected

def low_pass_filter(data, cutoff, fs, order=2):
    nyquist = 0.5 * fs
    normal_cutoff = cutoff / nyquist
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return filtfilt(b, a, data)

def main():
    global mag_x, mag_y, time_stamps
    print("Extracting data from bag...")
    mag_x, mag_y, time_stamps = extract_magnetometer_data(bag_path)
    print(f"Number of Data Points: {len(mag_x)}")
    print(f"Magnetometer Sample (X, Y): {mag_x[:5]}, {mag_y[:5]}")
    print(f"Time Stamps Sample: {time_stamps[:5]}")

    x_calibrated, y_calibrated = calibrate_magnetometer(mag_x, mag_y)
    yaw_mag = np.arctan2(y_calibrated, x_calibrated)
    fs = 50  
    cutoff = 1  
    yaw_mag_filtered = low_pass_filter(yaw_mag, cutoff, fs)

    print(f"Raw Yaw (Radians): {yaw_mag[:5]}")
    print(f"Filtered Yaw (Radians): {yaw_mag_filtered[:5]}")

    plt.figure(figsize=(10, 6))
    plt.plot(time_stamps, np.degrees(yaw_mag), label="Yaw (Raw Magnetometer)", linestyle="--", color="red")
    plt.plot(time_stamps, np.degrees(yaw_mag_filtered), label="Yaw (Calibrated Magnetometer)", color="blue")
    plt.title("Magnetometer Yaw Estimation Before and After Calibration vs. Time")
    plt.xlabel("Time (s)")
    plt.ylabel("Yaw (degrees)")
    plt.legend()
    plt.grid()
    plt.show()

if __name__ == "__main__":
    main()
