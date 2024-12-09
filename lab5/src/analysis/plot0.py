#!/usr/bin/env python
import rosbag
import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import lstsq

BAG_FILE_PATH = "/home/chauhan-anu/catkin_ws/src/lab5/src/data/data_going_in_circles.bag"
TOPIC_NAME = "/imu"  

def extract_magnetometer_data(bag_file_path):
    """
    Extracts magnetometer data (x, y) from the specified ROS bag file.
    """
    mag_x, mag_y = [], []
    
    print(f"Reading bag file: {bag_file_path}")
    try:
        with rosbag.Bag(bag_file_path, 'r') as bag:
            for topic, msg, t in bag.read_messages(topics=[TOPIC_NAME]):
                try:
                    mag_x.append(msg.mag_field.magnetic_field.x)
                    mag_y.append(msg.mag_field.magnetic_field.y)
                except AttributeError:
                    print(f"Message does not have magnetic_field data: {msg}")
    except Exception as e:
        print(f"Error reading bag file: {e}")
    
    return np.array(mag_x), np.array(mag_y)

def calibrate_magnetometer(x, y):
    if len(x) == 0 or len(y) == 0:
        print("No magnetometer data to calibrate.")
        return x, y

    x_centered = x - np.mean(x)
    y_centered = y - np.mean(y)

    print(f"Centered Data Means: X={np.mean(x_centered):.4f}, Y={np.mean(y_centered):.4f}")
    
    D = np.array([x_centered**2, x_centered * y_centered, y_centered**2]).T
    b = np.ones_like(x_centered)
    
    coeffs, _, _, _ = lstsq(D, b)
    a, b, c = coeffs

    print(f"Ellipse Coefficients: a={a:.4f}, b={b:.4f}, c={c:.4f}")

    scale_x = np.sqrt(1 / abs(a))
    scale_y = np.sqrt(1 / abs(c))

    print(f"Scaling Factors: scale_x={scale_x:.4f}, scale_y={scale_y:.4f}")

    x_corrected = x_centered * scale_x
    y_corrected = y_centered * scale_y

    return x_corrected, y_corrected

def main():
    mag_x, mag_y = extract_magnetometer_data(BAG_FILE_PATH)
    print(f"Extracted {len(mag_x)} magnetometer data points.")
    if len(mag_x) == 0 or len(mag_y) == 0:
        print("No data points found in the bag file. Exiting.")
        return
    x_corrected, y_corrected = calibrate_magnetometer(mag_x, mag_y)

    plt.figure()
    plt.scatter(mag_x, mag_y, label="Raw Data (Before Correction)", alpha=0.5, color="red")
    plt.scatter(x_corrected, y_corrected, label="Calibrated Data (After Correction)", alpha=0.5, color="blue")
    plt.title("Magnetometer Calibration: Before and After Correction")
    plt.xlabel("Magnetometer X")
    plt.ylabel("Magnetometer Y")
    plt.legend()
    plt.grid()
    plt.show()

if __name__ == "__main__":
    main()
