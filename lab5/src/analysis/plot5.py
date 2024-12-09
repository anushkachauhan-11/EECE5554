#!/usr/bin/env python
import rosbag
import numpy as np
import matplotlib.pyplot as plt

bag_path = "/home/chauhan-anu/catkin_ws/src/lab5/src/data/data_driving.bag"
gps_x, gps_y, time_stamps = [], [], []

def extract_gps_data(bag_file):
    global gps_x, gps_y, time_stamps
    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages():
            if "gps" in topic:  
                gps_x.append(msg.utm_easting)  
                gps_y.append(msg.utm_northing)  
                time_stamps.append(msg.header.stamp.to_sec())  
    return np.array(gps_x), np.array(gps_y), np.array(time_stamps)

def compute_velocity(x, y, time):
    distances = np.sqrt(np.diff(x)**2 + np.diff(y)**2)
    time_intervals = np.diff(time)
    velocity = distances / time_intervals
    velocity = np.insert(velocity, 0, 0)

    return velocity

def main():
    global gps_x, gps_y, time_stamps
    print("Extracting GPS data from bag...")
    gps_x, gps_y, time_stamps = extract_gps_data(bag_path)
    print(f"Number of Data Points: {len(gps_x)}")
    print(f"GPS Sample (X, Y): {gps_x[:5]}, {gps_y[:5]}")
    print(f"Time Stamps Sample: {time_stamps[:5]}")

    forward_velocity = compute_velocity(gps_x, gps_y, time_stamps)

    print(f"Forward Velocity (m/s): {forward_velocity[:5]}")

    plt.figure(figsize=(12, 6))
    plt.plot(time_stamps, forward_velocity, label="GPS Forward Velocity", color="blue")
    plt.title("Forward Velocity from GPS")
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (m/s)")
    plt.legend()
    plt.grid()
    plt.show()


if __name__ == "__main__":
    main()
