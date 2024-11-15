import datetime
import rosbag
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import utm

def load_rosbag(bag_path, topic):
    bag = rosbag.Bag(bag_path)

    data = {'timestamp': [], 'latitude': [], 'longitude': [], 'altitude': [], 'easting': [], 'northing': []}

    for _, msg, t in bag.read_messages(topics=[topic]):
        data['timestamp'].append(t.to_sec())
        data['latitude'].append(msg.latitude)
        data['longitude'].append(msg.longitude)
        data['altitude'].append(msg.altitude)
        utm_coords = utm.from_latlon(msg.latitude, msg.longitude)
        data['easting'].append(utm_coords[0])
        data['northing'].append(utm_coords[1])

    bag.close()

    return pd.DataFrame(data)

def analyze_moving_data(open_df, occluded_df):
    open_df['timestamp'] = pd.to_datetime(open_df['timestamp'], unit='s')
    occluded_df['timestamp'] = pd.to_datetime(occluded_df['timestamp'], unit='s')

    open_easting = open_df['easting'].to_numpy()
    open_northing = open_df['northing'].to_numpy()
    open_altitude = open_df['altitude'].to_numpy()

    occluded_easting = occluded_df['easting'].to_numpy()
    occluded_northing = occluded_df['northing'].to_numpy()
    occluded_altitude = occluded_df['altitude'].to_numpy()

    open_timestamps = open_df['timestamp'].values
    occluded_timestamps = occluded_df['timestamp'].values

    open_start_time = open_timestamps[0]
    open_time_seconds = (open_timestamps - open_start_time) / np.timedelta64(1, 's')

    occluded_start_time = occluded_timestamps[0]
    occluded_time_seconds = (occluded_timestamps - occluded_start_time) / np.timedelta64(1, 's')

    plt.figure()
    
    plt.scatter(open_easting, open_northing, c='#FF69B4', label='Open Data', marker='o', edgecolor='black', alpha=0.7)  # Pinkish color for Open Data
    plt.scatter(occluded_easting, occluded_northing, c='#800080', label='Occluded Data', marker='X', edgecolor='black', alpha=0.7)  # Purple for Occluded Data

    plt.title('Moving Data - Northing vs Easting')
    plt.xlabel('Easting (meters)')
    plt.ylabel('Northing (meters)')
    
    legend = plt.legend()
    frame = legend.get_frame()
    frame.set_facecolor('lightyellow')
    frame.set_edgecolor('black')
    frame.set_alpha(0.9)

    print("Scatter plot created")

    open_coeffs = np.polyfit(open_easting, open_northing, 1)
    open_best_fit_line = np.poly1d(open_coeffs)
    open_error = np.sqrt(np.mean((open_northing - open_best_fit_line(open_easting))**2))
    print("Error from the best-fit line to the open data:", open_error)
    plt.plot(open_easting, open_best_fit_line(open_easting), color='#FF4500', label='Open Best-fit Line') 
    occluded_coeffs = np.polyfit(occluded_easting, occluded_northing, 1)
    occluded_best_fit_line = np.poly1d(occluded_coeffs)
    occluded_error = np.sqrt(np.mean((occluded_northing - occluded_best_fit_line(occluded_easting))**2))
    print("Error from the best-fit line to the occluded data:", occluded_error)
    plt.plot(occluded_easting, occluded_best_fit_line(occluded_easting), color='#006400', label='Occluded Best-fit Line')  
    
    legend = plt.legend()
    frame = legend.get_frame()
    frame.set_facecolor('lightyellow')
    frame.set_edgecolor('black')
    frame.set_alpha(0.9)
    plt.figure()
    
    for i in range(len(open_time_seconds) - 1):
        plt.plot(open_time_seconds[i:i+2], open_altitude[i:i+2], color='#FF69B4', marker='o', label='Open Data' if i == 0 else "")  # Pinkish for Open Data

    for i in range(len(occluded_time_seconds) - 1):
        plt.plot(occluded_time_seconds[i:i+2], occluded_altitude[i:i+2], color='#800080', marker='X', label='Occluded Data' if i == 0 else "")  # Purple for Occluded Data

    plt.title('Moving Data Altitude vs. Time')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Altitude (meters)')
    
    legend = plt.legend()
    frame = legend.get_frame()
    frame.set_facecolor('lightyellow')
    frame.set_edgecolor('black')
    frame.set_alpha(0.9)
    plt.grid()
    print("Altitude plot created")

if __name__ == '__main__':
    open_df = load_rosbag('/home/chauhan-anu/catkin_ws/src/lab2git/data/square_open_area.bag', '/gps')
    occluded_df = load_rosbag('/home/chauhan-anu/catkin_ws/src/lab2git/data/square_partially_occluded.bag', '/gps')
    analyze_moving_data(open_df, occluded_df)
    plt.show()
