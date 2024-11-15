import datetime
import rosbag
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def load_rosbag(bag_path, topic):
    bag = rosbag.Bag(bag_path)

    data = {'timestamp': [], 'latitude': [], 'longitude': [], 'altitude': [], 'easting': [], 'northing': []}

    for _, msg, t in bag.read_messages(topics=[topic]):
        data['timestamp'].append(t.to_sec())
        data['latitude'].append(msg.latitude)
        data['longitude'].append(msg.longitude)
        data['altitude'].append(msg.altitude)
        data['easting'].append(msg.utm_easting)
        data['northing'].append(msg.utm_northing)

    bag.close()

    return pd.DataFrame(data)


def analyze_moving_data(df, location_type):
    df['timestamp'] = pd.to_datetime(df['timestamp'], unit='s')
    easting = df['easting'].to_numpy()
    northing = df['northing'].to_numpy()
    altitude = df['altitude'].to_numpy()
    timestamps = df['timestamp'].values
    start_time = timestamps[0]
    time_seconds = (timestamps - start_time) / np.timedelta64(1, 's')

    print("Timestamps:", timestamps)  

    plt.figure()
    plt.scatter(easting, northing, label=f'{location_type} Data', marker='o')
    plt.title(f'Moving {location_type} Data')
    plt.xlabel('Easting (meters)')
    plt.ylabel('Northing (meters)')
    plt.legend()

    print("Scatter plot created")

    coeffs = np.polyfit(easting, northing, 1)
    best_fit_line = np.poly1d(coeffs)
    error = np.sqrt(np.mean((northing - best_fit_line(easting))**2))

    print("Error from the best-fit line to the moving data:", error)
    plt.plot(easting, best_fit_line(easting), color='red', label='Best-fit Line')
    plt.legend()
    plt.figure()
    plt.plot(time_seconds, altitude, label=f'{location_type} Data', marker='x', color='red') 
    plt.title(f'Moving {location_type} Data Altitude vs. Time')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Altitude (meters)')
    plt.legend()

    print("Altitude plot created")

if __name__ == '__main__':
    walking_df = load_rosbag('/home/chauhan-anu/catkin_ws/src/gnss/data/moving_data.bag', '/gps')
    print("DataFrame shape:", walking_df.shape)  
    analyze_moving_data(walking_df, 'Walking')
    print("Plots created")
    plt.show()
