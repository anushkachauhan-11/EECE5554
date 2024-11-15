import datetime as dt
import rosbag
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import utm

def extract_rosbag_data(bag_file, topic_name):
    bag = rosbag.Bag(bag_file)

    data_record = {
        'timestamp': [],
        'latitude': [],
        'longitude': [],
        'altitude': [],
        'easting': [],
        'northing': []
    }

    for _, msg, t in bag.read_messages(topics=[topic_name]):
        data_record['timestamp'].append(t.to_sec())
        data_record['latitude'].append(msg.latitude)
        data_record['longitude'].append(msg.longitude)
        data_record['altitude'].append(msg.altitude)
        
        utm_coords = utm.from_latlon(msg.latitude, msg.longitude)
        data_record['easting'].append(utm_coords[0])
        data_record['northing'].append(utm_coords[1])

    bag.close()

    return pd.DataFrame(data_record)

def evaluate_stationary_positions(open_data, occluded_data):
    open_east = np.array(open_data['easting'])
    open_north = np.array(open_data['northing'])
    open_alt = np.array(open_data['altitude'])
    occluded_east = np.array(occluded_data['easting'])
    occluded_north = np.array(occluded_data['northing'])
    occluded_alt = np.array(occluded_data['altitude'])

    openoffset_east = open_east[0]
    openoffset_north = open_north[0]
    open_east_centered = open_east - openoffset_east
    open_north_centered = open_north - openoffset_north
    occludedoffset_east = occluded_east[0]
    occludedoffset_north = occluded_north[0]
    occluded_east_centered = occluded_east - occludedoffset_east
    occluded_north_centered = occluded_north - occludedoffset_north

    open_centroid_east = np.mean(open_east_centered)
    open_centroid_north = np.mean(open_north_centered)
    occluded_centroid_east = np.mean(occluded_east_centered)
    occluded_centroid_north = np.mean(occluded_north_centered)

    open_deviations_east = open_east_centered - open_centroid_east
    open_deviations_north = open_north_centered - open_centroid_north
    occluded_deviations_east = occluded_east_centered - occluded_centroid_east
    occluded_deviations_north = occluded_north_centered - occluded_centroid_north

    plt.figure()
    plt.scatter(open_deviations_east, open_deviations_north, label='Open Area', marker='o', color='blue', alpha=0.6)
    plt.scatter(occluded_deviations_east, occluded_deviations_north, label='Obstructed Area', marker='o', color='red', alpha=0.6)
    plt.scatter(open_centroid_east, open_centroid_north, label='Open Centroid', marker='x', color='green', s=150)
    plt.scatter(occluded_centroid_east, occluded_centroid_north, label='Occluded Centroid', marker='x', color='orange', s=150)
    plt.title('Stationary Northing vs. Easting Scatterplots')
    plt.xlabel('Easting (meters)')
    plt.ylabel('Northing (meters)')
    plt.legend()
    plt.grid()
    plt.xlim([-10, 10])
    plt.ylim([-10, 10])

    plt.figure()
    plt.plot(open_data['timestamp'], open_alt, label='Open Area', marker='o', color='blue', alpha=0.6)
    plt.plot(occluded_data['timestamp'], occluded_alt, label='Obstructed Area', marker='o', color='red', alpha=0.6)
    plt.title('Stationary Altitude vs. Time Plot')
    plt.xlabel('Timestamp (s)')
    plt.ylabel('Altitude (meters)')
    plt.legend()
    plt.grid()

    open_distances = np.sqrt((open_east - open_centroid_east)**2 + (open_north - open_centroid_north)**2)
    occluded_distances = np.sqrt((occluded_east - occluded_centroid_east)**2 + (occluded_north - occluded_centroid_north)**2)

    plt.figure()
    plt.hist(open_distances, bins=20, alpha=0.8, color='blue', edgecolor='black', linewidth=1.2)
    plt.title('Histogram of Euclidean Distance from Centroid - Open Area')
    plt.xlabel('Distance (meters)')
    plt.ylabel('Frequency')
    plt.grid()

    plt.figure()
    plt.hist(occluded_distances, bins=20, alpha=0.8, color='red', edgecolor='black', linewidth=1.2)
    plt.title('Histogram of Euclidean Distance from Centroid - Obstructed Area')
    plt.xlabel('Distance (meters)')
    plt.ylabel('Frequency')
    plt.grid()

if __name__ == '__main__':
    open_area_df = extract_rosbag_data('/home/chauhan-anu/catkin_ws/src/lab2git/data/stationary_open_area.bag', '/gps/fix')
    occluded_area_df = extract_rosbag_data('/home/chauhan-anu/catkin_ws/src/lab2git/data/stationary_partially_occluded.bag', '/gps/fix')
    evaluate_stationary_positions(open_area_df, occluded_area_df)
    plt.show()
