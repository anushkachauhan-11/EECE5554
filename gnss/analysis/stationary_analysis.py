import datetime
import rosbag
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

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

def analyze_stationary_data(open_df, occluded_df):
    # Convert timestamp to NumPy array for Open data
    open_df['timestamp'] = open_df['timestamp'].apply(lambda x: datetime.datetime.fromtimestamp(x))
    open_df['timestamp'] = pd.to_datetime(open_df['timestamp'])
    open_df['timestamp'] = open_df['timestamp'].apply(lambda x: x.to_pydatetime())

    # Convert timestamp to NumPy array for Occluded data
    occluded_df['timestamp'] = occluded_df['timestamp'].apply(lambda x: datetime.datetime.fromtimestamp(x))
    occluded_df['timestamp'] = pd.to_datetime(occluded_df['timestamp'])
    occluded_df['timestamp'] = occluded_df['timestamp'].apply(lambda x: x.to_pydatetime())

    # Convert columns to NumPy arrays
    open_easting = open_df['easting'].to_numpy()
    open_northing = open_df['northing'].to_numpy()
    open_altitude = open_df['altitude'].to_numpy()

    occluded_easting = occluded_df['easting'].to_numpy()
    occluded_northing = occluded_df['northing'].to_numpy()
    occluded_altitude = occluded_df['altitude'].to_numpy()

    # Calculate centroids for Open data
    open_centroid = {
        'easting': np.mean(open_easting),
        'northing': np.mean(open_northing)
    }

    # Calculate centroids for Occluded data
    occluded_centroid = {
        'easting': np.mean(occluded_easting),
        'northing': np.mean(occluded_northing)
    }

    # Plot northing vs. easting scatterplots for both Open and Occluded data
    plt.figure()
    plt.scatter(open_easting, open_northing, label='Open Spot', marker='o', color='blue')
    plt.scatter(occluded_easting, occluded_northing, label='Occluded Spot', marker='o', color='red')
    plt.scatter(open_centroid['easting'], open_centroid['northing'], label='Open Centroid', marker='x', color='green')
    plt.scatter(occluded_centroid['easting'], occluded_centroid['northing'], label='Occluded Centroid', marker='x', color='orange')
    plt.title('Stationary Northing vs. Easting Scatterplots')
    plt.xlabel('Easting (meters)')
    plt.ylabel('Northing (meters)')
    plt.legend()

    # Plot altitude vs. time plot for both Open and Occluded data
    plt.figure()
    plt.plot(open_df['timestamp'].values, open_altitude, label='Open Spot', marker='o', color='blue')
    plt.plot(occluded_df['timestamp'].values, occluded_altitude, label='Occluded Spot', marker='x', color='red')
    plt.title('Stationary Altitude vs. Time Plot')
    plt.xlabel('Timestamp')
    plt.ylabel('Altitude (meters)')
    plt.legend()

    # Calculate Euclidean distance from each point to the centroid for Open data
    open_distances = np.sqrt((open_easting - open_centroid['easting'])**2 + (open_northing - open_centroid['northing'])**2)

    # Calculate Euclidean distance from each point to the centroid for Occluded data
    occluded_distances = np.sqrt((occluded_easting - occluded_centroid['easting'])**2 + (occluded_northing - occluded_centroid['northing'])**2)

    # Print mean errors
    distance1 = np.mean(open_distances)
    distance2 = np.mean(occluded_distances)
    print("Error for Open data:", distance1)
    print("Error for Occluded data:", distance2)

    # Plot histogram of distances for Open Spot
    plt.figure()
    plt.hist(open_distances, bins=20, alpha=0.7, label='Open Spot - Distance from Centroid', color='blue')
    plt.title('Open Spot Distance from Centroid Histogram')
    plt.xlabel('Distance (meters)')
    plt.ylabel('Frequency')
    plt.legend()

    # Plot histogram of distances for Occluded Spot
    plt.figure()
    plt.hist(occluded_distances, bins=20, alpha=0.7, label='Occluded Spot - Distance from Centroid', color='red')
    plt.title('Occluded Spot Distance from Centroid Histogram')
    plt.xlabel('Distance (meters)')
    plt.ylabel('Frequency')
    plt.legend()

if __name__ == '__main__':
    open_spot_df = load_rosbag('/home/chauhan-anu/catkin_ws/src/gnss/data/no_building_stationary.bag', '/gps')
    occluded_spot_df = load_rosbag('/home/chauhan-anu/catkin_ws/src/gnss/data/building_stationary.bag', '/gps')

    # Analyze stationary data
    analyze_stationary_data(open_spot_df, occluded_spot_df)

    # Show the plots
    plt.show()
