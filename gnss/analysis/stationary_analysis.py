def analyze_stationary_positions(open_area_df, occluded_area_df):
    """Analyzes and visualizes stationary GPS data for open and occluded environments."""
    # Convert data into arrays
    open_easting = np.array(open_area_df['easting'])
    open_northing = np.array(open_area_df['northing'])
    open_altitude = np.array(open_area_df['altitude'])
    occluded_easting = np.array(occluded_area_df['easting'])
    occluded_northing = np.array(occluded_area_df['northing'])
    occluded_altitude = np.array(occluded_area_df['altitude'])

    # Center the data by subtracting the first point
    open_easting_centered = open_easting - open_easting[0]
    open_northing_centered = open_northing - open_northing[0]
    occluded_easting_centered = occluded_easting - occluded_easting[0]
    occluded_northing_centered = occluded_northing - occluded_northing[0]

    # Calculate centroids
    open_centroid_easting = np.mean(open_easting_centered)
    open_centroid_northing = np.mean(open_northing_centered)
    occluded_centroid_easting = np.mean(occluded_easting_centered)
    occluded_centroid_northing = np.mean(occluded_northing_centered)

    # Calculate deviations
    open_deviations_easting = open_easting_centered - open_centroid_easting
    open_deviations_northing = open_northing_centered - open_centroid_northing
    occluded_deviations_easting = occluded_easting_centered - occluded_centroid_easting
    occluded_deviations_northing = occluded_northing_centered - occluded_centroid_northing

    # Stationary northing vs. easting scatterplots
    plt.figure()
    plt.scatter(open_deviations_easting, open_deviations_northing, label='Open Area', 
                marker='o', color='#FFB6C1', alpha=0.7)  # Light Pink
    plt.scatter(occluded_deviations_easting, occluded_deviations_northing, label='Obstructed Area', 
                marker='o', color='#FF69B4', alpha=0.7)  # Hot Pink
    plt.scatter(open_centroid_easting, open_centroid_northing, label='Open Centroid', 
                marker='x', color='#FF1493', s=100)  # Deep Pink
    plt.scatter(occluded_centroid_easting, occluded_centroid_northing, label='Occluded Centroid', 
                marker='x', color='#C71585', s=100)  # Medium Violet Red

    # Annotate centroid values
    plt.annotate(f"Open Centroid: ({open_centroid_easting:.2f}, {open_centroid_northing:.2f})", 
                 xy=(2, 0.5), 
                 xytext=(0, 0), 
                 textcoords='offset points', 
                 color='blue')
    plt.annotate(f"Occluded Centroid: ({occluded_centroid_easting:.2f}, {occluded_centroid_northing:.2f})", 
                 xy=(3.5, 0.3), 
                 xytext=(5, -20), 
                 textcoords='offset points', 
                 color='green')

    plt.title('Stationary Northing vs. Easting Scatterplots')
    plt.xlabel('Easting (meters)')
    plt.ylabel('Northing (meters)')
    plt.legend()
    plt.grid()

    # Stationary altitude vs. time plot
    plt.figure()
    open_timestamps = np.array(open_area_df['timestamp'])
    occluded_timestamps = np.array(occluded_area_df['timestamp'])

    plt.plot(open_timestamps, open_altitude, label='Open Area', marker='o', 
             color='#FFB6C1', alpha=0.7)  # Light Pink
    plt.plot(occluded_timestamps, occluded_altitude, label='Obstructed Area', marker='o', 
             color='#FF69B4', alpha=0.7)  # Hot Pink
    plt.title('Stationary Altitude vs. Time Plot')
    plt.xlabel('Timestamp (s)')
    plt.ylabel('Altitude (meters)')
    plt.legend()
    plt.grid()

    # Histograms for position distances
    open_distances = np.sqrt((open_easting_centered - open_centroid_easting)**2 + 
                             (open_northing_centered - open_centroid_northing)**2)
    occluded_distances = np.sqrt((occluded_easting_centered - occluded_centroid_easting)**2 + 
                                  (occluded_northing_centered - occluded_centroid_northing)**2)

    plt.figure()
    plt.hist(open_distances, bins=20, alpha=0.8, color='#FF69B4', edgecolor='black', linewidth=1.5)  # Hot Pink
    plt.title('Histogram of Euclidean Distance from Centroid - Open Area')
    plt.xlabel('Distance (meters)')
    plt.ylabel('Frequency')
    plt.legend(['Open Area - Distance from Centroid'])
    plt.grid()

    plt.figure()
    plt.hist(occluded_distances, bins=20, alpha=0.8, color='#FFB6C1', edgecolor='black', linewidth=1.5)  # Light Pink
    plt.title('Histogram for Euclidean Distance from Centroid - Obstructed Area')
    plt.xlabel('Distance (meters)')
    plt.ylabel('Frequency')
   
