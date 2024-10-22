import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import UnivariateSpline

def load_data_from_csv(csv_file_path):
    """Load GPS data (easting, northing, altitude, and timestamp) from a CSV file."""
    df = pd.read_csv(csv_file_path)
    
    # Check if the data is empty
    if df.empty:
        print(f"Warning: No data found in {csv_file_path}")
    return df

def spline_fit_and_plot(easting, northing, location_type):
    """Generate spline fit and plot the walking data with spline smoothing."""
    if len(easting) < 2 or len(northing) < 2:
        print(f"Warning: Not enough data points for {location_type}. Skipping spline fit.")
        return

    spline_easting = UnivariateSpline(range(len(easting)), easting)
    spline_northing = UnivariateSpline(range(len(northing)), northing)

    # Generate smooth line for comparison
    smooth_easting = spline_easting(np.linspace(0, len(easting) - 1, 500))
    smooth_northing = spline_northing(np.linspace(0, len(northing) - 1, 500))

    plt.figure()
    plt.plot(easting, northing, 'o', label=f'{location_type} Walking Data', markersize=2, color='blue')
    plt.plot(smooth_easting, smooth_northing, color='red', label='Spline Fit')
    plt.xlabel('Easting (meters)')
    plt.ylabel('Northing (meters)')
    plt.title(f'Moving {location_type} Data with Spline Fit')
    plt.legend()
    plt.grid()

def analyze_moving_data(df, location_type):
    """Analyze and plot the moving data including spline fit and altitude vs. time."""
    if df.empty:
        print(f"Error: No data available for {location_type}.")
        return

    easting = df['easting'].to_numpy()
    northing = df['northing'].to_numpy()
    altitude = df['altitude'].to_numpy()
    timestamps = df['timestamp'].values

    # Calculate time in seconds
    start_time = timestamps[0]
    time_seconds = timestamps - start_time

    # Check if there is enough data to plot
    if len(easting) > 1 and len(northing) > 1:
        # Moving data scatterplot with spline fit
        spline_fit_and_plot(easting, northing, location_type)
    else:
        print(f"Warning: Not enough valid GPS data for {location_type}. Skipping plot.")

    # Moving data altitude plot
    plt.figure()
    plt.plot(time_seconds, altitude, label=f'{location_type} Walking Data', marker='x', color='red') 
    plt.title(f'Moving {location_type} Data Altitude vs. Time')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Altitude (meters)')
    plt.legend()
    plt.grid()

    print(f"Analysis for {location_type} completed")

# Main section to load data from two CSV files and analyze them
open_csv_path = '/home/chauhan-anu/catkin_ws/src/lab2/data/square_open_area.csv'
occluded_csv_path = '/home/chauhan-anu/catkin_ws/src/lab2/data/square_partially_occluded.csv'

# Load data from CSV files
open_data_df = load_data_from_csv(open_csv_path)
occluded_data_df = load_data_from_csv(occluded_csv_path)

# Analyze both datasets
analyze_moving_data(open_data_df, "Open")
analyze_moving_data(occluded_data_df, "Occluded")

# Show all plots together
plt.show()
