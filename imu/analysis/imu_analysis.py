import rosbag
import pandas as pd
import matplotlib.pyplot as plt
import math

# Load ROS bag file and extract data
def load_bag_data(bag_path):
    bag = rosbag.Bag(bag_path)
    time = []
    gyro_x, gyro_y, gyro_z = [], [], []
    accel_x, accel_y, accel_z = [], [], []
    roll, pitch, yaw = [], [], []

    for topic, msg, t in bag.read_messages(topics=['/imu']):
        time.append(t.to_sec())
        gyro_x.append(msg.imu.angular_velocity.x)
        gyro_y.append(msg.imu.angular_velocity.y)
        gyro_z.append(msg.imu.angular_velocity.z)
        accel_x.append(msg.imu.linear_acceleration.x)
        accel_y.append(msg.imu.linear_acceleration.y)
        accel_z.append(msg.imu.linear_acceleration.z)

        # Convert quaternions to Euler angles
        qx, qy, qz, qw = msg.imu.orientation.x, msg.imu.orientation.y, msg.imu.orientation.z, msg.imu.orientation.w
        roll_val = math.atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx ** 2 + qy ** 2))
        pitch_val = math.asin(2.0 * (qw * qy - qz * qx))
        yaw_val = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy ** 2 + qz ** 2))
        
        roll.append(math.degrees(roll_val))
        pitch.append(math.degrees(pitch_val))
        yaw.append(math.degrees(yaw_val))

    bag.close()

    # Normalize time to start at 0
    start_time = time[0]
    time = [t - start_time for t in time]

    return pd.DataFrame({
        'Time': time,
        'gyro_x': gyro_x, 'gyro_y': gyro_y, 'gyro_z': gyro_z,
        'accel_x': accel_x, 'accel_y': accel_y, 'accel_z': accel_z,
        'roll': roll, 'pitch': pitch, 'yaw': yaw
    })

# Updated plotting function to handle numpy array conversion
def plot_time_series(data, column_names, ylabel, title, fig_num):
    plt.figure(fig_num, figsize=(10, 6))
    for i, column in enumerate(column_names, 1):
        plt.subplot(3, 1, i)
        plt.plot(data['Time'].to_numpy(), data[column].to_numpy(), label=column, marker='o' if i == 1 else ('x' if i == 2 else '^'))
        plt.ylabel(ylabel)
        plt.legend()
    plt.xlabel('Time (s)')
    plt.suptitle(title)
    plt.tight_layout(rect=[0, 0, 1, 0.96])

def plot_histograms(data, column_names, ylabel, title, fig_num):
    plt.figure(fig_num, figsize=(10, 6))
    for i, column in enumerate(column_names, 1):
        plt.subplot(3, 1, i)
        plt.hist(data[column], bins=30, label=column)
        plt.ylabel(ylabel)
        plt.legend()
    plt.xlabel('Degrees')
    plt.suptitle(title)
    plt.tight_layout(rect=[0, 0, 1, 0.96])

# Load data from specified ROS bag file path
data = load_bag_data('/home/chauhan-anu/catkin_ws/src/lab3/data/data.bag')

# Generate required plots
plot_time_series(data, ['gyro_x', 'gyro_y', 'gyro_z'], 'Degrees/s', 'Fig 0: Gyro Rotational Rate', 0)
plot_time_series(data, ['accel_x', 'accel_y', 'accel_z'], 'm/s²', 'Fig 1: Accelerometer', 1)
plot_time_series(data, ['roll', 'pitch', 'yaw'], 'Degrees', 'Fig 2: VN Orientation (Euler Angles)', 2)
plot_histograms(data, ['roll', 'pitch', 'yaw'], 'Frequency', 'Fig 3: Histograms of Rotation', 3)

plt.show()
