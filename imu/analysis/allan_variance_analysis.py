import rosbag
import numpy as np
import allantools
import matplotlib.pyplot as plt

def sanitize_numeric_string(value):
    if value.count('.') > 1:
        value = value.replace('.', '', value.count('.') - 1)
    return value

def parse_vnymr_string(vnymr_str):
    vnymr_str = ''.join(filter(lambda x: x in set('0123456789.,+-*'), vnymr_str))
    parts = vnymr_str.split(',')
    
    if len(parts) >= 13:
        roll = float(sanitize_numeric_string(parts[1]))
        pitch = float(sanitize_numeric_string(parts[2]))
        yaw = float(sanitize_numeric_string(parts[3]))
        gyro_x = float(sanitize_numeric_string(parts[4]))
        gyro_y = float(sanitize_numeric_string(parts[5]))
        gyro_z = float(sanitize_numeric_string(parts[6]))
        accel_x = float(sanitize_numeric_string(parts[10]))
        accel_y = float(sanitize_numeric_string(parts[11]))
        accel_z = float(sanitize_numeric_string(parts[12].split('*')[0]))  # Remove checksum part from the last value
        return roll, pitch, yaw, gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z
    else:
        return None

def load_bag_data(bag_path):
    bag = rosbag.Bag(bag_path)
    data = {'Time': [], 'gyro_x': [], 'gyro_y': [], 'gyro_z': [],
            'accel_x': [], 'accel_y': [], 'accel_z': []}

    for topic, msg, t in bag.read_messages(topics=['/vectornav']):
        parsed_data = parse_vnymr_string(msg.data)
        if parsed_data:
            roll, pitch, yaw, gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z = parsed_data
            data['Time'].append(t.to_sec())
            data['gyro_x'].append(gyro_x)
            data['gyro_y'].append(gyro_y)
            data['gyro_z'].append(gyro_z)
            data['accel_x'].append(accel_x)
            data['accel_y'].append(accel_y)
            data['accel_z'].append(accel_z)

    bag.close()
    print("Sample data:", {k: v[:5] for k, v in data.items()})  # Print first 5 entries for verification
    return data

def plot_allan_variance(data, sampling_interval, sensor_type):
    plt.figure() 
    for axis in ['gyro_x', 'gyro_y', 'gyro_z'] if sensor_type == "Gyroscope" else ['accel_x', 'accel_y', 'accel_z']:
        tau, adev, _, _ = allantools.oadev(data[axis], rate=1/sampling_interval, data_type="freq")
        plt.loglog(tau, adev, label=f'{axis}')
    plt.xlabel('Time (s)')
    plt.ylabel('Allan Deviation')
    plt.title(f'Allan Variance - {sensor_type}')
    plt.legend()
    plt.grid()
    plt.show()

bag_path = '/home/chauhan-anu/catkin_ws/src/lab3/data/LocationC.bag'
imu_data = load_bag_data(bag_path)

sampling_interval = np.mean(np.diff(imu_data['Time']))

plot_allan_variance(imu_data, sampling_interval, "Gyroscope")

plot_allan_variance(imu_data, sampling_interval, "Accelerometer")
