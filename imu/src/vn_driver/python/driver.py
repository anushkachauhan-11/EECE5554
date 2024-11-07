#!/usr/bin/env python3

import rospy
import serial
import re
from vn_driver.msg import Vectornav
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Header
import math

def convert_to_quaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(roll * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy

    return qx, qy, qz, qw

def parse_data(line):
    # Clean the line by keeping only digits, decimal points, commas, and hyphens
    clean_line = re.sub(r'[^\d\.,-]', '', line)  # Removes unexpected characters
    data = clean_line.split(',')

    # Check if the data line has the required number of values
    if len(data) < 10:  # Adjusted based on the number of expected values
        rospy.logerr(f"Incomplete data line: {line}")
        return None

    try:
        # Attempt to parse values safely, ensuring only expected floats are present
        accel_x, accel_y, accel_z = map(float, data[1:4])
        gyro_x, gyro_y, gyro_z = map(float, data[4:7])
        roll, pitch, yaw = map(float, data[7:10])
        return accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, roll, pitch, yaw
    except (ValueError, IndexError) as e:
        rospy.logerr(f"Error parsing IMU data: {e} - Raw data: {line}")
        return None  # Return None if parsing fails

def publish_imu_data():
    rospy.init_node('imu_driver_node', anonymous=True)
    imu_pub = rospy.Publisher('/imu', Vectornav, queue_size=10)

    port = rospy.get_param('~port', '/dev/ttyUSB0')

    try:
        ser = serial.Serial(port, 115200, timeout=1)
        set_rate_command = b'VNWRG,75,40\r\n'
        ser.write(set_rate_command)
        rospy.loginfo("Configured IMU output rate to 40 Hz.")
    except serial.SerialException as e:
        rospy.logerr("Error opening serial port: %s", e)
        return

    rate = rospy.Rate(40)

    while not rospy.is_shutdown():
        try:
            line = ser.readline().decode('utf-8').strip()
            rospy.logdebug(f"Raw IMU Data: {line}")

            if line.startswith('$VNYMR'):
                parsed_values = parse_data(line)
                if parsed_values:
                    accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, roll, pitch, yaw = parsed_values
                    qx, qy, qz, qw = convert_to_quaternion(roll, pitch, yaw)

                    # Create the custom Vectornav message
                    vectornav_msg = Vectornav()
                    vectornav_msg.header = Header()
                    vectornav_msg.header.stamp = rospy.Time.now()
                    vectornav_msg.header.frame_id = "imu1_frame"

                    # Fill in the IMU data
                    vectornav_msg.imu.orientation.x = qx
                    vectornav_msg.imu.orientation.y = qy
                    vectornav_msg.imu.orientation.z = qz
                    vectornav_msg.imu.orientation.w = qw
                    vectornav_msg.imu.angular_velocity.x = gyro_x
                    vectornav_msg.imu.angular_velocity.y = gyro_y
                    vectornav_msg.imu.angular_velocity.z = gyro_z
                    vectornav_msg.imu.linear_acceleration.x = accel_x
                    vectornav_msg.imu.linear_acceleration.y = accel_y
                    vectornav_msg.imu.linear_acceleration.z = accel_z

                    # Fill in magnetic field data (placeholder values)
                    vectornav_msg.mag_field.magnetic_field.x = 0.0
                    vectornav_msg.mag_field.magnetic_field.y = 0.0
                    vectornav_msg.mag_field.magnetic_field.z = 0.0
                    vectornav_msg.raw_imu_data = line

                    imu_pub.publish(vectornav_msg)
                    rospy.loginfo(f"Published IMU Data: Roll={roll}, Pitch={pitch}, Yaw={yaw}")

                rate.sleep()

        except serial.SerialException as e:
            rospy.logerr("Error reading from IMU: %s", e)
        except Exception as e:
            rospy.logerr("Unexpected error: %s", e)

if __name__ == '__main__':
    try:
        publish_imu_data()
    except rospy.ROSInterruptException:
        pass
