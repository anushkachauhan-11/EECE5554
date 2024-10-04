#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
This script reads data from a GNSS receiver over USB serial,
parses the GPGGA string, converts latitude and longitude to UTM,
and publishes the data in a custom ROS message.
"""

import rospy
import serial
import utm
from std_msgs.msg import Header
from gps_driver.msg import Customgps 
import time
from datetime import datetime

def is_gpgga_in_string(input_string):
    """
    Checks if the input string contains the GPGGA identifier.
    """
    return '$GPGGA' in input_string

def deg_mins_to_deg_dec(lat_or_long, is_latitude=True):
    """
    Converts latitude or longitude from DDmm.mm to DD.dddd format.
    
    Args:
        lat_or_long (str): Latitude or longitude in DDmm.mm format.
        is_latitude (bool): True if the input is latitude, False if longitude.

    Returns:
        float: Converted latitude or longitude in decimal degrees.
    """
    if is_latitude:
        deg = int(lat_or_long[:2])  # Latitude degrees (2 digits)
        mins = float(lat_or_long[2:])  # Latitude minutes
    else:
        deg = int(lat_or_long[:3])  # Longitude degrees (3 digits)
        mins = float(lat_or_long[3:])  # Longitude minutes
    return deg + (mins / 60.0)

def lat_long_sign_conversion(lat_or_long, lat_or_long_dir):
    """
    Converts latitude or longitude to signed value based on direction.
    """
    if lat_or_long_dir in ['S', 'W']:  
        return -1 * lat_or_long  
    else:
        return lat_or_long

def convert_to_utm(latitude_signed, longitude_signed):
    """
    Converts latitude and longitude to UTM coordinates.
    """
    utm_vals = utm.from_latlon(latitude_signed, longitude_signed)
    return list(utm_vals)

def utc_to_utc_epoch(utc):
    """
    Converts UTC time from hhmmss.ss format to epoch time based on the current day.
    """
    # Extract hours, minutes, seconds
    hours = int(utc[:2])
    minutes = int(utc[2:4])
    seconds = float(utc[4:])
    
    # Get current time (year, month, day) and replace hour, minute, second
    now = datetime.utcnow()
    utc_time = now.replace(hour=hours, minute=minutes, second=int(seconds), microsecond=int((seconds % 1) * 1e6))
    
    # Return UTC time as epoch time
    return utc_time.timestamp()

def main():
    """
    Main function to read data from serial port, parse GPGGA string,
    convert latitude and longitude to UTM, and publish data as ROS message.
    """
    rospy.init_node('gnss_driver')
    port = rospy.get_param('~port', '/dev/ttyUSB0')  # Adjust to your actual serial port
    ser = serial.Serial(port, 4800, timeout=1)
    pub = rospy.Publisher('/gps', Customgps, queue_size=10)
    rate = rospy.Rate(10)

    # Initialize last known values
    main.last_latitude = None
    main.last_longitude = None

    while not rospy.is_shutdown():
        gpgga_data = ser.readline().decode().strip()
        rospy.loginfo(f"Received raw GPGGA data: {gpgga_data}")  # Debug log
        
        gpggaString = gpgga_data.split(",")
        
        if gpggaString[0] != "$GPGGA":
            continue
        
        # Check for incomplete or missing fields
        if len(gpggaString) < 15 or not gpggaString[2] or not gpggaString[4] or not gpggaString[1]:
            rospy.logwarn("Incomplete or invalid GPGGA data")
            
            # Use last known values if available
            if main.last_latitude is not None and main.last_longitude is not None:
                rospy.loginfo(f"Using last known values - Latitude: {main.last_latitude}, Longitude: {main.last_longitude}")
                LatitudeSigned = main.last_latitude
                LongitudeSigned = main.last_longitude
            else:
                continue  # Skip this iteration if there's no last known data
            
            continue

        # Extract relevant fields from the GPGGA sentence
        latitude = gpggaString[2]
        latitudeDir = gpggaString[3]
        longitude = gpggaString[4]
        longitudeDir = gpggaString[5]
        altitude = float(gpggaString[9]) if gpggaString[9] else 0.0
        hdop = float(gpggaString[8]) if gpggaString[8] else 0.0
        utc = gpggaString[1]
        
        # Convert latitude and longitude from DDmm.mm to decimal degrees
        LatitudeDec = deg_mins_to_deg_dec(latitude, is_latitude=True)
        LongitudeDec = deg_mins_to_deg_dec(longitude, is_latitude=False)
        
        # Convert to signed values based on direction
        LatitudeSigned = lat_long_sign_conversion(LatitudeDec, latitudeDir)
        LongitudeSigned = lat_long_sign_conversion(LongitudeDec, longitudeDir)

        rospy.loginfo(f"Converted Latitude: {LatitudeSigned}, Longitude: {LongitudeSigned}")

        # Save the current valid latitude and longitude for future use
        main.last_latitude = LatitudeSigned
        main.last_longitude = LongitudeSigned

        # Convert to UTM coordinates
        UTMCoords = convert_to_utm(LatitudeSigned, LongitudeSigned)
        rospy.loginfo(f"UTM Coordinates: {UTMCoords}")
        
        # Convert UTC time to epoch time
        try:
            epoch_time = utc_to_utc_epoch(utc)
        except ValueError:
            rospy.logwarn("Invalid UTC time format")
            continue
        
        # Create and publish the ROS message
        msg = Customgps()
        msg.header = Header()
        msg.header.frame_id = 'GPS1_Frame'
        msg.header.stamp.secs = int(epoch_time)
        msg.header.stamp.nsecs = int((epoch_time - int(epoch_time)) * 1e9)
        msg.latitude = LatitudeSigned
        msg.longitude = LongitudeSigned
        msg.altitude = altitude
        msg.utm_easting = UTMCoords[0]
        msg.utm_northing = UTMCoords[1]
        msg.zone = UTMCoords[2]
        msg.letter = UTMCoords[3]
        msg.hdop = hdop
        msg.gpgga_read = gpgga_data
        
        rospy.loginfo(msg)
        pub.publish(msg)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
