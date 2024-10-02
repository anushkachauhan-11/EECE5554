#!/usr/bin/env python
import rospy
from gps_driver.msg import gps_msg  # Ensure this matches your message definition

def talker():
    pub = rospy.Publisher('gps', gps_msg, queue_size=10)
    rospy.init_node('gps_driver', anonymous=True)
    rate = rospy.Rate(10)  # 10 Hz
    
    while not rospy.is_shutdown():
        msg = gps_msg()
        # Set your message fields here, e.g.:
        msg.Header.stamp = rospy.Time.now()
        msg.Latitude = 37.7749  # Replace with actual GPS data
        msg.Longitude = -122.4194  # Replace with actual GPS data
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
