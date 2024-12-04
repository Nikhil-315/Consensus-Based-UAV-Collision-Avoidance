#!/usr/bin/env python3

import rospy
import os
from sensor_msgs.msg import NavSatFix
import utm

# Initialize global variables to store positions
drone1_position = None
drone2_position = None
txt_filename = '/home/usr/cas_ws/src/collision_avoidance/src/trajectories.txt'

def drone1_callback(data):
    global drone1_position
    x, y, _, _ = utm.from_latlon(data.latitude, data.longitude)

    # Extract position from NavSatFix message
    drone1_position = (x, y, data.altitude)

def drone2_callback(data):
    global drone2_position
    x, y, _, _ = utm.from_latlon(data.latitude, data.longitude)

    # Extract position from NavSatFix message
    drone2_position = (x, y, data.altitude)

def main():
    # Delete the TXT file if it already exists
    if os.path.exists(txt_filename):
        os.remove(txt_filename)

    rospy.init_node('gcs', anonymous=True)

    # Subscribe to the drone positions 
    drone1_sub = rospy.Subscriber('/drone1/mavros/global_position/global', NavSatFix, drone1_callback)
    drone2_sub = rospy.Subscriber('/drone2/mavros/global_position/global', NavSatFix, drone2_callback)

    # Open a TXT file for writing
    with open(txt_filename, mode='w') as file:
        # Write a header
        file.write('x1 y1 alt1 x2 y2 alt2\n')

        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            if drone1_position and drone2_position:
                
                x1, y1, alt1 = drone1_position
                x2, y2, alt2 = drone2_position

                # Write the data as a space-separated line
                file.write(f"{x1} {y1} {alt1} {x2} {y2} {alt2}\n")
                file.flush()  # Ensure the data is written to the file immediately

            rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
