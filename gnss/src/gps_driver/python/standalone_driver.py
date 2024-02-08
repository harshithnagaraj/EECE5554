#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from datetime import datetime, timedelta
import rospy
import serial
import utm
import sys
from gps_driver.msg import Customgps
from gps_driver.msg import *
def float_to_epoch_time(float_time):
    # Convert the float to hours, minutes, and seconds
    time_str = str(int(float_time)).zfill(6)  # Pad with leading zeros if necessary
    hh = int(time_str[:2])
    mm = int(time_str[2:4])
    ss = int(time_str[4:])

    # Get the current date and time
    current_datetime = datetime.now()

    # Create a new datetime object with the time from the float and the current date
    new_datetime = datetime(current_datetime.year, current_datetime.month, current_datetime.day, hh, mm, ss)

    # Calculate the difference in seconds between the new datetime and the beginning of the current day
    epoch_time = int((new_datetime - datetime(current_datetime.year, current_datetime.month, current_datetime.day)).total_seconds())

    return epoch_time
   
def driver():
    
    rospy.init_node('talker', anonymous=True)
    pub = rospy.Publisher('gps', Customgps, queue_size=10)
    
    # args = rospy.myargv(argv = sys.argv)
    # if len(args) != 2:
        # print("error")
        # sys.exit(1)

    # connected_port = args[1]
    # connected_port = "/dev/pts/3"
    connected_port = "/dev/ttyUSB0"


    
    serial_port = rospy.get_param('~port', connected_port)
    serial_baud = rospy.get_param('~baudrate', 4800)
    
    port = serial.Serial(serial_port, serial_baud, timeout = 3)
    
    rospy.loginfo("Publishing parsed GPS data")

    while not rospy.is_shutdown():  
        
        data = str(port.readline())
        data = data.lstrip("b'\\r$")
        data = data.lstrip("b'$")
        data = data.rstrip("\\n'")
        if data == '':
            rospy.logwarn("No GPS data found")
        else:
            recievedData = data.split(',')
            if recievedData[0] == "GPGGA":
                utc = float(recievedData[1])
                rospy.loginfo(recievedData[1])
                # utc_hours = utc // 10000
                # utc_minutes = (utc-(utc_hours*10000))//100
                # utc_seconds = (utc - (utc_hours*10000) - (utc_minutes*100))
                # total_utc_secs = (utc_hours*3600 + utc_minutes*60 + utc_seconds)
                # total_utc_nsecs = int((total_utc_secs * (10**7)))

                latitude = float(recievedData[2])
                latitude_direction = recievedData[3]
                latitude_correction = 1
                if latitude_direction == "S":
                    latitude_correction = -1

                latitude_degree = int(latitude/100)
                latitude_minutes = float(latitude) - (latitude_degree * 100)
                latitude_converted = latitude_correction * \
                    float(latitude_degree + latitude_minutes/60)

                longitude = float(recievedData[4])
                longitude_direction = recievedData[5]
                longitude_correction = 1
                if longitude_direction == "W":
                    longitude_correction = -1

                longitude_degree = int(longitude / 100)
                longitude_minutes = float(longitude) - (longitude_degree * 100)
                longitude_converted = longitude_correction * \
                    float(longitude_degree + longitude_minutes/60)

                hdop = float(recievedData[8])
                altitude = float(recievedData[9])

                utc_data = float(recievedData[1])
                # rospy.loginfo(utc_data)
                # utc_data = utc_data[:2]+ " Hours " + utc_data[2:4] + " Minutes " + utc_data[4:] + " Seconds"
                utm_lat_long = utm.from_latlon(
                    latitude_converted, longitude_converted)
            
                    
                msg = Customgps()
                # utc_datetime = da/tetime.strptime(utc_data, '%H%M%S')
                # epoch_time = (utc_datetime - datetime(1970, 1, 1)).total_seconds()

                msg.Header.stamp.secs = float_to_epoch_time(utc)
                # msg.Header.stamp.nsecs = int(str(total_utc_nsecs)[:5])
                #print(f'UTM_East, UTM_north, Zone, Letter: {utm_lat_long}')
                msg.Header.frame_id = "GPS1_Frame"
                msg.Latitude = latitude_converted
                msg.Longitude = longitude_converted
                msg.HDOP = hdop
                msg.Altitude = altitude
                msg.UTM_easting = utm_lat_long[0]
                msg.UTM_northing = utm_lat_long[1]
                msg.UTC = utc_data
                msg.zone = utm_lat_long[2]
                msg.Letter = utm_lat_long[3]
                rospy.loginfo(msg)
                pub.publish(msg)
   

if __name__ == '__main__':

    try:
        driver()
    except rospy.ROSInterruptException:
        pass









# #!/usr/bin/env python3

# import rospy
# from std_msgs.msg import Header,String
# from gps_driver.msg import Customgps
# #from gps_driver.msg import Customgps
# from utm import from_latlon
# from datetime import datetime

# def parse_gpgga(gpgga_str):
#     """Parses a GPGGA string and extracts relevant data."""
#     values = gpgga_str.split(',')
#     if len(values) >= 15:
#         try:
#             latitude = float(values[2])
#             longitude = float(values[4])
#             utc = values[1]  # Format: hhmmss.ss
#             hdop = float(values[8])
#             return latitude, longitude, utc, hdop
#         except ValueError:
#             rospy.logwarn("Invalid GPGGA string: %s", gpgga_str)
#     return None, None, None, None

# def gps_callback(data):
#     utc, latitude, longitude, hdop = parse_gpgga(data.data)

#     # Convert UTC to epoch time
#     utc_datetime = datetime.strptime(utc, '%H%M%S')
#     epoch_time = (utc_datetime - datetime(1970, 1, 1)).total_seconds()

#     # Convert latitude and longitude to UTM
#     utm_easting, utm_northing, zone, letter = from_latlon(latitude, longitude)

#     # Create Customgps message
#     custom_gps_msg = Customgps()
#     #custom_gps_msg.Header = Header()
#     custom_gps_msg.Header.frame_id = 'GPS1_Frame'
#     custom_gps_msg.Header.stamp.sec = int(epoch_time)
#     custom_gps_msg.Header.stamp.nsec = 0  # You may need to adjust this depending on your GPS unit

#     custom_gps_msg.Latitude = latitude
#     custom_gps_msg.Longitude = longitude
#     custom_gps_msg.Altitude = 0.0  # Replace with actual altitude value
#     custom_gps_msg.UTM_easting = utm_easting
#     custom_gps_msg.UTM_northing = utm_northing
#     custom_gps_msg.zone = zone
#     custom_gps_msg.Letter = letter
#     custom_gps_msg.HDOP = hdop
#     #custom_gps_msg.gpgga_read = data.data
#     rospy.loginfo(custom_gps_msg)

#     # Publish Customgps message to topic '/gps'
#     gps_publisher.publish(custom_gps_msg)

# if __name__ == '__main__':
#     rospy.init_node('gps_converter_node', anonymous=True)
#     gps_publisher = rospy.Publisher('/gps', Customgps, queue_size=10)
#     rospy.Subscriber('gps_driver', String, gps_callback)  # Replace '/your_gps_topic' with the actual GPS topic

#     rospy.spin()

