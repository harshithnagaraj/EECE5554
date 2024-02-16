#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import rosbag
import matplotlib.pyplot as plt
import numpy as np
from gps_driver.msg import Customrtk
from sklearn.linear_model import LinearRegression

class GPSAnalyzer:
    def __init__(self):
        self.easting_data = []
        self.northing_data = []
        self.altitude_data = []
        self.utc_data = []
        self.is_stationary = True  # Assuming the initial state is stationary

    def callback(self, msg):
        self.easting_data.append(msg.UTM_easting)
        self.northing_data.append(msg.UTM_northing)
        self.altitude_data.append(msg.Altitude)
        self.utc_data.append(msg.utc)

    def analyze_data(self):
        if not self.easting_data or not self.northing_data or not self.altitude_data:
            rospy.logwarn("No data found for analysis.")
            return

        easting_centroid = np.mean(self.easting_data)
        northing_centroid = np.mean(self.northing_data)

        # Subtract centroid from each data point
        centered_easting = np.array(self.easting_data) - easting_centroid
        centered_northing = np.array(self.northing_data) - northing_centroid

        # Create scatterplot for stationary northing vs. easting
        plt.figure(figsize=(10, 6))
        plt.scatter(centered_easting, centered_northing, label="Stationary Data", marker='o')
        plt.scatter(0, 0, color='red', marker='x', label="Centroid")
        plt.xlabel("Easting (m)")
        plt.ylabel("Northing (m)")
        plt.title("Stationary Northing vs. Easting Scatterplot")
        plt.legend()
        plt.grid(True)
        plt.show()

        # Create stationary altitude vs. time plot
        plt.figure(figsize=(10, 6))
        plt.plot(self.utc_data, self.altitude_data, label="Stationary Data", marker='o')
        plt.xlabel("Time (s)")
        plt.ylabel("Altitude (m)")
        plt.title("Stationary Altitude vs. Time Plot")
        plt.legend()
        plt.grid(True)
        plt.show()

def read_bag_data(bag_file):
    gps_analyzer = GPSAnalyzer()
    bag = rosbag.Bag(bag_file)

    for topic, msg, t in bag.read_messages(topics=['/gps']):
        gps_analyzer.callback(msg)

    bag.close()

    return gps_analyzer

if __name__ == '__main__':
    try:
        rospy.init_node('gps_analyzer', anonymous=True)

        # Replace 'your_bag_file.bag' with the actual path to your bag file
        bag_file = '/home/harshi/gnss/src/data/openRTK_H.bag'

        gps_analyzer = read_bag_data(bag_file)
        gps_analyzer.analyze_data()
        
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
