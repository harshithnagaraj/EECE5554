#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import rosbag
import matplotlib.pyplot as plt
import numpy as np
from gps_driver.msg import Customrtk
from sklearn.linear_model import LinearRegression

class GPSAnalyzer:
    def __init__(self, bag_index):
        self.bag_index = bag_index
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
            rospy.logwarn(f"No data found for analysis in bag {self.bag_index}.")
            return

        easting_centroid = np.mean(self.easting_data)
        northing_centroid = np.mean(self.northing_data)

        # Subtract centroid from each data point
        centered_easting = np.array(self.easting_data) - easting_centroid
        centered_northing = np.array(self.northing_data) - northing_centroid

        # Create scatterplot for stationary northing vs. easting with different colors
        plt.figure(figsize=(10, 6))
        plt.scatter(centered_easting, centered_northing, label=f"Bag {self.bag_index} Data", marker='o')
        plt.scatter(0, 0, color='red', marker='x', label="Centroid")
        plt.xlabel("Easting (m)")
        plt.ylabel("Northing (m)")
        plt.title(f"Stationary Northing vs. Easting Scatterplot (Bag {self.bag_index})")
        plt.legend()
        plt.grid(True)
        plt.show()

        # Create stationary altitude vs. time plot with different colors
        plt.figure(figsize=(10, 6))
        plt.plot(self.utc_data, self.altitude_data, label=f"Bag {self.bag_index} Data", marker='o')
        plt.xlabel("Time (s)")
        plt.ylabel("Altitude (m)")
        plt.title(f"Stationary Altitude vs. Time Plot (Bag {self.bag_index})")
        plt.legend()
        plt.grid(True)
        plt.show()

def read_bag_data(bag_files):
    gps_analyzers = []
    
    for i, bag_file in enumerate(bag_files):
        gps_analyzer = GPSAnalyzer(i)
        bag = rosbag.Bag(bag_file)

        for topic, msg, t in bag.read_messages(topics=['/gps']):
            gps_analyzer.callback(msg)

        bag.close()
        gps_analyzers.append(gps_analyzer)

    return gps_analyzers

if __name__ == '__main__':
    try:
        rospy.init_node('gps_analyzer', anonymous=True)

        # Replace with the actual paths to your bag files
        bag_files = ['/home/harshi/gnss/src/data/openRTK_H.bag', '/home/harshi/gnss/src/data/occludedRTK_H.bag']

        gps_analyzers = read_bag_data(bag_files)
        
        for gps_analyzer in gps_analyzers:
            gps_analyzer.analyze_data()

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
