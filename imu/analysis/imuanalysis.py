import rosbag
import bagpy
import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt


def get_data(file):
    easting = []
    northing = []
    altitude = []
    time = []
    cent_easting = []
    cent_northing = []

    with rosbag.Bag(file) as bag:
        northing_tot = 0.0
        easting_tot = 0.0
        pts = 0
        northing_first = 0.0
        easting_first = 0.0


        for topic, msg, t in bag.read_messages(topics="/gps"):
            east_msg = msg.utm_easting
            north_msg = msg.utm_northing
            alt_msg = msg.altitude
            time_msg = msg.header.stamp.sec

            if pts == 0:
                northing_first = north_msg
                easting_first = east_msg
