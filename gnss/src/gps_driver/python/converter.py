import rosbag
import pandas as pd

def convert_bag_to_csv(bag_path, csv_path):
    bag = rosbag.Bag(bag_path)
    
    data = {'timestamp': [], 'latitude': [], 'longitude': [], 'altitude': [], 'hdop': [], 'utm_easting': [], 'utm_northing': [],
            'zone': [], 'letter': [], 'fix_quality': [], 'utc': []}

    for _, msg, t in bag.read_messages(topics=['/gps']):
        data['timestamp'].append(t.to_sec())
        data['latitude'].append(msg.latitude)
        data['longitude'].append(msg.longitude)
        data['altitude'].append(msg.altitude)
        data['hdop'].append(msg.hdop)
        data['utm_easting'].append(msg.utm_easting)
        data['utm_northing'].append(msg.utm_northing)
        data['zone'].append(msg.zone)
        data['letter'].append(msg.letter)
        data['fix_quality'].append(msg.fix_quality)
        data['utc'].append(msg.utc)

    bag.close()

    df = pd.DataFrame(data)
    df['timestamp'] = pd.to_datetime(df['timestamp'], unit='s')

    df.to_csv(csv_path, index=False)

# Replace with your .bag and desired .csv file paths
bag_file_path = '/home/harshi/gnss/moving.bag'
csv_file_path = '/home/harshi/gnss/movingA.csv'

# Convert the bag to CSV
convert_bag_to_csv(bag_file_path, csv_file_path)
