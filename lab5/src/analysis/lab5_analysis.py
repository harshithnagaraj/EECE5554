import matplotlib.pyplot as plt
import math
import pandas as pd
import numpy as np


def lab5_plt(csv_file):
  csv = pd.read_csv(csv_file)
  mag_x = csv['mag_field.magnetic_field.x']
  mag_y = csv['mag_field.magnetic_field.y']
  return mag_x,mag_y

circle_csv = ('/content/circle_lab5_imu.csv')
mag_x,mag_y = lab5_plt(circle_csv)
#Plot before calibration
plt.scatter(mag_x,mag_y, label = 'circle data', marker = '.', color = 'r')
plt.xlabel('Magnetic data North(T)')
plt.ylabel('Magnetic data East(T)')
plt.title('Magnetometer circle data before calibration')
plt.grid(True)
plt.legend()
plt.show()


# Hard iron calibration
mag_x_hi = mag_x - np.mean(mag_x)  # Subtracting mean from all the points
mag_y_hi = mag_y - np.mean(mag_y)

# Calculate mean center and radius
mean_mag_x = np.mean(mag_x_hi)
mean_mag_y = np.mean(mag_y_hi)
r = np.mean(np.sqrt(mag_x_hi**2 + mag_y_hi**2))

# Plot after hard iron calibration
plt.plot(r * np.cos(np.linspace(0, 2 * np.pi, 360)) + mean_mag_x,
         r * np.sin(np.linspace(0, 2 * np.pi, 360)) + mean_mag_y, color='b')
plt.scatter(mag_x_hi, mag_y_hi, label='Hard iron calibrated data', marker='.', color='r')
plt.xlabel('Magnetic data North (T)')
plt.ylabel('Magnetic data East (T)')
plt.title('Magnetometer data after hard iron calibration')
plt.grid(True)
plt.legend()
plt.show()


#soft iron calibration
euclidean_dist = np.sqrt(np.array(mag_x_hi)**2 + np.array(mag_y_hi)**2)
r = np.mean(euclidean_dist)
t = np.linspace(0, 2*np.pi, 100)

print(euclidean_dist)

mean_x = np.mean(mag_x)
mean_y = np.mean(mag_y)

def calc_anb() :
    sort_dist = np.sort(euclidean_dist)
    b = np.mean(sort_dist[0:40000])
    a = np.mean(sort_dist[-40000:])

    return a, b


a, b = calc_anb()
# # print({a})
# # print({b})


mean_x = np.mean(mag_x)
mean_y = np.mean(mag_y)

mag_x_si = []
mag_y_si = []
theta = 0
for i in range(len(mag_x)):
    x_si, y_si = np.matmul([[(a+b)*np.cos(theta)/(2*a), -1*(a+b)*np.sin(theta)/(2*a)],
                            [(a+b)*np.sin(theta)/(2*b), (a+b)*np.cos(theta)/(2*b)]],
                            [mag_x[i]-mean_x, mag_y[i]-mean_y])
    mag_x_si.append(x_si)
    mag_y_si.append(y_si)

plt.figure(figsize=(8, 8))
plt.plot(r*np.cos(t), r*np.sin(t), color="b")
plt.scatter(mag_x_si, mag_y_si, label='Soft Iron calibrated data', marker='.', color='r')
plt.title("Magnetometer data after soft iron calibration")
plt.xlabel("Magnetic data North(T)")
plt.ylabel("Magnetic data East(T)")
plt.grid(True)
plt.axis("equal")
plt.legend()
plt.show()




#gps
import matplotlib.pyplot as plt
import math
import pandas as pd
import numpy as np


def lab5_plt(csv_file):
  csv = pd.read_csv(csv_file)
  utm_x = csv['utm_northing']
  utm_y = csv['utm_easting']
  return utm_x,utm_y

circle_csv = ('/content/circle_lab5_gps.csv')
utm_x,utm_y = lab5_plt(circle_csv)


#before calibration
plt.scatter(utm_x,utm_y, label = 'circle data of gps', marker = '.', color = 'g')
plt.xlabel('gps data Northing')
plt.ylabel('gps data Easting')
plt.title('gps circle data before calibration')
plt.grid(True)
plt.legend()
plt.show()


# Hard iron calibration
utm_x_hi = utm_x - np.mean(utm_x)  # Subtracting mean from all the points
utm_y_hi = utm_y - np.mean(utm_y)

# Calculate mean center and radius
mean_utm_x = np.mean(utm_x_hi)
mean_utm_y = np.mean(utm_y_hi)
r = np.mean(np.sqrt(utm_x_hi**2 + utm_y_hi**2))

# Plot after hard iron calibration
plt.plot(r * np.cos(np.linspace(0, 2 * np.pi, 360)) + mean_utm_x,
         r * np.sin(np.linspace(0, 2 * np.pi, 360)) + mean_utm_y, color='b')
plt.scatter(utm_x_hi, utm_y_hi, label='Hard iron calibrated data', marker='.', color='g')
plt.xlabel('gps data North (T)')
plt.ylabel('gps data East (T)')
plt.title('gps data after hard iron calibration')
plt.grid(True)
plt.legend()
plt.show()


#soft iron calibration
euclidean_dist = np.sqrt(np.array(utm_x_hi)**2 + np.array(utm_y_hi)**2)
r = np.mean(euclidean_dist)
t = np.linspace(0, 2*np.pi, 100)

#print(euclidean_dist)

mean_x = np.mean(utm_x)
mean_y = np.mean(utm_y)

def calc_anb() :
    sort_dist = np.sort(euclidean_dist)
    b = np.mean(sort_dist[0:6000])
    a = np.mean(sort_dist[-6000:])

    return a, b


a, b = calc_anb()
# # print({a})
# # print({b})


mean_x = np.mean(utm_x)
mean_y = np.mean(utm_y)

utm_x_si = []
utm_y_si = []
theta = 0
for i in range(len(utm_x)):
    x_si, y_si = np.matmul([[(a+b)*np.cos(theta)/(2*a), -1*(a+b)*np.sin(theta)/(2*a)],
                            [(a+b)*np.sin(theta)/(2*b), (a+b)*np.cos(theta)/(2*b)]],
                            [utm_x[i]-mean_x, utm_y[i]-mean_y])
    utm_x_si.append(x_si)
    utm_y_si.append(y_si)

plt.figure(figsize=(8, 8))
plt.plot(r*np.cos(t), r*np.sin(t), color="b")
plt.scatter(utm_x_si, utm_y_si, label='Soft Iron calibrated data', marker='.', color='g')
plt.title("utmnetometer data after soft iron calibration")
plt.xlabel("gps data North(T)")
plt.ylabel("gps data East(T)")
plt.grid(True)
plt.axis("equal")
plt.legend()
plt.show()


#imu
from re import X
import matplotlib.pyplot as plt
import math
import pandas as pd
import numpy as np


# def lab5_plt(csv_file):
#   csv = pd.read_csv(csv_file)
#   x = csv['imu.orientation.x']
#   y = csv['imu.orientation.y']
#   T = csv['Time']
#   return x,y,T


drive_csv = ('/content/cardata_lab5_imu.csv')
imu_csv = pd.read_csv(drive_csv)

x = imu_csv['mag_field.magnetic_field.x']
y = imu_csv['mag_field.magnetic_field.y']
T = imu_csv['Time']
raw_yaw = np.arctan2(y,x)
#Plot before calibration
plt.scatter(T,raw_yaw, label = 'yaw v/s time', marker = '.', color = 'y')
plt.xlabel('Time')
plt.ylabel('Yaw')
plt.title('Magnetometer yaw data before calibration')
plt.grid(True)
plt.legend()
plt.show()

#hard iron callibration
x_hi = x - np.mean(x)  # Subtracting mean from all the points
y_hi = y - np.mean(y)
#yaw calculation after hard iron calibration
yaw_hi = np.arctan2(y_hi,x_hi)
yaw_deg_hi = np.degrees(yaw_hi)


plt.scatter(T,yaw_hi, label='Hard iron calibrated data of yaw v/s time', marker='.', color='r')
plt.xlabel('Time')
plt.ylabel('Yaw')
plt.title('Magnetometer data after hard iron calibration')
plt.grid(True)
plt.legend()
plt.show()

#soft iron calibration
euclidean_dist = np.sqrt(np.array(x_hi)**2 + np.array(y_hi)**2)
#r = np.mean(euclidean_dist)
#t = np.linspace(0, 2*np.pi, 100)


def calc_anb() :
    sort_dist = np.sort(euclidean_dist)
    a = np.mean(sort_dist[:int(len(euclidean_dist)*0.5)])
    b = np.mean(sort_dist[-1*int(len(euclidean_dist)*0.5)])

    return a, b


a, b = calc_anb()

mean_x = np.mean(x)
mean_y = np.mean(y)

mag_x_si = []
mag_y_si = []
theta = 0
for i in range(len(x)):
    x_si, y_si = np.matmul([[(a+b)*np.cos(theta)/(2*a), -1*(a+b)*np.sin(theta)/(2*a)],
                            [(a+b)*np.sin(theta)/(2*b), (a+b)*np.cos(theta)/(2*b)]],
                            [x[i]-mean_x, y[i]-mean_y])
    mag_x_si.append(x_si)
    mag_y_si.append(y_si)


yaw_si = np.arctan2(mag_y_si,mag_x_si)

plt.figure(figsize=(8, 8))
#plt.plot(r*np.cos(t), r*np.sin(t), color="b")
plt.scatter(T, yaw_si, label='Soft Iron calibrated data', marker='.', color='r')
plt.title("Magnetometer data after soft iron calibration")
plt.xlabel("Time")
plt.ylabel("Yaw")
plt.grid(True)
plt.axis("equal")
plt.legend()
plt.show()