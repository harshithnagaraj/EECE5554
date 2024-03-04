import serial
import rospy
import numpy as np
from vn_driver.msg import Vectornav
from std_msgs.msg import Header

def convert_to_quaternion(roll,pitch,yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return [qx, qy, qz, qw]
    
def driver():
    pub = rospy.Publisher('imu', Vectornav, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    vnav_msg= Vectornav()
    connected_port = "/dev/pts/2"   
    serial_port = rospy.get_param('~port', connected_port)
    serial_baud = rospy.get_param('~baudrate', 115200)    
    port = serial.Serial(serial_port, serial_baud, timeout = 3)
    #read VNYMR string from serial port
    raw_data = port.readline().decode('utf-8').strip()
    
    while not rospy.is_shutdown():
        VNYMR_str_split = raw_data.split(',')
        if VNYMR_str_split[0]== "$VNYMR":
        
     
    # Extract values for accelerometer, gyroscope, orientation, and magnetometer
            yaw = float(VNYMR_str_split[1])
            pitch = float(VNYMR_str_split[2])
            roll = float(VNYMR_str_split[3])
            mag_x = float(VNYMR_str_split[4])
            mag_y = float(VNYMR_str_split[5])
            mag_z = float(VNYMR_str_split[6])
            accel_x = float(VNYMR_str_split[7])
            accel_y = float(VNYMR_str_split[8])
            accel_z = float(VNYMR_str_split[9])
            anr_x = float(VNYMR_str_split[10])
            anr_y = float(VNYMR_str_split[11])
            anr_z = str(VNYMR_str_split[12])
            anr_z = anr_z.split('*')[0]

            print("Roll,Pitch,yaw", roll,pitch,yaw) 
            euler_to_quat = convert_to_quaternion(roll,pitch,yaw)

# publishing data to Vectornav
            vnav_msg.header = Header(frame_id='imu1_frame', stamp=rospy.Time.now())
            vnav_msg.imu.orientation.x = euler_to_quat[0]
            vnav_msg.imu.orientation.y = euler_to_quat[1]
            vnav_msg.imu.orientation.z = euler_to_quat[2]
            vnav_msg.imu.orientation.w = euler_to_quat[3]
            vnav_msg.imu.angular_velocity.x = anr_x
            vnav_msg.imu.angular_velocity.y = anr_y
            vnav_msg.imu.angular_velocity.z = anr_z
            vnav_msg.imu.linear_acceleration.x = accel_x
            vnav_msg.imu.linear_acceleration.y = accel_y
            vnav_msg.imu.linear_acceleration.z = accel_z
            vnav_msg.mag_field.magnetic_field.x = mag_x
            vnav_msg.mag_field.magnetic_field.y = mag_y
            vnav_msg.mag_field.magnetic_field.z = mag_z
            vnav_msg.raw_data = str(raw_data)
            rospy.Rate(40)

            rospy.loginfo(vnav_msg)
            pub.publish(vnav_msg)

if __name__ == '__main__':

    try:
        print("Starting ...")
        driver()
    except rospy.ROSInterruptException:
        pass



