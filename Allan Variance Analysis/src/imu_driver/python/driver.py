#! /usr/bin/env python3
import rospy
import serial
import utm
import sys
import time
from imu_driver.msg import *
import numpy as np
 
def get_quaternion_from_euler(rolld, pitchd, yawd):
  roll = rolld*(np.pi/180)
  pitch = pitchd*(np.pi/180)
  yaw = yawd*(np.pi/180)
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]

	
if __name__ == '__main__':
	rospy.init_node('gps_data')
	serial_port = rospy.get_param('~port')
	#serial_port = '/dev/pts/4'
	serial_baud = 1152200
	#sampling_rate = 1.0
	
	port = serial.Serial(serial_port, serial_baud, timeout=3)
	rospy.loginfo("Using GPS on port"+ serial_port + " at " +str(serial_baud))
	#sleep_time = 1/sampling_rate - 0.025
	
	msg = imu_msg()
	line = port.readline()
	imu_pub = rospy.Publisher('imu', imu_msg, queue_size=5)
	
	msg.Header.seq = 0
	imu_frame_id = "IMU1_frame"
	msg.Header.frame_id = "IMU1_Frame"

	port.write(b"$VNWRG, 07, 40*XX")

	try:
		while not rospy.is_shutdown():
			line = port.readline().decode().strip('\r$')
			if line == ' ':
				rospy.logwarn("No data recieved.")
			else:
				if line.startswith('VNYMR'):
					rospy.loginfo("______________________")
					rospy.loginfo(line)
					sep = line.split(",")
					rospy.loginfo(sep)
			## All the headers
					msg.IMU.header.frame_id = imu_frame_id
					msg.MagField.header.frame_id = imu_frame_id

					now = rospy.Time.from_sec(time.time())
					msg.Header.stamp = now
					msg.IMU.header.stamp = now
					msg.MagField.header.stamp = now	
					
					msg.Header.seq += 1
					msg.IMU.header.seq += 1
					msg.MagField.header.seq += 1
			## sensor_msgs/Imu
					orient = get_quaternion_from_euler(float(sep[1]), float(sep[2]), float(sep[3]))
					msg.IMU.orientation.x = orient[0]
					msg.IMU.orientation.y = orient[1]
					msg.IMU.orientation.z = orient[2]
					msg.IMU.orientation.w = orient[3]
					msg.IMU.linear_acceleration.x =  float(sep[7])
					msg.IMU.linear_acceleration.y =  float(sep[8])
					msg.IMU.linear_acceleration.z =  float(sep[9])
					msg.IMU.angular_velocity.x =  float(sep[10])
					msg.IMU.angular_velocity.y =  float(sep[11])
					l = sep[12].split("*")
					msg.IMU.angular_velocity.z =  float(l[0])
					rospy.loginfo(msg.IMU)
			## sensor_msgs/Magenetic_Field		
					msg.MagField.magnetic_field.x = float(sep[4])
					msg.MagField.magnetic_field.y = float(sep[5])
					msg.MagField.magnetic_field.z = float(sep[6])
					rospy.loginfo(msg.MagField)
			## (Optional) Printing the entry
					msg.DataLine = line
					imu_pub.publish(msg)
			#rospy.sleep(sleep_time)
			
	except rospy.ROSInterruptException:
		port.close()
	
	except serial.serialutil.SerialException:
		rospy.loginfo("Shut down node...")
