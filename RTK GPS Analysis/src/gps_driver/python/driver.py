import rospy
import serial
import utm
import sys
from gps_driver.msg import *

def lat_normal(data):
	min = float(data[2:])
	deg = float(data[0:2])
	rospy.loginfo(deg)
	rospy.loginfo(min)
	latlon = deg + (min/60)
	return latlon

def lon_normal(data):
	min = float(data[3:])
	deg = float(data[0:3])
	rospy.loginfo(deg)
	rospy.loginfo(min)
	latlon = deg + (min/60)
	return latlon

def to_sec(data):
	hrs = float(data[0:2])
	min = float(data[2:4])
	sec = float(data[4:])
	secs = hrs*3600 + min*60 + sec
	rospy.loginfo(secs)
	return secs	
	
	
if __name__ == '__main__':
	rospy.init_node('gps_data')
	serial_port = rospy.get_param('~port')
	serial_baud = 4800
	sampling_rate = 1.0
	
	port = serial.Serial(serial_port, serial_baud, timeout=3)
	rospy.loginfo("Using GPS on port"+ serial_port + " at " +str(serial_baud))
	#sleep_time = 1/sampling_rate - 0.025
	
	msg = gps_msg()
	line = port.readline()
	gps_pub = rospy.Publisher('gps', gps_msg, queue_size=5)
	
	msg.Header.seq = 0
	msg.Header.frame_id = "GPS1_Frame"

	try:
		while not rospy.is_shutdown():
			line = port.readline().decode().strip('\r$')
			if line == ' ':
				rospy.logwarn("No data recieved.")
			else:
				if line.startswith('GNGGA'):
					rospy.loginfo(line)
					rospy.loginfo("______________________")
					sep = line.split(",")
					msg.Header.stamp = rospy.Time.from_sec(to_sec(sep[1]))
					
					longit = float(sep[4])
					
			# If the longitude is west, multiply by -1
					if (sep[5]=="W"):
						longit=longit*-1
					
					
					msg.Altitude = float(sep[9])
					ut = utm.from_latlon(la1(sep[2]), lon_normal(str(longit)))		
					msg.UTM_northing = ut[0]
					msg.UTM_easting = ut[1]
					msg.Zone = ut[2]
					msg.Quality = int(sep[6])
					msg.Letter = ut[3]
					msg.Longitude = lon_normal(str(longit))
					msg.Latitude = lat_normal(sep[2])
					msg.Header.seq += 1
					rospy.loginfo(msg)
					gps_pub.publish(msg)
			#rospy.sleep(sleep_time)
	except rospy.ROSInterruptException:
		port.close()
	
	except serial.serialutil.SerialException:
		rospy.loginfo("Shut down node...")
