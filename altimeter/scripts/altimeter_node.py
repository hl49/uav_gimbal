#!/usr/bin/python

import rospy
from altimeter.BMP180_driver import BMP180
from altimeter.msg import Altimeter

class AltimeterBMP180():

	def __init__(self):
		self.pub_alt = rospy.Publisher("~BMP180",
			Altimeter, queue_size = 10)
		self.alt_sensor = BMP180()

		self.temp = 0.0	# In celcius degrees
		self.alt = 0.0 # In meters


		self.alt_dat_seq_counter = 0

	def read_data(self):

		self.temp = self.alt_sensor.temperature()
		self.alt = self.alt_sensor.altitude()
	

	def publish_data(self):

		alt_data = Altimeter()

		alt_data.header.seq = self.alt_dat_seq_counter
		alt_data.header.stamp = rospy.Time.now()
		alt_data.header.frame_id = ''

		alt_data.altitude = self.alt
		alt_data.temperature = self.temp

		self.pub_alt.publish(alt_data)

if __name__ == '__main__':
	
	# Init node
	rospy.init_node("altimeter")

	altimeter = AltimeterBMP180()

	rate = rospy.Rate(2)

	while not rospy.is_shutdown():
		altimeter.read_data()
		altimeter.publish_data()
		rate.sleep()
