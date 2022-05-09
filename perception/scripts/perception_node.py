#!/usr/bin/python
# -*- coding: utf-8 -*-

from __future__ import division
import numpy as np
import cv2 as cv
import sympy as sp

import rospy
from perception.msg import RPYAxes
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class Perception():

	def __init__(self):

		self.bridge = CvBridge()

		# Define publisher
		self.pub_refs = rospy.Publisher("/perception_rpy",
			RPYAxes, queue_size = 1)
	
		self.sub = rospy.Subscriber("/segnet/color_mask",
			Image, self.process_img_callback)

		self.image_counter = 0
		self.theta_inc = 0.0
		self.last_angle = 0.0
		self.last_b = 0.0
		self.b_inc = 0.0
		self.d_ref = 0.0
		self.reference_angle = 0.0
		self.focal_length_pixel = 0.0
		self.focal_length_mm = 0.0
		self.last_last_angle = 0.0
		self.last_last_b = 0.0
		self.reference_b = 0.0
		self.img_height = 0.0
		self.img_width = 0.0
		self.area_ref = 0.0
		self.vec_ref = 0.0
		self.roll_angle_compensate = 0.0
		self.pitch_angle_compensate = 0.0

	def fit(self, x, y): 
		"""Curve Fitting Straight line. Return the slope of the line a and the y-intercept b"""
		xbar = sum(x) / len(x)
		ybar = sum(y) / len(y)
		n = len(x) # or len(y)
		numer = sum([xi * yi for xi,yi in zip(x, y)]) - n * xbar * ybar
		denum = sum([xi ** 2 for xi in x]) - n * xbar ** 2
		a = numer / denum
		b = ybar - a * xbar
		# print("xbar = ", xbar)
		# print("ybar = ", ybar)
		# print("n = ", n)
		# print("numer = ", numer)
		# print("denum = ", denum)
		return a, b


	def distance(self, x, y): 
		"""Straight line distance between two points"""
		x1 = x[0]
		x2 = x[-1]
		y1 = y[0]
		y2 = y[-1]
		d = np.sqrt(((x2-x1) ** 2) + ((y2-y1) ** 2))
		return d


	def boundary_removal(self, img): 
		"""Remove edges from the boundary of the image frame."""
		for i in range (1,13):
			img[:,-i] = img[:,-15]
		for i in range (0,12):
			img[:,i] = img[:,15]
		for i in range (0,12):
			img[i,:] = img[15,:]
		return img


	def estimate_plane(self, a, b, c):
		"""Estimate the parameters of the plane passing by three points.
		Return:center(float): The center point of the three input points.
		normal(float): The normal to the plane."""
		center = (a + b + c) / 3
		normal = np.cross(b - a, c - a)
		assert(np.isclose(np.dot(b - a, normal), np.dot(c - a, normal)))
		return center, normal


	def plane_area(self, plane_coordinates):
		"""Estimate the area of the plane given its coordinates (array)."""
		a = 0
		ox, oy = plane_coordinates[0]
		for x, y in plane_coordinates[1:]:
			a += (x * oy - y * ox)
			ox, oy = x, y
		a_plane = a / 2
		return a_plane


	def rotation_matrix_from_vectors(self, vec1, vec2):
		""" Find the rotation matrix that aligns vec1 to vec2
		vec1: A 3d "source" vector
		vec2: A 3d "destination" vector
		Return: A transform matrix (3x3)
		"""
		a, b = (vec1 / np.linalg.norm(vec1)).reshape(3), (vec2 / np.linalg.norm(vec2)).reshape(3)
		v = np.cross(a, b)
		c = np.dot(a, b)
		s = np.linalg.norm(v)
		kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
		rotation_matrix = np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s ** 2))
		return rotation_matrix

	def publish_data(self, roll, pitch, counter):

		# Define msg
		rpy_data = RPYAxes()
		
		rpy_data.header.seq = counter
		rpy_data.header.stamp = rospy.Time.now()
		rpy_data.header.frame_id = ''

		rpy_data.skyline  =  [roll, pitch]

		self.pub_refs.publish(rpy_data)


	# process frames until user exits
	# while True:
	def process_img_callback(self, data):
	# while not rospy.is_shutdown():
		

		# capture the next image
		# img_input = input.Capture()
		# img_input_data = data
		try: 
			output = self.bridge.imgmsg_to_cv2(data, desired_encoding='mono8')
			img = np.copy(output)
		except CvBridgeError as e:
			print(e)

		self.image_counter +=1
		if self.image_counter == 1:			
			self.img_height = img.shape[0] 
			self.img_width = img.shape[1] 
			# print("h*w = ", self.img_height, self.img_width)
			#HFOV = Horizontal Field of View of the camera
			#HFOV_GoPro = 170° ; Raspberry Camera = 62.2°
			HFOV = 62.2 #degrees
			self.focal_length_pixel = (self.img_width * 0.5) / np.tan(HFOV * 0.5 * np.pi / 180.0) #pixels
			#Focal length milimiters 1.7 mm lens GoPro. Raspberry 3.04 mm
			self.focal_length_mm = 3.04 #milimeters
			img = self.boundary_removal(img)
			#(x_position, y_position) = Sky line coordinates
			x_position = []
			y_position = []
			for i in range (0, len(img[0, :]), 5):
				for j in range (0, len(img[:, 0]), 5):
					if img[j, i] == 0:
						x_position.append(i)
						y_position.append(j)
						break
			#a = Slope
			#b = y-intercept
			a, b = self.fit(x_position, y_position)
			# print("a, b = ", a,b)
			angle_a = np.arctan(a) #radians
			self.last_last_angle = angle_a #radians
			self.last_last_b = b #pixels

			#Reference Sky line Values
			img_reference = img
			self.reference_angle = angle_a #radians
			self.reference_b = b #pixels
			
			#Reference Sky line Distance
			self.d_ref = self.distance(x_position, y_position) #pixels
			
			#Reference Area below the first frame sky line
			height_rectangle = (self.img_height - y_position[0])
			self.area_ref = height_rectangle * self.img_width #pixels^2
			
			#Plane 1 Reference Values
			# (p,q) = Center Straight Line Coordinates
			p = x_position[int(len(x_position) / 2)] #pixels
			q = y_position[int(len(y_position) / 2)] #pixels
			#Threshold values
			t_x = 200 #pixels
			t_y = 100 #pixels
			altitude = 300 #milimeters
			# m, n, l = Coordinates of three points in a reference plane
			if p > t_x and q > t_y:
				m = np.array([p, q + 50, altitude * self.focal_length_pixel / self.focal_length_mm]) #pixels
				n = np.array([p + 100, q + 100, altitude * self.focal_length_pixel / self.focal_length_mm]) #pixels
				l = np.array([p - 100, self.img_height, altitude * self.focal_length_pixel / self.focal_length_mm]) #pixels
			else:
				m = np.array([p + t_x, q + t_y + 50, altitude * self.focal_length_pixel / self.focal_length_mm]) #pixels
				n = np.array([p + t_x + 100, q + t_y + 100, altitude * self.focal_length_pixel / self.focal_length_mm]) #pixels
				l = np.array([p + t_x - 100, self.img_height, altitude * self.focal_length_pixel / self.focal_length_mm]) #pixels
			#Estimate reference plane center and reference plane normal vector
			center_ref, self.vec_ref = self.estimate_plane(m, n, l)

			# print("img_height", self.img_height)
			# print("img_width", self.img_width)
			# print("last_last_angle = ", self.last_last_angle) #radians
			# print("last_last_b = ", self.last_last_b )#pixels")
			# print("reference_angle =", self.reference_angle)
			# print("reference_b = ", self.reference_b)

		elif self.image_counter == 2:
			img = self.boundary_removal(img)
			#(x_position, y_position) = Current sky line image coordinates
			x_position = []
			y_position = []
			for i in range (0, len(img[0, :]), 5):
				for j in range (0, len(img[:, 0]), 5):
					if img[j, i] == 0:
						x_position.append(i)
						y_position.append(j)
						break
			#a = Current Slope
			#b = Current y-intercept
			a, b = self.fit(x_position, y_position)   
			angle_a = np.arctan(a) #radians
			self.last_angle = angle_a #radians
			self.last_b = b #pixels

			#Increase of current a and b with respect to the reference image
			self.theta_inc = self.last_angle - self.last_last_angle #radians
			self.b_inc = self.last_b - self.last_last_b #pixels

			#Sky line roll angle compensation movement with respect to the reference image
			angle_current = angle_a #radians
			#Sky line pich angle compensation movement with respect to the reference image variables
			b_current = b #pixels
			
			#Roll angle compensation movement
			if angle_current != self.reference_angle:
				self.roll_angle_compensate = angle_current - self.reference_angle #radians
				self.pitch_angle_compensate = 0.0
				#(x_position, y_position) = Current sky line image coordinates
				x_position = []
				y_position = []
				for i in range (0, len(img[0, :]), 5):
					for j in range (0, len(img[:, 0]), 5):
						if img[j, i] == 0:
							x_position.append(i)
							y_position.append(j)
							break
				#a = Current Slope
				#b = Current y-intercept
				a, b = self.fit(x_position, y_position)   
				angle_a = np.arctan(a) #radians
				self.last_angle = angle_a #radians
				self.last_b = b #pixels

				#Increase of current a and b with respect to the reference image
				self.theta_inc = self.last_angle - self.last_last_angle #radians
				self.b_inc = self.last_b - self.last_last_b #pixels
				
				#Sky line roll angle compensation movement with respect to the reference image variables
				angle_current = angle_a #radians
				#Sky line pitch angle compensation movement with respect to the reference image variables
				b_current = b #pixels
				
				#Pitch angle compensation movement
				if angle_current == self.reference_angle:
					if b_current != self.reference_b:
						b_movement = b_current - self.reference_b #pixels
						#b measured from the center of the image to the reference image
						b_center_ref = self.reference_b - self.img_height / 2 #pixels
						#Angle measured from the center of the image to the reference image
						b_center_ref_angle = np.arctan(b_center_ref / self.focal_length_pixel) #radians
						#b measured from the center of the image to the current sky line image height
						b_total = b_center_ref + b_movement #pixels
						#Angle measured from the center of the image to the current sky line image height
						b_total_angle = np.arctan(b_total / self.focal_length_pixel) #radians
						self.pitch_angle_compensate = np.arctan(b_total_angle - b_center_ref_angle) #radians
					else:
						self.pitch_angle_compensate = 0.0
						
			#Pitch angle compensation movement
			else:
				self.roll_angle_compensate = 0.0
				if b_current != self.reference_b:
					b_movement = b_current - self.reference_b #pixels
					#b measured from the center of the image to the reference image
					b_center_ref = self.reference_b - self.img_height / 2 #pixels
					#Angle measured from the center of the image to the reference image
					b_center_ref_angle = np.arctan(b_center_ref / self.focal_length_pixel) #radians
					#b measured from the center of the image to the current sky line image height
					b_total = b_center_ref + b_movement #pixels
					#Angle measured from the center of the image to the current sky line image height
					b_total_angle = np.arctan(b_total / self.focal_length_pixel) #radians
					self.pitch_angle_compensate = np.arctan(b_total_angle - b_center_ref_angle) #radians
				else:
					self.pitch_angle_compensate = 0.0


			#Plane 2 Values
			# (p,q) = Current Center Straight Line Coordinates
			p = x_position[int(len(x_position) / 2)] #pixels
			q = y_position[int(len(y_position) / 2)] #pixels
			#Threshold values
			t_x = 200 #pixels
			t_y = 100 #pixels
			altitude = 300 #milimeters
			# m, n, l = Coordinates of three points in a current plane
			if p > t_x and q > t_y:
				m = np.array([p, q + 50, altitude * self.focal_length_pixel / self.focal_length_mm]) #pixels
				n = np.array([p + 100, q + 100, altitude * self.focal_length_pixel / self.focal_length_mm]) #pixels
				l = np.array([p - 100, self.img_height, altitude * self.focal_length_pixel / self.focal_length_mm]) #pixels
			else:
				m = np.array([p + t_x, q + t_y + 50, altitude * self.focal_length_pixel / self.focal_length_mm]) #pixels
				n = np.array([p + t_x + 100, q + t_y + 100, altitude * self.focal_length_pixel / self.focal_length_mm]) #pixels
				l = np.array([p + t_x - 100, self.img_height, altitude * self.focal_length_pixel / self.focal_length_mm]) #pixels
			#Estimate plane center and plane normal vector
			center_current, vec_current = self.estimate_plane(m, n, l)
			#Rotation Matrix between the current plane and the reference plane
			rotation_matrix = self.rotation_matrix_from_vectors(vec_current, self.vec_ref)
			#Plane angles compensation
			theta_x = np.arctan2(rotation_matrix[2,1], rotation_matrix[2,2]) #radians
			theta_y = np.arctan2(-rotation_matrix[2,0], np.sqrt((rotation_matrix[2,1]) ** 2 + (rotation_matrix[2,2]) ** 2)) #radians
			theta_z = np.arctan2(rotation_matrix[1,0], rotation_matrix[0,0]) #radians

			print('Theta x, Theta y, Theta z [rad]: ', theta_x, theta_y, theta_z)
			print("Roll and Pitch angles [rad]:", self.roll_angle_compensate, self.pitch_angle_compensate)
			self.publish_data(self.roll_angle_compensate, self.pitch_angle_compensate, self.image_counter)


		elif self.image_counter > 2:
			img = self.boundary_removal(img) 
			#Sky line motion prediction values
			theta_pred = self.theta_inc + self.last_angle #radians
			b_pred = self.last_b + self.b_inc #pixels
			a_p = np.tan(theta_pred) #Sky line slope predicted
			b_p = b_pred #Sky line y-intercept predicted

			self.last_last_angle = self.last_angle #radians
			self.last_last_b = self.last_b #pixels

			#(x_position_predict,  y_position_predict) = Sky line predicted coordinates
			x_position_predict = [i for i in range(0, len(img[0,:]), 5)]
			y_position_predict = []
			# print("a_p =", a_p, "b_p=", b_p)	
			for x in x_position_predict:
				y = a_p * x + b_p
				y_position_predict.append(y)

			#(x_position,  y_position) = Current sky line image coordinates
			x_position = x_position_predict 
			y_position = []
			for i,j in zip(x_position, y_position_predict):
				j=int(j)
				if img[j,i] == 0: 
					while (img[j,i] == 0):              
						j -= 1 
					y_position.append(j)
				else:
					while (img[j,i] == 127):              
						j += 1 
					y_position.append(j)

			#a = Current Slope
			#b = Current y-intercept
			a, b = self.fit(x_position, y_position)
			angle_a = np.arctan(a) #radians
			self.last_angle = angle_a #radians
			self.last_b = b #pixels

			#Increase of current a and b with respect to the reference image
			self.theta_inc = self.last_angle - self.last_last_angle #radians
			self.b_inc = self.last_b - self.last_last_b #pixels

			#Current Sky line Distance
			d_current = self.distance(x_position, y_position) #pixels
			#Sky line Distance Check
			if d_current <= (1 / 5) * self.d_ref:
				return

			#Sky line roll angle compensation movement with respect to the reference image variables
			angle_current = angle_a #radians
			#Sky line pich angle compensation movement with respect to the reference image variables
			b_current = b #pixels
			
			#Roll angle compensation movement
			if angle_current != self.reference_angle:
				self.roll_angle_compensate = angle_current - self.reference_angle #radians
				self.pitch_angle_compensate = 0.0
				#(x_position,  y_position) = Current sky line image coordinates
				x_position = x_position_predict 
				y_position = []
				for i,j in zip(x_position, y_position_predict):
					j=int(j)
					if img[j,i] == 0: 
						while (img[j,i] == 0):              
							j -= 1 
						y_position.append(j)
					else:
						while (img[j,i] == 127):              
							j += 1 
						y_position.append(j)

				#a = Current Slope
				#b = Current y-intercept
				a, b = self.fit(x_position, y_position)
				angle_a = np.arctan(a) #radians
				self.last_angle = angle_a #radians
				self.last_b = b #pixels

				#Increase of current a and b with respect to the reference image
				self.theta_inc = self.last_angle - self.last_last_angle #radians
				self.b_inc = self.last_b - self.last_last_b #pixels
				
				#Sky line roll angle compensation movement with respect to the reference image variables
				angle_current = angle_a #radians
				#Sky line pitch angle compensation movement with respect to the reference image variables
				b_current = b #pixels
				
				#Pitch angle compensation movement
				if angle_current == self.reference_angle:
					if b_current != self.reference_b:
						b_movement = b_current - self.reference_b #pixels
						#b measured from the center of the image to the reference image
						b_center_ref = self.reference_b - self.img_height / 2 #pixels
						#Angle measured from the center of the image to the reference image
						b_center_ref_angle = np.arctan(b_center_ref / self.focal_length_pixel) #radians
						#b measured from the center of the image to the current sky line image height
						b_total = b_center_ref + b_movement #pixels
						#Angle measured from the center of the image to the current sky line image height
						b_total_angle = np.arctan(b_total / self.focal_length_pixel) #radians
						self.pitch_angle_compensate = np.arctan(b_total_angle - b_center_ref_angle) #radians
					else:
						self.pitch_angle_compensate = 0.0
						
			#Pitch angle compensation movement
			else:
				self.roll_angle_compensate = 0.0
				if b_current != self.reference_b:
					b_movement = b_current - self.reference_b #pixels
					#b measured from the center of the image to the reference image
					b_center_ref = self.reference_b - self.img_height / 2 #pixels
					#Angle measured from the center of the image to the reference image
					b_center_ref_angle = np.arctan(b_center_ref / self.focal_length_pixel) #radians
					#b measured from the center of the image to the current sky line image height
					b_total = b_center_ref + b_movement #pixels
					#Angle measured from the center of the image to the current sky line image height
					b_total_angle = np.arctan(b_total / self.focal_length_pixel) #radians
					self.pitch_angle_compensate = np.arctan(b_total_angle - b_center_ref_angle) #radians
				else:
					self.pitch_angle_compensate = 0.0
					

			#Plane 3 Values
			# (p,q) = Current Center Straight Line Coordinates
			p = x_position[int(len(x_position) / 2)] #pixels
			q = y_position[int(len(y_position) / 2)] #pixels
			#Threshold values
			t_x = 200 #pixels
			t_y = 100 #pixels
			altitude = 300 #milimeters
			# m, n, l = Coordinates of three points in a current plane
			if p > t_x and q > t_y:
				m = np.array([p, q + 50, altitude * self.focal_length_pixel / self.focal_length_mm]) #pixels
				n = np.array([p + 100, q + 100, altitude * self.focal_length_pixel / self.focal_length_mm]) #pixels
				l = np.array([p - 100, self.img_height, (altitude - 0.4) * self.focal_length_pixel / self.focal_length_mm]) #pixels
			else:
				m = np.array([p + t_x, q + t_y + 50, altitude * self.focal_length_pixel / self.focal_length_mm]) #pixels
				n = np.array([p + t_x + 100, q + t_y + 100, altitude * self.focal_length_pixel / self.focal_length_mm]) #pixels
				l = np.array([p + t_x - 100, self.img_height, (altitude - 0.4) * self.focal_length_pixel / self.focal_length_mm]) #pixels

			#Current Area
			m_2=np.delete(m, -1)
			n_2=np.delete(n, -1)
			l_2=np.delete(l, -1)
			plane_coordinates = np.array([m_2, n_2, l_2]) #pixels
			area_current = abs(self.plane_area(plane_coordinates)) #pixels^2
			#Plane Area Check
			if area_current <= (1 / 5) * self.area_ref:
			 	return
			#Estimate plane center and plane normal vector
			center_current, vec_current = self.estimate_plane(m, n, l)

			#Rotation Matrix between the current plane and the reference plane
			rotation_matrix = self.rotation_matrix_from_vectors(vec_current, self.vec_ref)
			#Plane angles compensation
			theta_x = np.arctan2(rotation_matrix[2,1], rotation_matrix[2,2]) #radians
			theta_y = np.arctan2(-rotation_matrix[2,0], np.sqrt((rotation_matrix[2,1]) ** 2 + (rotation_matrix[2,2]) ** 2)) #radians
			theta_z = np.arctan2(rotation_matrix[1,0], rotation_matrix[0,0]) #radians

			print('Theta x, Theta y, Theta z [rad]: ', theta_x, theta_y, theta_z)
			print("Roll and Pitch angles [rad]:", self.roll_angle_compensate, self.pitch_angle_compensate)
			self.publish_data(self.roll_angle_compensate, self.pitch_angle_compensate, self.image_counter)



if __name__ == '__main__':

	### ROS configuration

	percep = Perception()

	# ROS node creation
	rospy.init_node("perception")

	try:
		rospy.spin()
	except KeyboardInterrupt:
		pass
			
