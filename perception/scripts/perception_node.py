#!/usr/bin/python
# -*- coding: utf-8 -*-

from __future__ import division
import numpy as np
import cv2 as cv

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
		return a, b


	def distance(self, x, y): 
		"""Straight line distance between two points"""
		x1 = x[0]
		y1 = y[0]
		x2 = x[-1]
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


	def trapezoid_area(self, base_1, base_2, tr_height):
		"""Estimate the area of a rectangular trapezoid"""
		area = ((base_1 + base_2) / 2) * tr_height
		return area


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


	def publish_data(self, pitch, roll, pPlane, yPlane, rPlane, counter):
		# Define msg
		rpy_data = RPYAxes()
		
		rpy_data.header.seq = counter
		rpy_data.header.stamp = rospy.Time.now()
		rpy_data.header.frame_id = 'camera_link'

		rpy_data.name = ['pitch', 'yaw', 'roll']
		rpy_data.skyline  =  [pitch, 0, roll]
		rpy_data.norm_vect = [pPlane, yPlane, rPlane]

		self.pub_refs.publish(rpy_data)


	def process_img_callback(self, data):
		# process frames until user exits
		try: 
			output = self.bridge.imgmsg_to_cv2(data, desired_encoding='mono8')
			img = np.copy(output)
		except CvBridgeError as e:
			print(e)
		self.image_counter += 1
		if self.image_counter == 1:			
			self.img_height = img.shape[0] 
			self.img_width = img.shape[1] 
			#HFOV = Horizontal Field of View of the camera
			#HFOV_GoPro = 170° ; Raspberry Camera = 62.2°
			HFOV = 62.2 #degrees
			self.focal_length_pixel = (self.img_width * 0.5) / np.tan(HFOV * 0.5 * np.pi / 180.0) #pixels
			#Focal length milimiters 1.7 mm lens GoPro. Raspberry 3.04 mm
			self.focal_length_mm = 3.04 #milimeters
			try:
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
				#Plane Variable
				altitude = 300 #milimeters
				VFOV_half = 24.4 #degrees Vertical Field of View Raspberry Pi camera (48.8°)
				z_position_mm = altitude / np.tan(np.deg2rad(VFOV_half)) #mm
				z_position_pixels = z_position_mm * self.focal_length_pixel / self.focal_length_mm #pixels
				#Threshold values
				t_y = 100 #pixels
				# m, n, l = Coordinates of three points in a reference plane
				if q > t_y:
					m = np.array([p, q + 100, z_position_pixels + 50]) #pixels
					n = np.array([p + 100, q + 100, z_position_pixels]) #pixels
					l = np.array([p - 100, q + 100, z_position_pixels]) #pixels
				else:
					m = np.array([p, q + t_y + 100, z_position_pixels + 50]) #pixels
					n = np.array([p + 100, q + t_y + 100, z_position_pixels]) #pixels
					l = np.array([p - 100, q + t_y + 100, z_position_pixels]) #pixels
				#Estimate reference plane center and reference plane normal vector
				center_ref, self.vec_ref = self.estimate_plane(m, n, l)

			except Exception as e:
				print(e)
				print("There is not an outdoor image")
				return

		elif self.image_counter == 2:
			try:
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
				x_half = x_position[int(len(x_position) / 2)] #pixels
				b_current = a * x_half + b #pixels
				
				#Roll angle compensation movement
				if (angle_current < self.reference_angle - 0.08) or (angle_current > self.reference_angle + 0.08) :
					self.roll_angle_compensate = angle_current - self.reference_angle #radians
					if (b_current < self.reference_b - 1) or (b_current > self.reference_b + 1) :
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
						print("(Pitch, Yaw, Roll) [rad]:", self.pitch_angle_compensate, 0, self.roll_angle_compensate)
						
					else:
						print("(Pitch, Yaw, Roll) [rad]:", self.pitch_angle_compensate, 0, self.roll_angle_compensate)
						
				#Pitch angle compensation movement
				else:
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
						print("(Pitch, Yaw, Roll) [rad]:", self.pitch_angle_compensate, 0, self.roll_angle_compensate)
						
					else:
						print("(Pitch, Yaw, Roll) [rad]:", self.pitch_angle_compensate, 0, self.roll_angle_compensate)
									
				#Plane 2 Values
				#(p,q) = Center Straight Line Coordinates
				p = x_position[int(len(x_position) / 2)] #pixels
				q = y_position[int(len(y_position) / 2)] #pixels
				y_plane_position_m = a * (p) + (b + 100) #pixels
				y_plane_position_n = a * (p + 100) + (b + 100) #pixels
				y_plane_position_l = a * (p - 100) + (b + 100) #pixels
				#Plane Variable
				altitude = 300 #milimeters
				VFOV_half = 24.4 #degrees Vertical Field of View Raspberry Pi camera (48.8°)
				z_plane_position_mm = altitude / np.tan(np.deg2rad(VFOV_half)) #mm
				z_plane_position_pixels = z_plane_position_mm * self.focal_length_pixel / self.focal_length_mm #pixels
				#Threshold values
				t_y = 100 #pixels
				# m, n, l = Coordinates of three points in a current plane
				b_movementplane = b_current - self.reference_b
				b_movementplane = b_movementplane * 0.1
				z_movementplane = np.sqrt((z_plane_position_pixels + 100) ** 2 - (b_movementplane) ** 2)
				if q > t_y:
					if b_current != self.reference_b:
						m = np.array([p, y_plane_position_m + b_movementplane, z_movementplane]) #pixels
						n = np.array([p + 100, y_plane_position_n, z_plane_position_pixels]) #pixels
						l = np.array([p - 100, y_plane_position_l, z_plane_position_pixels]) #pixels
					else:
						m = np.array([p, y_plane_position_m, z_plane_position_pixels + 100]) #pixels
						n = np.array([p + 100, y_plane_position_n, z_plane_position_pixels]) #pixels
						l = np.array([p - 100, y_plane_position_l, z_plane_position_pixels]) #pixels
				else:
					if b_current != self.reference_b:
						m = np.array([p,  y_plane_position_m + t_y + b_movementplane, z_movementplane]) #pixels
						n = np.array([p + 100,  y_plane_position_n + t_y, z_plane_position_pixels]) #pixels
						l = np.array([p - 100,  y_plane_position_l + t_y, z_plane_position_pixels]) #pixels
					else:
						m = np.array([p,  y_plane_position_m + t_y, z_plane_position_pixels + 100]) #pixels
						n = np.array([p + 100,  y_plane_position_n + t_y, z_plane_position_pixels]) #pixels
						l = np.array([p - 100,  y_plane_position_l + t_y, z_plane_position_pixels]) #pixels			
				#Estimate plane center and plane normal vector
				center_current, vec_current = self.estimate_plane(m, n, l)
				#Rotation Matrix between the current plane and the reference plane
				rotation_matrix = self.rotation_matrix_from_vectors(vec_current, self.vec_ref)
				#Plane angles compensation
				theta_x = np.arctan2(rotation_matrix[2,1], rotation_matrix[2,2]) #radians
				theta_y = np.arctan2(-rotation_matrix[2,0], np.sqrt((rotation_matrix[2,1]) ** 2 + (rotation_matrix[2,2]) ** 2)) #radians
				theta_z = np.arctan2(rotation_matrix[1,0], rotation_matrix[0,0]) #radians

				print('(Pitch_plane, Yaw_plane, Roll_plane) [rad]: ', theta_x, theta_y, theta_z)
				self.publish_data(self.pitch_angle_compensate, self.roll_angle_compensate, theta_x, theta_y, theta_z, self.image_counter)
			except Exception as e:
				print(e)
				print("There is not an outdoor image")
				return


		elif self.image_counter > 2:
			try:
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
				counter_error_1 = 0
				counter_error_2 = 0
				for x in x_position_predict:
					y = a_p * x + b_p
					if y < 0:
						y = 0
						counter_error_1 += counter_error_1
					if y > self.img_height:
						y = self.img_height
						counter_error_2 += counter_error_2
					y_position_predict.append(y)
				if counter_error_1 > 0:
					del y_position_predict[:counter_error_1]
					del x_position_predict[:counter_error_1]
				if counter_error_2 > 0:
					del y_position_predict[-counter_error_2:]
					del x_position_predict[:counter_error_2]

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
				x_half = x_position[int(len(x_position) / 2)] #pixels
				b_current = a * x_half + b #pixels
				
				#Roll angle compensation movement
				if (angle_current < self.reference_angle - 0.08) or (angle_current > self.reference_angle + 0.08) :
					self.roll_angle_compensate = angle_current - self.reference_angle #radians
					if (b_current < self.reference_b - 1) or (b_current > self.reference_b + 1) :
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
						print("(Pitch, Yaw, Roll) [rad]:", self.pitch_angle_compensate, 0, self.roll_angle_compensate)
						
					else:
						print("(Pitch, Yaw, Roll) [rad]:", self.pitch_angle_compensate, 0, self.roll_angle_compensate)
				#Pitch angle compensation movement
				else:
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
						print("(Pitch, Yaw, Roll) [rad]:", self.pitch_angle_compensate, 0, self.roll_angle_compensate)
					else:
						print("(Pitch, Yaw, Roll) [rad]:", self.pitch_angle_compensate, 0, self.roll_angle_compensate)
						
				#Plane 3 Values
				#(p,q) = Center Straight Line Coordinates
				p = x_position[int(len(x_position) / 2)] #pixels
				q = y_position[int(len(y_position) / 2)] #pixels
				y_plane_position_m = a * (p) + (b + 100) #pixels
				y_plane_position_n = a * (p + 100) + (b + 100) #pixels
				y_plane_position_l = a * (p - 100) + (b + 100) #pixels
	   			#Plane Variable
				altitude = 300 #milimeters
				VFOV_half = 24.4 #degrees Vertical Field of View Raspberry Pi camera (48.8°)
				z_plane_position_mm = altitude / np.tan(np.deg2rad(VFOV_half)) #mm
				z_plane_position_pixels = z_plane_position_mm * self.focal_length_pixel / self.focal_length_mm #pixels
				#Threshold values
				t_y = 100 #pixels
				# m, n, l = Coordinates of three points in a current plane
				b_movementplane = b_current - self.reference_b
				b_movementplane = b_movementplane * 0.1
				z_movementplane = np.sqrt((z_plane_position_pixels + 100) ** 2 - (b_movementplane) ** 2)
				if q > t_y:
					if b_current != self.reference_b:
						m = np.array([p, y_plane_position_m + b_movementplane, z_movementplane]) #pixels
						n = np.array([p + 100, y_plane_position_n, z_plane_position_pixels]) #pixels
						l = np.array([p - 100, y_plane_position_l, z_plane_position_pixels]) #pixels
					else:
						m = np.array([p, y_plane_position_m, z_plane_position_pixels + 100]) #pixels
						n = np.array([p + 100, y_plane_position_n, z_plane_position_pixels]) #pixels
						l = np.array([p - 100, y_plane_position_l, z_plane_position_pixels]) #pixels
				else:
					if b_current != self.reference_b:
						m = np.array([p,  y_plane_position_m + t_y + b_movementplane, z_movementplane]) #pixels
						n = np.array([p + 100,  y_plane_position_n + t_y, z_plane_position_pixels]) #pixels
						l = np.array([p - 100,  y_plane_position_l + t_y, z_plane_position_pixels]) #pixels
					else:
						m = np.array([p,  y_plane_position_m + t_y, z_plane_position_pixels + 100]) #pixels
						n = np.array([p + 100,  y_plane_position_n + t_y, z_plane_position_pixels]) #pixels
						l = np.array([p - 100,  y_plane_position_l + t_y, z_plane_position_pixels]) #pixels
				
				#Current Area
				base_1 = self.img_height - y_position[0]
				base_2 = self.img_height - y_position[-1]
				tr_height = self.img_width
				area_current = self.trapezoid_area(base_1, base_2, tr_height) #pixels^2'''
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

				print('(Pitch_plane, Yaw_plane, Roll_plane) [rad]: ', theta_x, theta_y, theta_z)
				self.publish_data(self.pitch_angle_compensate, self.roll_angle_compensate, theta_x, theta_y, theta_z, self.image_counter)
			except Exception as e:
				print(e)
				print("There is not an outdoor image")
				return



if __name__ == '__main__':

	#ROS configuration

	percep = Perception()

	#ROS node creation
	rospy.init_node("perception")

	try:
		rospy.spin()
	except KeyboardInterrupt:
		pass
			
