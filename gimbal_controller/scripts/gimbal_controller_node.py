#!/usr/bin/env python

# from atexit import unregister
# from logging import setLogRecordFactory, shutdown
# from ntpath import join
from __future__ import division

import rospy
import numpy as np
import math
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
from perception.msg import RPYAxes 
from tf.transformations import euler_from_quaternion

class Gimbal:

	def __init__(self):

		# Set variables
		# Node frequency
		self.frequency = 60.0 # Hz
		self.pos_initilized = False
		self.servo_data_seq_counter = 0
		# Angles roll, pitch and yaw are renamed to
		# roll, pitch, pan.
		self.skyline_rpy = {'roll': 0.0, 'pitch':0.0}	# From perception-skyline: input
		self.gndPlane_rpy = {'roll': 0.0, 'pitch':0.0}	# From perception-ground plane: input
		self.imu_rpy = {'roll':0.0, 'pitch':0.0}
		self.ini_pos = {'roll':0.0, 'pitch':0.0}
		self.pitch_limit_max = math.radians(35)
		self.pitch_limit_min = math.radians(-35)
		self.roll_limit_max = math.radians(42)
		self.roll_limit_min = math.radians(-42)
		self.count_init_servo_pos = 0 
		self.num_samples_init = 20 # Number of samples (ms)
		self.ini_pitch_arr = np.zeros([self.num_samples_init, 1])
		self.ini_roll_arr = np.zeros([self.num_samples_init, 1])
		self.a = 0.60 # a const for imu
		self.b = 0.20 # b const for skyline
		self.c = 0.20 # c const for ground plane
		self.tf = np.array([[0, 1, 0, 0],
			[0, 0, -1, 18.765],
			[-1, 0, 0, 46.367],
			[0, 0, 0, 1]])

		# Init ROS node
		rospy.init_node("gimbal_controller")


		# Create topics 

		self.pub_servo = rospy.Publisher("/desired_joint_states",
					JointState,
					queue_size = 1)

		self.pub_tf_sky = rospy.Publisher("/tf_perception_rpy",
					JointState,
					queue_size = 1)

		self.sub_joint_states = rospy.Subscriber("/joint_states",
					JointState,
					self.get_ini_joint_states_callback,
					queue_size = 1)

		self.sub_perception = rospy.Subscriber("/perception_rpy",
					RPYAxes,
					self.get_perception_data_callback,
					queue_size = 1)

		self.sub_perception = rospy.Subscriber("/imu/data",
		# self.sub_perception = rospy.Subscriber("/imu/filtered_data",
					Imu,
					self.get_imu_data_callback,
					queue_size = 5)

	def get_ini_joint_states_callback(self, data):
		""" Get initial position of servos and calculate the mean value 
			to stablish the general initial position
		"""
		if self.count_init_servo_pos < self.num_samples_init:
			joints = dict(zip(data.name, data.position))
			self.ini_pitch_arr[self.count_init_servo_pos, 0] = joints['pitch']
			self.ini_roll_arr[self.count_init_servo_pos, 0] = joints['roll']
			self.count_init_servo_pos += 1
		else:
			# Check variance
			self.ini_pos['pitch'] = self.ini_pitch_arr.mean()
			self.ini_pos['roll'] = self.ini_roll_arr.mean()
			self.pos_initilized = True
			print("Servos Initialized!")
			self.sub_joint_states.unregister()
	

	def get_perception_data_callback(self, data):
		""" Read reference position vector (ref_pos) from topic
			and check they are within a desired range.
		"""
		skyline = dict(zip(data.name, data.skyline))
		norm_vect = dict(zip(data.name, data.norm_vect))
		self.skyline_rpy['roll'] = skyline['roll']*(-1.0)
		self.skyline_rpy['pitch'] = skyline['pitch']
		self.gndPlane_rpy['roll'] = norm_vect['roll']
		self.gndPlane_rpy['pitch'] = norm_vect['pitch']
	
	def get_imu_data_callback(self, data):
		# print(data.orientation)
		x = data.orientation.x
		y = data.orientation.y
		z = data.orientation.z
		w = data.orientation.w
		
		r, p, y = euler_from_quaternion([x, y, z, w])
		self.imu_rpy['roll'] = y 
		self.imu_rpy['pitch'] = r

	def stabilize(self):

		# Calculate the target position using the initial and reference positions,
		# then publish those angles (radians)

		rate = rospy.Rate(self.frequency)

		#Time values
		currentTime = 0.0
		lastTime = rospy.Time.now().to_nsec()
		dt = 0.0


		# # Roll PID constants
		roll_kp = 1.0 
		roll_ki = 0.004
		roll_kd = 0.8
		roll_i_term = 0.0
		roll_last_error = 0.0
		roll_sp = 0.0	# Roll setpoint
		roll_feedback = 0.0
		roll_servo = 0.0

		# # Pitch PID constants
		pitch_kp = 1.0
		pitch_ki = 0.004
		pitch_kd = 0.8
		pitch_i_term = 0.0
		pitch_last_error = 0.0
		pitch_sp = 0.0
		pitch_feedback = 0.0
		pitch_servo = 0.0

		while not rospy.is_shutdown():
			if self.pos_initilized == True:

				currentTime = rospy.Time.now().to_nsec()
				dt = round((currentTime - lastTime)/1e6,2)  # delta time in ms
				lastTime = currentTime

				roll_feedback = (self.a * self.imu_rpy['roll'] +
									self.b * self.skyline_rpy['roll'] + 
									self.c * self.gndPlane_rpy['roll'])

				pitch_feedback = (self.a * self.imu_rpy['pitch'] +
									self.b * self.skyline_rpy['pitch'] + 
									self.c * self.gndPlane_rpy['pitch'])
				
				# Roll PID
				roll_error = roll_sp - roll_feedback
				roll_pid_p = roll_kp * roll_error
				roll_i_term = roll_i_term + (roll_error * dt)
				roll_d_term = (roll_last_error - roll_error)/dt
				roll_last_error = roll_error

				roll_servo = roll_pid_p + roll_i_term * roll_ki + roll_d_term * roll_kd

				# Pitch PID
				pitch_error = pitch_sp - pitch_feedback
				pitch_pid_p = pitch_kp * pitch_error
				pitch_i_term = pitch_i_term + (pitch_error * dt)
				pitch_d_term = (pitch_last_error - pitch_error)/dt
				pitch_last_error = pitch_error

				pitch_servo = pitch_pid_p + pitch_i_term * pitch_ki + pitch_d_term * pitch_kd

				# Check position limits in pitch axis
				if pitch_servo > self.pitch_limit_max:
					pitch_servo = self.pitch_limit_max
				elif pitch_servo < self.pitch_limit_min:
					pitch_servo = self.pitch_limit_min

				# Check position limits in roll axis
				if roll_servo > self.roll_limit_max:
					roll_servo = self.roll_limit_max
				elif roll_servo < self.roll_limit_min:
					roll_servo = self.roll_limit_min
				

				# print("roll ", "{:.4f}".format(math.degrees(self.imu_rpy['roll'])),
				# 	"pitch:", "{:.4f}".format(math.degrees(self.imu_rpy['pitch'])))
				# print("roll_sky: ","{:.4f}".format(math.degrees(self.skyline_rpy['roll'])),
				# 	"pitch_sky: ","{:.4f}".format(math.degrees(self.skyline_rpy['pitch'])),)
				# print("roll_norm: ","{:.4f}".format(math.degrees(self.gndPlane_rpy['roll'])),
				# 	"pitch_norm: ","{:.4f}".format(math.degrees(self.gndPlane_rpy['pitch'])),)


				print("roll:", "{:.2f}".format(math.degrees(roll_servo)),
						"pitch: ", "{:.2f}".format(math.degrees(pitch_servo)))

				self.pub_servos_pos(sp_r = roll_servo, sp_p = pitch_servo)

				rate.sleep()


	def pub_servos_pos(self, sp_r = 0.0, sp_p = 0.0):
		""" Publish servo position to topic read by the driver
		
			Parameters:
			sp_r: set point roll
			sp_p: set point pitch
		"""

		servo_data = JointState()

		servo_data.header.stamp = rospy.Time.now()
		servo_data.header.frame_id = ''
		servo_data.header.seq = self.servo_data_seq_counter

		servo_data.name = ["pitch", "roll"]
		servo_data.position = [sp_p, sp_r]

		self.servo_data_seq_counter =+ 1
		self.pub_servo.publish(servo_data)


if __name__ == "__main__":

	gimbal = Gimbal()
	try:
		gimbal.stabilize()
	except rospy.ROSInterruptException:
		pass
