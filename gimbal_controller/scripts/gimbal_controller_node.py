#!/usr/bin/env python

# from atexit import unregister
# from logging import setLogRecordFactory, shutdown
# from ntpath import join
import rospy
import numpy as np
import math
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from perception.msg import RPYAxes 

class Gimbal:

    def __init__(self):

        # Set variables
        # Node frequency
        self.frequency = 10.0 # Hz
        self.pos_initilized = False
        self.servo_data_seq_counter = 0
        self.pitch = 0.0
        self.roll = 0.0
        # Angles roll, pitch and yaw are renamed to
        # roll, pitch, pan.
        self.ref_pos = {'roll': 0.0, 'pitch':0.0} # From perception: input
        self.target_pos = {'roll': 0.0, 'pitch':0.0} # Command we send to servos: output
        self.ini_pos = {'roll':0.0, 'pitch':0.0} # 
        self.pitch_limit_max = math.radians(45)
        self.pitch_limit_min = math.radians(-45)
        self.roll_limit_max = math.radians(45)
        self.roll_limit_min = math.radians(-45)
        self.count_init_servo_pos = 0 
        self.num_samples_init = 20 # Number of samples (ms)
        self.ini_pitch_arr = np.zeros([self.num_samples_init, 1])
        self.ini_roll_arr = np.zeros([self.num_samples_init, 1])


        # Init ROS node
        rospy.init_node("gimbal_controller")


        # Create topics 

        self.servo_pub = rospy.Publisher("/desired_joint_states",
                    JointState,
                    queue_size = 1)

        self.sub_joint_states = rospy.Subscriber("/joint_states",
                    JointState,
                    self.get_ini_joint_states_callback,
                    queue_size = 1)

        self.sub_imu = rospy.Subscriber("/perception_rpy",
                    RPYAxes,
                    self.get_perception_data_callback,
                    queue_size = 1)


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
        joints = dict(zip(data.name, data.position))
        self.ref_pos['roll'] = joints['roll']
        self.ref_pos['pitch'] = joints['pitch']

        # Check reference position is within the desired range (radians)
        if self.ref_pos['pitch'] > self.pitch_limit_max:
            self.ref_pos['pitch'] = self.pitch_limit_max
        elif self.ref_pos['pitch'] < self.pitch_limit_min:
            self.ref_pos['pitch'] = self.pitch_limit_min

        if self.ref_pos['roll'] > self.roll_limit_max:
            self.ref_pos['roll'] = self.roll_limit_max
        elif self.ref_pos['roll'] < self.roll_limit_min:
            self.ref_pos['roll'] = self.roll_limit_min
        
        # Calculate the target position using the initial and reference positions,
        # then publish those angles (radians)
        if self.pos_initilized == True:
            self.target_pos['pitch'] = self.ref_pos['pitch'] + self.ini_pos['pitch']
            self.target_pos['roll'] = self.ref_pos['roll'] + self.ini_pos['roll']

            # Check position limits in pitch axis
            if self.target_pos['pitch'] > self.pitch_limit_max:
                self.target_pos['pitch'] = self.pitch_limit_max
            elif self.target_pos['pitch'] < self.pitch_limit_min:
                self.target_pos['pitch'] = self.pitch_limit_min

            # Check position limits in roll axis
            if self.target_pos['roll'] > self.roll_limit_max:
                self.target_pos['roll'] = self.roll_limit_max
            elif self.target_pos['roll'] < self.roll_limit_min:
                self.target_pos['roll'] = self.roll_limit_min

            self.pub_servos_pos()
            # print('roll = ', "{:.4f}".format(self.target_pos['roll']),'pitch = ', "{:.4f}".format(self.target_pos['pitch']))
        

    def pub_servos_pos(self):
        """ Publish servo position to topic read by the driver
        """

        servo_data = JointState()

        servo_data.header.stamp = rospy.Time.now()
        servo_data.header.frame_id = ''
        servo_data.header.seq = self.servo_data_seq_counter

        servo_data.name = ["pitch", "roll"]
        servo_data.position = [self.target_pos['pitch'], self.target_pos['roll']]

        self.servo_data_seq_counter =+ 1
        self.servo_pub.publish(servo_data)


if __name__ == "__main__":

    gimbal = Gimbal()
    rospy.spin()  
