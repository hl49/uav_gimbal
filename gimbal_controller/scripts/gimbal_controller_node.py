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
        self.tilt = 0.0
        self.roll = 0.0
        self.ref_pos = {'tilt':0.0, 'roll': 0.0} # From perception: input
        self.target_pos = {'tilt':0.0, 'roll': 0.0} # Command we send to servos: output
        self.ini_pos = {'tilt':0.0, 'roll':0.0} # 
        self.tilt_limit_max = math.radians(45)
        self.tilt_limit_min = math.radians(-45)
        self.roll_limit_max = math.radians(45)
        self.roll_limit_min = math.radians(-45)
        self.count_init_servo_pos = 0 
        self.num_samples_init = 20 # Number of samples (ms)
        self.ini_tilt_arr = np.zeros([self.num_samples_init, 1])
        self.ini_roll_arr = np.zeros([self.num_samples_init, 1])

        # self.max_tilt = math.radians(65)
        # self.max_roll = math.radians(45)

        # Init ROS node
        rospy.init_node("gimbal_controller")


        # Create topics 

        # self.sub_target = rospy.Subscriber("/Imu_data",
        #         Imu,
        #         self._get_target_callback,
        #         queue_size=1)

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
        if self.count_init_servo_pos < self.num_samples_init:
            joints = dict(zip(data.name, data.position))
            self.ini_tilt_arr[self.count_init_servo_pos, 0] = joints['tilt']
            self.ini_roll_arr[self.count_init_servo_pos, 0] = joints['roll']
            self.count_init_servo_pos += 1
        else:
            # Check variance
            self.ini_pos['tilt'] = self.ini_tilt_arr.mean()
            self.ini_pos['roll'] = self.ini_roll_arr.mean()
            self.pos_initilized = True
            self.sub_joint_states.unregister()
    

    def get_perception_data_callback(self, data):
        # print(data.data)
        # self.tilt = round(data.data[0],2) * -1.0
        # self.roll = round(data.data[1],2) * -1.0

        pass

        # if self.tilt > - self.max_tilt and self.tilt <

    def pub_servos_pos(self):

        servo_data = JointState()

        servo_data.header.stamp = rospy.Time.now()
        servo_data.header.frame_id = ''
        servo_data.header.seq = self.servo_data_seq_counter

        servo_data.name = ["tilt", "roll"]
        servo_data.position = [self.target_pos['tilt'], self.target_pos['roll']]

        self.servo_data_seq_counter =+ 1
        self.servo_pub.publish(servo_data)


    def stabilize(self):

        # if there is not target position, set 0. 

        # Controller
        
        # Frequency of execution
        # Verify with Ariel perception frequency
        rate = rospy.Rate(self.frequency)

        while not rospy.is_shutdown():
            if self.pos_initilized == True:
                self.target_pos['tilt'] = self.ref_pos['tilt'] + self.ini_pos['tilt']
                self.target_pos['roll'] = self.ref_pos['roll'] + self.ini_pos['roll']


                # Check position limits in tilt axis
                if self.target_pos['tilt'] > self.tilt_limit_max:
                    self.target_pos['tilt'] = self.tilt_limit_max
                elif self.target_pos['tilt'] < self.tilt_limit_min:
                    self.target_pos['tilt'] = self.tilt_limit_min

                # Check position limits in roll axis
                if self.target_pos['roll'] > self.roll_limit_max:
                    self.target_pos['roll'] = self.roll_limit_max
                elif self.target_pos['roll'] < self.roll_limit_min:
                    self.target_pos['roll'] = self.roll_limit_min

                self.pub_servos_pos()

            rate.sleep()


if __name__ == "__main__":

    gimbal = Gimbal()

    try:
        gimbal.stabilize();
    except rospy.ROSInterruptException:
        pass

    
