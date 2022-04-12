#!/usr/bin/env python

import rospy
from pymavlink import mavutil
# import time
import numpy as np
import matplotlib.pyplot as plt
import math
from std_msgs.msg import Float32MultiArray


class Pixhawk:
    def __init__(self):
        self.roll = 0.0
        self.pitch = 0.0

        # Init ROS node
        rospy.init_node("pixhawk_imu")

        # Start a connection listening to a UDP port
        self.the_connection = mavutil.mavlink_connection('/dev/ttyACM1', baud=57600)
        # the_connection = mavutil.mavlink_connection('tcp:127.0.0.1:5762')

        self.the_connection.wait_heartbeat()
        print("Heartbeat from system (system %u component %u)" % (self.the_connection.target_system, self.the_connection.target_component))

        self.imu_pub = rospy.Publisher("/pixhawk_rpy",
                    Float32MultiArray,
                    queue_size=1)

    def get_imu(self, event=None):

        msg = self.the_connection.recv_match(type='ATTITUDE', blocking=True)
        if not msg:
            pass
        # self.roll = round(math.degrees(float(msg.roll)), 2)
        # self.pitch = round(math.degrees(float(msg.pitch)), 2)
        self.roll = round(float(msg.roll), 2)
        self.pitch = round(float(msg.pitch), 2)

        data_imu = Float32MultiArray()
        data_imu.data = [self.pitch, self.roll]

        self.imu_pub.publish(data_imu)


if __name__ == '__main__':
    pixhawk = Pixhawk()
    rospy.Timer(rospy.Duration(0.1), pixhawk.get_imu)

    rospy.spin()