from __future__ import division

import os

import rospy
from std_msgs.msg import Float32MultiArray
from intera_core_msgs.msg import EndpointState
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

import numpy as np
# from scipy import signal
# import matplotlib.pyplot as plt

from message_filters import ApproximateTimeSynchronizer, Subscriber

from tf.transformations import euler_from_quaternion, quaternion_from_euler



class GetData(object):
    def __init__(self):
        self.pcf_data = np.zeros((1,14))
        self.endeff_data = np.empty((1,15))
        self.data = np.empty((1,16))
        # self.gg = grasp_generator()

    def start_recording(self):

        self.pcf_sub = rospy.Subscriber("/sensor_values", Float32MultiArray, self.pcf_callback)
        # self.ft_sub = rospy.Subscriber("/robot/limb/right/endpoint_state", EndpointState, self.endeff_callback)

        # broadcast the new frame you are going to subscribe in the next line
        # self.gg.broadcast_frame()

        # MESSAGE FILTERS: http://docs.ros.org/api/message_filters/html/python/#message_filters.TimeSynchronizer
        # self.pcf_sub = Subscriber("/sensor_values", Float32MultiArray)
        # self.ft_sub = Subscriber("/robot/limb/right/endpoint_state", EndpointState)
        # self.pose_sub = Subscriber("/new_right_gripper_frame", PoseStamped)

        # ats = ApproximateTimeSynchronizer([self.pcf_sub, self.ft_sub, self.pose_sub], queue_size=5, slop=0.1, allow_headerless=True)
        # ats.registerCallback(self.got_data)


    def got_data(self, float32MultiArray, endpointState, posestamped):
        # print float32MultiArray.data[0]
        # print endpointState.pose.position.x

        ################## TIME ######################
        sec = posestamped.header.stamp.secs
        nsec = posestamped.header.stamp.nsecs
        ################## POSE ######################
        px = posestamped.pose.position.x
        py = posestamped.pose.position.y
        pz = posestamped.pose.position.z
        ox = posestamped.pose.orientation.x
        oy = posestamped.pose.orientation.y
        oz = posestamped.pose.orientation.z
        ow = posestamped.pose.orientation.w

        # save euler angles instead of quaternions
        # helpful link: http://www.theconstructsim.com/ros-qa-how-to-convert-quaternions-to-euler-angles/
        orientation_list = [ox, oy, oz, ow]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

        ################## ForceTorque ###############
        fx = endpointState.wrench.force.x
        fy = endpointState.wrench.force.y
        fz = endpointState.wrench.force.z
        tx = endpointState.wrench.torque.x
        ty = endpointState.wrench.torque.y
        tz = endpointState.wrench.torque.z
        ############### PCF data #####################
        baro = float32MultiArray.data[0]
        ir = float32MultiArray.data[1]

        self.data = np.append(self.data, np.array([[sec, nsec, px, py, pz, roll, pitch, yaw, fx, fy, fz, tx, ty, tz, baro, ir]]), axis=0)


    def pcf_callback(self, msg):
        print msg
        ################## TIME ######################
        now = rospy.get_rostime()
        sec = now.secs
        nsec = now.nsecs
        ################## PCF ######################
        baro_0 = msg.data[0]
        baro_1 = msg.data[1]
        baro_2 = msg.data[2]
        baro_3 = msg.data[3]
        ir_0 = msg.data[4]
        ir_1 = msg.data[5]
        ir_2 = msg.data[6]
        ir_3 = msg.data[7]
        nn_0 = msg.data[8]
        nn_1 = msg.data[9]
        nn_2 = msg.data[10]
        nn_3 = msg.data[11]
        self.pcf_data = np.append(self.pcf_data, np.array([[sec, nsec, baro_0, baro_1, baro_2, baro_3, ir_0, ir_1, ir_2, ir_3,nn_0,nn_1,nn_2,nn_3]]), axis=0)



    def endeff_callback(self, data):
        ################## TIME ######################
        sec = data.header.stamp.secs
        nsec = data.header.stamp.nsecs
        ################## POSE ######################
        px = data.pose.position.x
        py = data.pose.position.y
        pz = data.pose.position.z
        ox = data.pose.orientation.x
        oy = data.pose.orientation.y
        oz = data.pose.orientation.z
        ow = data.pose.orientation.w
        ################## ForceTorque ######################
        fx = data.wrench.force.x
        fy = data.wrench.force.y
        fz = data.wrench.force.z
        tx = data.wrench.torque.x
        ty = data.wrench.torque.y
        tz = data.wrench.torque.z

        self.endeff_data = np.append(self.endeff_data, np.array([[sec, nsec, px, py, pz, ox, oy, oz, ow, fx, fy, fz, tx, ty, tz]]), axis=0)


    def stop_recording(self):
        self.pcf_sub.unregister()
        # self.ft_sub.unregister()


    def convertandsave(self,i):
        # convert to numpy array and save
        path = '/home/radhen/Documents/bebionic_expData/'

        # np.savetxt(path+'/pcf_data_{}.txt'.format(i), self.pcf_data)
        # np.savetxt(path+'/endeff_data_{}.txt'.format(i), self.endeff_data)

        np.savetxt(path + '/trial_{}.txt'.format(i), self.pcf_data)



if __name__ == "__main__":
    rospy.init_node('get_data')
    gd = GetData()
    # while 1:
    gd.start_recording()
    rospy.sleep(3)
    gd.stop_recording()
    gd.convertandsave(0)
    # rospy.spin()
