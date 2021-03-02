#!/usr/bin/env python2 

import rospy
import math
import copy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
from tf import transformations
from sensor_msgs.msg import MagneticField


# Node that helps with GPS-MAG fusion
#   - Intermediate solution to be tested
#   - Subscribes to:
#       - attitude [type odometry]: Used to obtain roll and pitch to compensate mag
#       - magnetometer [type MagneticField]: Magnetometer data
#       - gps_in [type Odometry]: GPS position + covariance.
#
#   - Publishes
#       - gps_mag_out [type Oodmetry]: Position + Position covariance from GPS,
#                                       Orientation: Heading from Magnetometer
#
#   - Configurations:
#       - cov_orientation: Covariance of orientation estimate from Mag (forwarded to output)
#       - mag_orientation: Orientation of Mag measurement w.r.t. body
#       - mag_calib_aniv:  A_inverse part of magnetometer calibration
#       - mag_calib_b:      B part of magnetometer claibration
#
#       See calib_mag script for obtaining A_inverse and B.

class MagGpsFusionNode:

    def __init__(self):

        self._sub_att = rospy.Subscriber("attitude", Odometry, self.attitude_callback)
        self._sub_mag = rospy.Subscriber("mag", MagneticField, self.mag_callback)
        self._sub_gps = rospy.Subscriber("pose_enu_cov", PoseWithCovarianceStamped,
                                         self.gps_callback)

        self._pub = rospy.Publisher('gps_mag_out', Odometry, queue_size=10)

        # Todo: Factor out configuration to yaml file or rosparams.
        self._cov_orientation = np.diag([0.007607716, 0.007607716, 0.007607716])

        self._mag_orientation = np.eye(3)

        self._mag_calib_multiplier = 1E3

        self._mag_calib_Ainv = np.array([[5.38606762e+01, -7.14641776e-03, 1.71967955e+00],
                                         [-7.14641776e-03, 5.17309889e+01, 4.99221738e+00],
                                         [1.71967955e+00, 4.99221738e+00, 6.76031190e+01]])

        self._mag_calib_B = np.array([0.08355408,
                                      - 0.04023363,
                                      -0.04951364])

        self._q_odom = np.array([0, 0, 0, 1.0])
        self._last_heading = None
        self._last_q_heading = None

    def rectify(self, mag_data):
        mag_data *= self._mag_calib_multiplier
        # undo distortion
        mag_data_rect = self._mag_calib_Ainv.dot(mag_data - self._mag_calib_B)

        # rotate into body frame
        mag_data_rot = self._mag_orientation.dot(mag_data_rect)
        return mag_data_rot

        # returns 2d field strength in plane

    def compensate_roll_pitch(self, mag_data):
        rpy = transformations.euler_from_quaternion(self._q_odom)
        c_r = rpy[0]
        c_p = -rpy[1]
        c_y = rpy[2]
        cos_pitch = np.cos(c_p)
        sin_pitch = np.sin(c_p)
        cos_roll = np.cos(c_r)
        sin_roll = np.sin(c_r)
        # would be nicer in vector/matrix form, but it works like that
        t_magx = mag_data[0] * cos_pitch + mag_data[2] * sin_pitch
        t_magy = mag_data[0] * sin_roll * sin_pitch + mag_data[1] * cos_roll - mag_data[2] * sin_roll * cos_pitch

        return np.array([t_magx, t_magy])

    def gps_callback(self, gps_odom):
        if self._last_heading is None or self._last_q_heading is None:
            rospy.logwarn("No MAG data processed. Check topic mappings")
            return

        final_msg = Odometry()
        final_msg.header.stamp = gps_odom.header.stamp
        final_msg.header.frame_id = "enu"
        final_msg.pose.pose.position = gps_odom.pose.pose.position
        final_msg.pose.covariance = gps_odom.pose.covariance
        final_msg.pose.pose.orientation.x = self._last_q_heading[0]
        final_msg.pose.pose.orientation.y = self._last_q_heading[1]
        final_msg.pose.pose.orientation.z = self._last_q_heading[2]
        final_msg.pose.pose.orientation.w = self._last_q_heading[3]

        # add covariance
        full_cov = np.array(final_msg.pose.covariance).reshape([6, 6])
        full_cov[3:6, 3:6] = self._cov_orientation
        final_msg.pose.covariance = full_cov.flatten()

        print(np.diag(full_cov))
        print(full_cov[0:3, 0:3])
        print(full_cov[3:6, 3:6])

        self._pub.publish(final_msg)

    def mag_callback(self, msg):
        if self._q_odom is None:
            rospy.logwarn("No attitude received, ignoring mag callback")
            return

        raw_data = np.array([msg.magnetic_field.x,
                             msg.magnetic_field.y,
                             msg.magnetic_field.z])
        rect_data = self.rectify(raw_data)
        plane_data = self.compensate_roll_pitch(rect_data)

        # get heading and generate odometry message with that quaternion
        heading = np.arctan2(rect_data[0], rect_data[1])
        q_head = transformations.quaternion_from_euler(0, 0, heading, 'rxyz')

        self._last_heading = heading
        self._last_q_heading = q_head

    def attitude_callback(self, msg):
        q_odom = np.array([msg.pose.pose.orientation.x,
                           msg.pose.pose.orientation.y,
                           msg.pose.pose.orientation.z,
                           msg.pose.pose.orientation.w])
        self._q_odom = q_odom


if __name__ == "__main__":
    rospy.init_node('mag_fusion_node')
    node = MagGpsFusionNode()
    rospy.spin()
