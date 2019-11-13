#!/usr/bin/env python

#
#  Title:        geodetic_survey.py
#  Description:  Mean a fixed number of GPS points to survey base station position.
#

import rospy
import roslib.packages
from piksi_rtk_msgs.srv import *
from piksi_rtk_msgs.msg import PositionWithCovarianceStamped
import std_srvs.srv
from sensor_msgs.msg import (NavSatFix, NavSatStatus)
import os
import time
import numpy as np
import math
import pyproj


class GeodeticSurvey:
    kRosPackageName = "piksi_multi_rtk_ros"
    kServiceTimeOutSeconds = 10.0
    kWaitBetweenReadReqAndResSeconds = 1.0
    kRelativeTolleranceGeodeticComparison = 1e-10

    def __init__(self):
        rospy.init_node('geodetic_survey')
        rospy.loginfo(rospy.get_name() + " start")

        self.latitude_accumulator = 0.0
        self.longitude_accumulator = 0.0
        self.altitude_accumulator = 0.0
        self.number_of_fixes = 0
        self.surveyed_position_set = False

        # Kalman filter variables.
        self.x = np.array([0.0, 0.0, 0.0])
        self.P = np.zeros([3,3])
        self.x_init = False

        # Settings
        self.number_of_desired_fixes = rospy.get_param('~number_of_desired_fixes', 5000)
        self.navsatfix_topics_name = rospy.get_param('~navsatfix_topics_name', 'piksi_multi_base_station/navsatfix_spp')
        self.write_settings_service_name = rospy.get_param('~write_settings_service_name', 'piksi_multi_base_station/settings_write')
        self.save_settings_service_name = rospy.get_param('~save_settings_service_name', 'piksi_multi_base_station/settings_save')
        self.read_req_settings_service_name = rospy.get_param('~read_req_settings_service_name',
                                                              'piksi_multi_base_station/settings_read_req')
        self.read_resp_settings_service_name = rospy.get_param('~read_resp_settings_service_name',
                                                               'piksi_multi_base_station/settings_read_resp')
        self.height_base_station_from_ground = rospy.get_param('~height_base_station_from_ground', 0.0)
        self.use_covariance = rospy.get_param('~use_covariance', False)
        self.pos_ecef_cov_topics_name = rospy.get_param('~pos_ecef_cov_topics_name', 'piksi_multi_base_station/pos_ecef_cov')

        # Least squares variables.
        self.R_inv = np.zeros((3 * self.number_of_desired_fixes, 3 * self.number_of_desired_fixes))
        self.y = np.zeros(3 * self.number_of_desired_fixes)

        # Subscribe.
        rospy.Subscriber(self.navsatfix_topics_name, NavSatFix,
                         self.navsatfix_callback)
        rospy.Subscriber(self.pos_ecef_cov_topics_name, PositionWithCovarianceStamped, self.pos_ecef_cov_callback)

        rospy.spin()

    def pos_ecef_cov_callback(self, msg):
        if not self.use_covariance:
            return
        if self.number_of_fixes >= self.number_of_desired_fixes:
            return

        # Measurement.
        z = np.array([msg.position.position.x, msg.position.position.y, msg.position.position.z])
        R = np.array([[msg.position.covariance[0], msg.position.covariance[1], msg.position.covariance[2]],
                      [msg.position.covariance[3], msg.position.covariance[4], msg.position.covariance[5]],
                      [msg.position.covariance[6], msg.position.covariance[7], msg.position.covariance[8]]])

        i = self.number_of_fixes * 3
        self.R_inv[i:i+3, i:i+3] = np.linalg.inv(R)
        self.y[i:i+3] = z

        # Initialize x with current measurement.
        if self.x_init == False:
            self.x = z
            self.P = R
            self.x_init = True
        else:
            # Innovation.
            y = (z - self.x).transpose()
            S = self.P + R
            # Gain
            K = self.P.dot(np.linalg.inv(S))
            # Update
            self.x = self.x + K.dot(y)
            self.P = (np.identity(3) - K).dot(self.P)

        (P_eig_values, P_eig_vectors) = np.linalg.eig(self.P)
        (R_eig_values, R_eig_vectors) = np.linalg.eig(R)

        self.number_of_fixes += 1

        rospy.loginfo(
            "Received: [%.3f, %.3f, %.3f]; Measurement 3-sigma bound: [%.3f, %.3f, %.3f]; temporary mean: [%.3f, %.3f, %.3f]; temporary 3-sigma bound: [%.3f, %.3f, %.3f] waiting for %d samples" % (
                z[0], z[1], z[2],
                3 * math.sqrt(R_eig_values[0]), 3 * math.sqrt(R_eig_values[1]), 3 * math.sqrt(R_eig_values[2]),
                self.x[0], self.x[1], self.x[2],
                3 * math.sqrt(P_eig_values[0]), 3 * math.sqrt(P_eig_values[1]), 3 * math.sqrt(P_eig_values[2]),
                self.number_of_desired_fixes - self.number_of_fixes))

        if self.number_of_fixes >= self.number_of_desired_fixes and not self.surveyed_position_set:
            # Least squares estimation.
            H = np.tile(np.identity(3), (self.number_of_desired_fixes, 1))
            a = H.transpose().dot(self.R_inv.dot(H))
            b = H.transpose().dot(self.R_inv).dot(self.y)
            x = np.linalg.solve(a,b)

            P = np.linalg.inv(a)
            (P_eig_values, P_eig_vectors) = np.linalg.eig(P)

            rospy.loginfo(
                "ML estimate: [%.3f, %.3f, %.3f]; ML 3-sigma bound: [%.3f, %.3f, %.3f]" % (
                    x[0], x[1], x[2],
                    3 * math.sqrt(P_eig_values[0]), 3 * math.sqrt(P_eig_values[1]), 3 * math.sqrt(P_eig_values[2])))

            ecef = pyproj.Proj(proj='geocent', ellps='WGS84', datum='WGS84')
            lla = pyproj.Proj(proj='latlong', ellps='WGS84', datum='WGS84')
            lon0, lat0, alt0 = pyproj.transform(ecef, lla, x[0], x[1], x[2], radians=False)

            if self.set_base_station_position(lat0, lon0, alt0):
                self.surveyed_position_set = True
                rospy.loginfo("Base station position set correctly.")
                rospy.loginfo(
                    "Creating ENU frame on surveyed position and substructing specified height of base station.")
                self.log_enu_origin_position(lat0, lon0, alt0)
                rospy.signal_shutdown("Base station position set correctly. Stop this node and launch base station node.")
            else:
                rospy.logerr("Base station position not set correctly.")
                rospy.signal_shutdown("Base station position not set correctly.")


    def navsatfix_callback(self, msg):
        if self.use_covariance:
            return

        # Sanity check: we should have either SPP or SBAS fix.
        if msg.status.status != NavSatStatus.STATUS_FIX and msg.status.status != NavSatStatus.STATUS_SBAS_FIX:
            rospy.logerr(
                "[navsatfix_callback] received a navsatfix message with status '%d'." % (msg.status.status) +
                " Accepted status are 'STATUS_FIX' (%d) or 'STATUS_SBAS_FIX' (%d)" % (NavSatStatus.STATUS_FIX, NavSatStatus.STATUS_SBAS_FIX))
            return

        self.latitude_accumulator += msg.latitude
        self.longitude_accumulator += msg.longitude
        self.altitude_accumulator += msg.altitude
        self.number_of_fixes += 1

        rospy.loginfo(
            "Received: [%.10f, %.10f, %.1f]; temporary average: [%.10f, %.10f, %.1f]; waiting for %d samples" % (
                msg.latitude, msg.longitude, msg.altitude, self.latitude_accumulator / self.number_of_fixes,
                self.longitude_accumulator / self.number_of_fixes, self.altitude_accumulator / self.number_of_fixes,
                self.number_of_desired_fixes - self.number_of_fixes))

        if self.number_of_fixes >= self.number_of_desired_fixes and not self.surveyed_position_set:
            lat0 = self.latitude_accumulator / self.number_of_fixes
            lon0 = self.longitude_accumulator / self.number_of_fixes
            alt0 = self.altitude_accumulator / self.number_of_fixes

            if self.set_base_station_position(lat0, lon0, alt0):
                self.surveyed_position_set = True
                rospy.loginfo("Base station position set correctly.")
                rospy.loginfo(
                    "Creating ENU frame on surveyed position and substructing specified height of base station.")
                self.log_enu_origin_position(lat0, lon0, alt0)
                rospy.signal_shutdown("Base station position set correctly. Stop this node and launch base station node.")
            else:
                rospy.logerr("Base station position not set correctly.")
                rospy.signal_shutdown("Base station position not set correctly.")

    def set_base_station_position(self, lat0, lon0, alt0):
        everything_ok = []
        # Use only 2 digits for altitude as it is more than enough.
        alt0 = float("%.2f" % alt0)
        rospy.loginfo("Setting Piksi Multi surveyed position to: %.10f, %.10f, %.1f\n" % (lat0, lon0, alt0))

        # Write settings.
        write_lat0_ok = self.write_settings_to_piksi("surveyed_position", "surveyed_lat", "%.10f" % lat0)
        write_lon0_ok = self.write_settings_to_piksi("surveyed_position", "surveyed_lon", "%.10f" % lon0)
        write_alt0_ok = self.write_settings_to_piksi("surveyed_position", "surveyed_alt", "%.2f" % alt0)

        if write_lat0_ok and write_lon0_ok and write_alt0_ok:
            # Save and check what was actually written to flash
            settings_saved = self.save_settings_to_piksi()
            if settings_saved:
                read_lat0, read_lat0_value = self.read_settings_from_piksi("surveyed_position", "surveyed_lat")
                read_lon0, read_lon0_value = self.read_settings_from_piksi("surveyed_position", "surveyed_lon")
                read_alt0, read_alt0_value = self.read_settings_from_piksi("surveyed_position", "surveyed_alt")

                if read_lat0 and read_lon0 and read_alt0:
                    # Check read values == computed values
                    if self.is_close(float(read_lat0_value), lat0,
                                     rel_tol=self.kRelativeTolleranceGeodeticComparison) and self.is_close(
                        float(read_lon0_value),
                        lon0, rel_tol=self.kRelativeTolleranceGeodeticComparison) and self.is_close(
                        float(read_alt0_value), alt0, rel_tol=self.kRelativeTolleranceGeodeticComparison):
                        everything_ok = True
                        self.log_surveyed_position(lat0, lon0, alt0)

                        # Make sure Piksi is properly configured to act as base station.
                        read_surveyed_broadcast, read_surveyed_broadcast_value = self.read_settings_from_piksi(
                            "surveyed_position", "broadcast")

                        read_surveyed_broadcast_value = True if (read_surveyed_broadcast_value == "True") else False
                        if read_surveyed_broadcast_value:
                            rospy.loginfo(
                                "Your Piksi Multi is already configured to broadcast its surveyed position and act as base station.")
                        else:
                            rospy.logwarn(
                                "Your Piksi Multi is NOT configured to broadcast its surveyed position and act as base station. "
                                "Please use piksi console (by Siwftnav) to change settings. See Wiki page where you downloaded this ROS driver.")
                    else:
                        rospy.logwarn(
                            "Read values do NOT correspond to written ones. Please use piksi console (See swiftnav support).")
                        everything_ok = False
                else:
                    rospy.logerr("Error while saving base station position to Piksi flash.")
                    everything_ok = False
            else:
                rospy.logerr("Error while saving base station position to Piksi flash.")
                everything_ok = False
        else:
            rospy.logerr("Error while writing base station position to Piksi.")
            everything_ok = False

        return everything_ok

    def write_settings_to_piksi(self, section_setting, setting, value):
        rospy.wait_for_service(self.write_settings_service_name, timeout=GeodeticSurvey.kServiceTimeOutSeconds)
        write_settings_service = rospy.ServiceProxy(self.write_settings_service_name, SettingsWrite)
        try:
            rospy.loginfo("Setting %s.%s to %s" % (section_setting, setting, value))
            write_resp = write_settings_service(section_setting=section_setting, setting=setting, value=value)
            return write_resp.success
        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: " + str(exc))
            return False

    def save_settings_to_piksi(self):
        rospy.wait_for_service(self.save_settings_service_name, timeout=GeodeticSurvey.kServiceTimeOutSeconds)
        save_settings_service = rospy.ServiceProxy(self.save_settings_service_name, std_srvs.srv.SetBool)
        try:
            rospy.loginfo("Saving settings to Piksi Multi flash.")
            save_resp = save_settings_service(True)
            return save_resp.success
        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: " + str(exc))
            return False

    def read_settings_from_piksi(self, section_setting, setting):
        # Read request.
        rospy.wait_for_service(self.read_req_settings_service_name, timeout=GeodeticSurvey.kServiceTimeOutSeconds)
        read_req_settings_service = rospy.ServiceProxy(self.read_req_settings_service_name, SettingsReadReq)
        try:
            read_req_resp = read_req_settings_service(section_setting=section_setting, setting=setting)
        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: " + str(exc))
            return False, -1

        if read_req_resp.success:
            # Read req sent, wait before we read the response.
            time.sleep(GeodeticSurvey.kWaitBetweenReadReqAndResSeconds)
            # Read response.
            rospy.wait_for_service(self.read_resp_settings_service_name, timeout=GeodeticSurvey.kServiceTimeOutSeconds)
            read_resp_settings_service = rospy.ServiceProxy(self.read_resp_settings_service_name, SettingsReadResp)
            try:
                read_resp_resp = read_resp_settings_service()
            except rospy.ServiceException as exc:
                rospy.logerr("Service did not process request: " + str(exc))
                return False, -1

            if read_resp_resp.success:
                rospy.loginfo("Read [%s.%s: %s] from Piksi settings." % (
                    read_resp_resp.section_setting, read_resp_resp.setting, read_resp_resp.value))
                return True, read_resp_resp.value

        return False, -1

    def log_surveyed_position(self, lat0, lon0, alt0):
        now = time.strftime("%Y-%m-%d-%H-%M-%S")
        package_path = roslib.packages.get_pkg_dir(self.kRosPackageName)
        desired_path = "%s/log_surveys/base_station_survey_%s.txt" % (package_path, now)
        file_obj = open(desired_path, 'w')
        file_obj.write("# File automatically generated on %s\n\n" % now)
        file_obj.write("latitude0_deg: %.10f\n" % lat0)
        file_obj.write("longitude0_deg: %.10f\n" % lon0)
        file_obj.write("altitude0: %.2f\n" % alt0)
        file_obj.close()
        rospy.loginfo("Surveyed position saved in \'" + desired_path + "\'")

    def log_enu_origin_position(self, lat0, lon0, alt0):
        # current path of geodetic_survey.py file
        now = time.strftime("%Y-%m-%d-%H-%M-%S")
        package_path = roslib.packages.get_pkg_dir(self.kRosPackageName)
        desired_path = "%s/log_surveys/enu_origin_%s.txt" % (package_path, now)
        file_obj = open(desired_path, 'w')
        file_obj.write("# File automatically generated on %s\n" % now)
        file_obj.write(
            "# ENU altitude0 = surveyed_altitude0 - %.3f (=height_base_station_from_ground)\n\n" % self.height_base_station_from_ground)
        file_obj.write("latitude0_deg: %.10f\n" % lat0)
        file_obj.write("longitude0_deg: %.10f\n" % lon0)
        file_obj.write("altitude0: %.2f\n" % (alt0 - self.height_base_station_from_ground))
        file_obj.close()
        rospy.loginfo("ENU origin saved in \'" + desired_path + "\'")

    # https://www.python.org/dev/peps/pep-0485/#proposed-implementation
    def is_close(self, a, b, rel_tol=1e-09, abs_tol=0.0):
        return abs(a - b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)
