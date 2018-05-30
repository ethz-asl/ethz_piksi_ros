#!/usr/bin/env python

#
#  Title:        geodetic_survey.py
#  Description:  Mean a fixed number of GPS points to survey base station position.
#

import rospy
import roslib.packages
from piksi_rtk_msgs.srv import *
import std_srvs.srv
from sensor_msgs.msg import (NavSatFix, NavSatStatus)
import os
import time


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

        # Subscribe.
        rospy.Subscriber(self.navsatfix_topics_name, NavSatFix,
                         self.navsatfix_callback)

        rospy.spin()

    def navsatfix_callback(self, msg):
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
