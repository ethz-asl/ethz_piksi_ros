#!/usr/bin/env python

#
#  Title:        piksi_multi.py
#  Description:  ROS Driver for Piksi Multi RTK GPS module
#  Dependencies: libsbp (https://github.com/swift-nav/libsbp), tested with version = see LIB_SBP_VERSION_MULTI
#  Based on original work of https://bitbucket.org/Daniel-Eckert/piksi_node
#

import rospy
import math
import quaternion
import numpy as np
import datetime, time, leapseconds
from collections import deque
import std_srvs.srv
# Import message types
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu, MagneticField
import piksi_rtk_msgs # TODO(rikba): If we dont have this I get NameError: global name 'piksi_rtk_msgs' is not defined.
from piksi_rtk_msgs.msg import (AgeOfCorrections, BaselineEcef, BaselineHeading, BaselineNed, BasePosEcef, BasePosLlh,
                                DeviceMonitor_V2_3_15, DopsMulti, ExtEvent, GpsTimeMulti, Heartbeat, ImuRawMulti,
                                InfoWifiCorrections, Log, MagRaw, MeasurementState_V2_4_1, Observation,
                                PositionWithCovarianceStamped, PosEcef, PosEcefCov, PosLlhCov, PosLlhMulti,
                                ReceiverState_V2_4_1, UartState_V2_3_15, UtcTimeMulti,
                                VelEcef, VelEcefCov, VelNed, VelNedCov, VelocityWithCovarianceStamped)
from piksi_rtk_msgs.srv import *
from geometry_msgs.msg import (PoseWithCovarianceStamped, PointStamped, PoseWithCovariance, Point, TransformStamped,
                               Transform)
from visualization_msgs.msg import Marker
# Import Piksi SBP library
from sbp.client.drivers.pyserial_driver import PySerialDriver
from sbp.client.drivers.network_drivers import TCPDriver
from sbp.client import Handler, Framer
from sbp.navigation import *
from sbp.logging import *
from sbp.system import *
from sbp.tracking import *  # WARNING: tracking is part of the draft messages, could be removed in future releases of libsbp.
from sbp.piksi import *  # WARNING: piksi is part of the draft messages, could be removed in future releases of libsbp.
from sbp.observation import *
from sbp.orientation import *  # WARNING: orientation messages are still draft messages.
from sbp.settings import *
from zope.interface.exceptions import Invalid
# Piksi Multi features an IMU
from sbp.imu import *
# Piksi Multi features a Magnetometer Bosh bmm150 : https://www.bosch-sensortec.com/bst/products/all_products/bmm150
from sbp.mag import SBP_MSG_MAG_RAW, MsgMagRaw
from sbp.ext_events import *
# At the moment importing 'sbp.version' module causes ValueError: Cannot find the version number!
# import sbp.version
# networking stuff
import UdpHelpers
import time
import subprocess
import re
import threading
import sys
import collections


class PiksiMulti:
    LIB_SBP_VERSION_MULTI = '2.6.5'  # SBP version used for Piksi Multi.

    # Geodetic Constants.
    kSemimajorAxis = 6378137.0
    kSemiminorAxis = 6356752.3142
    kFirstEccentricitySquared = 6.69437999014 * 0.001
    kSecondEccentricitySquared = 6.73949674228 * 0.001
    kFlattening = 1 / 298.257223563

    # IMU scaling constants.
    kSensorSensitivity = 1.0 / np.iinfo(np.int16).max
    kGravity = 9.81
    kDegToRad = np.pi / 180.0
    kFromMicro = 1.0 / 10**6
    kFromMilli = 1.0 / 10**3

    kAccPrescale = kGravity * kSensorSensitivity
    kGyroPrescale = kDegToRad * kSensorSensitivity
    kMagScaleXY = 1300.0 * kFromMicro * kSensorSensitivity
    kMagScaleZ = 2500.0 * kFromMicro * kSensorSensitivity

    def __init__(self):

        # Print info.
        rospy.init_node('piksi')
        rospy.sleep(0.5)  # Wait for a while for init to complete before printing.
        rospy.loginfo(rospy.get_name() + " start")

        # Check SBP version.
        if 'sbp.version' in sys.modules:
            installed_sbp_version = sbp.version.get_git_version()
        else:
            installed_sbp_version = self.get_installed_sbp_version()

        rospy.loginfo("libsbp version currently used: " + installed_sbp_version)

        # Check for correct SBP library version dependent on Piksi device.
        if PiksiMulti.LIB_SBP_VERSION_MULTI != installed_sbp_version:
            rospy.logwarn("Lib SBP version in usage (%s) is different than the one used to test this driver (%s)!\n"
                          "Please run the install script: 'install/install_piksi_multi.sh'" % (
                              installed_sbp_version, PiksiMulti.LIB_SBP_VERSION_MULTI))

        # Open a connection to SwiftNav receiver.
        interface = rospy.get_param('~interface', 'serial')

        if interface == 'tcp':
            tcp_addr = rospy.get_param('~tcp_addr', '192.168.0.222')
            tcp_port = rospy.get_param('~tcp_port', 55555)
            try:
                self.driver = TCPDriver(tcp_addr, tcp_port)
            except SystemExit:
                rospy.logerr("Unable to open TCP connection %s:%s", (tcp_addr, tcp_port))
                raise
        else:
            serial_port = rospy.get_param('~serial_port', '/dev/ttyUSB0')
            baud_rate = rospy.get_param('~baud_rate', 230400)
            try:
                self.driver = PySerialDriver(serial_port, baud=baud_rate)
            except SystemExit:
                rospy.logerr("Swift receiver not found on serial port '%s'", serial_port)
                raise

        # Create a handler to connect Piksi driver to callbacks.
        self.driver_verbose = rospy.get_param('~driver_verbose', True)
        self.framer = Framer(self.driver.read, self.driver.write, verbose=self.driver_verbose)
        self.handler = Handler(self.framer)

        self.debug_mode = rospy.get_param('~debug_mode', False)
        if self.debug_mode:
            rospy.loginfo("Swift driver started in debug mode, every available topic will be published.")
            # Debugging parameters.
            debug_delayed_corrections_stack_size = rospy.get_param('~debug_delayed_corrections_stack_size', 10)
            # self.received_corrections_fifo_stack = collections.deque([], debug_delayed_corrections_stack_size)
            # rospy.loginfo("Debug mode: delayed corrections stack size: %d." % debug_delayed_corrections_stack_size)
        else:
            rospy.loginfo("Swift driver started in normal mode.")

        # Corrections over WiFi settings.
        self.base_station_mode = rospy.get_param('~base_station_mode', False)
        self.udp_broadcast_addr = rospy.get_param('~broadcast_addr', '255.255.255.255')
        self.udp_port = rospy.get_param('~broadcast_port', 26078)
        self.base_station_ip_for_latency_estimation = rospy.get_param(
            '~base_station_ip_for_latency_estimation',
            '192.168.0.1')
        self.multicaster = []
        self.multicast_recv = []

        # Navsatfix settings.
        self.var_spp = rospy.get_param('~var_spp', [25.0, 25.0, 64.0])
        self.var_rtk_float = rospy.get_param('~var_rtk_float', [25.0, 25.0, 64.0])
        self.var_rtk_fix = rospy.get_param('~var_rtk_fix', [0.0049, 0.0049, 0.01])
        self.var_spp_sbas = rospy.get_param('~var_spp_sbas', [1.0, 1.0, 1.0])
        self.navsatfix_frame_id = rospy.get_param('~navsatfix_frame_id', 'gps')

        # Covariance topic settings.
        self.publish_covariances = rospy.get_param('~publish_covariances', False)
        self.llh_frame = rospy.get_param('~llh_frame', 'wgs84')
        self.ecef_frame = rospy.get_param('~ecef_frame', 'ecef')
        self.antenna_ned_frame = rospy.get_param('~antenna_ned_frame', 'antenna_ned')
        self.base_ned_frame = rospy.get_param('~base_ned_frame', 'base_ned')

        # Local ENU frame settings.
        self.origin_enu_set = False
        self.latitude0 = 0.0
        self.longitude0 = 0.0
        self.altitude0 = 0.0
        self.initial_ecef_x = 0.0
        self.initial_ecef_y = 0.0
        self.initial_ecef_z = 0.0
        self.ecef_to_ned_matrix = np.eye(3)
        self.enu_frame_id = rospy.get_param('~enu_frame_id', 'enu')
        self.transform_child_frame_id = rospy.get_param('~transform_child_frame_id', 'gps_receiver')

        if rospy.has_param('~latitude0_deg') and rospy.has_param('~longitude0_deg') and rospy.has_param(
                '~altitude0'):
            latitude0 = rospy.get_param('~latitude0_deg')
            longitude0 = rospy.get_param('~longitude0_deg')
            altitude0 = rospy.get_param('~altitude0')

            # Set origin ENU frame to coordinate specified by rosparam.
            self.init_geodetic_reference(latitude0, longitude0, altitude0)
            rospy.loginfo("Origin ENU frame set by rosparam.")

        # Watchdog timer info
        self.watchdog_time = rospy.get_rostime()
        self.messages_started = False

        # Other parameters.
        self.publish_raw_imu_and_mag = rospy.get_param('~publish_raw_imu_and_mag', False)
        # Publish IMU
        self.acc_scale = 8 * PiksiMulti.kAccPrescale
        self.gyro_scale = 125 * PiksiMulti.kGyroPrescale
        self.has_imu_scale = False


        # Advertise topics and services.
        self.publishers = self.advertise_topics()
        self.service_servers = self.advertise_services()

        # Create topic callbacks.
        self.create_topic_callbacks()

        # Init messages with "memory".
        self.receiver_state_msg = self.init_receiver_state_msg()
        self.num_wifi_corrections = self.init_num_corrections_msg()

        # Corrections over wifi message, if we are not the base station.
        if not self.base_station_mode:
            # Start new thread to periodically ping base station.
            threading.Thread(target=self.ping_base_station_over_wifi).start()

        # Handle firmware settings services
        self.last_section_setting_read = []
        self.last_setting_read = []
        self.last_value_read = []

        # Only have start-up reset in base station mode
        if self.base_station_mode:
            # Things have 30 seconds to start or we will kill node
            rospy.Timer(rospy.Duration(30), self.cb_watchdog, True)

        # Buffer UTC times. key: tow, value: UTC
        self.use_gps_time = rospy.get_param('~use_gps_time', False)
        self.utc_times = {}
        self.tow = deque()

        self.handler.start()

        # Spin.
        rospy.spin()

    def create_topic_callbacks(self):
        # Callbacks from SBP messages (cb_sbp_*) implemented "manually".
        self.handler.add_callback(self.cb_sbp_glonass_biases, msg_type=SBP_MSG_GLO_BIASES)
        self.handler.add_callback(self.cb_sbp_heartbeat, msg_type=SBP_MSG_HEARTBEAT)
        self.handler.add_callback(self.cb_sbp_pos_llh, msg_type=SBP_MSG_POS_LLH)
        self.handler.add_callback(self.cb_sbp_base_pos_ecef, msg_type=SBP_MSG_BASE_POS_ECEF)
        self.handler.add_callback(self.cb_sbp_obs, msg_type=SBP_MSG_OBS)
        self.handler.add_callback(self.cb_sbp_settings_read_by_index_resp, msg_type=SBP_MSG_SETTINGS_READ_BY_INDEX_RESP)
        self.handler.add_callback(self.cb_settings_read_resp, msg_type=SBP_MSG_SETTINGS_READ_RESP)
        self.handler.add_callback(self.cb_sbp_measurement_state, msg_type=SBP_MSG_MEASUREMENT_STATE)
        self.handler.add_callback(self.cb_sbp_uart_state, msg_type=SBP_MSG_UART_STATE)
        self.handler.add_callback(self.cb_sbp_utc_time, msg_type=SBP_MSG_UTC_TIME)
        self.handler.add_callback(self.cb_sbp_ext_event, msg_type=SBP_MSG_EXT_EVENT)

        # Callbacks generated "automatically".
        self.init_callback('baseline_ecef_multi', BaselineEcef,
                           SBP_MSG_BASELINE_ECEF, MsgBaselineECEF,
                           'tow', 'x', 'y', 'z', 'accuracy', 'n_sats', 'flags')
        self.init_callback('baseline_ned_multi', BaselineNed,
                           SBP_MSG_BASELINE_NED, MsgBaselineNED,
                           'tow', 'n', 'e', 'd', 'h_accuracy', 'v_accuracy', 'n_sats', 'flags')
        self.init_callback('dops_multi', DopsMulti,
                           SBP_MSG_DOPS, MsgDops, 'tow', 'gdop', 'pdop', 'tdop', 'hdop', 'vdop', 'flags')
        self.init_callback('gps_time_multi', GpsTimeMulti,
                           SBP_MSG_GPS_TIME, MsgGPSTime, 'wn', 'tow', 'ns_residual', 'flags')
        self.init_callback('utc_time_multi', UtcTimeMulti,
                           SBP_MSG_UTC_TIME, MsgUtcTime,
                           'flags', 'tow', 'year', 'month', 'day', 'hours', 'minutes', 'seconds', 'ns')
        self.init_callback('pos_ecef_multi', PosEcef,
                           SBP_MSG_POS_ECEF, MsgPosECEF,
                           'tow', 'x', 'y', 'z', 'accuracy', 'n_sats', 'flags')
        self.init_callback('vel_ecef', VelEcef,
                           SBP_MSG_VEL_ECEF, MsgVelECEF,
                           'tow', 'x', 'y', 'z', 'accuracy', 'n_sats', 'flags')
        self.init_callback('vel_ned', VelNed,
                           SBP_MSG_VEL_NED, MsgVelNED,
                           'tow', 'n', 'e', 'd', 'h_accuracy', 'v_accuracy', 'n_sats', 'flags')
        self.init_callback('log', Log,
                           SBP_MSG_LOG, MsgLog, 'level', 'text')
        self.init_callback('baseline_heading', BaselineHeading,
                           SBP_MSG_BASELINE_HEADING, MsgBaselineHeading, 'tow', 'heading', 'n_sats', 'flags')
        self.init_callback('age_of_corrections', AgeOfCorrections,
                           SBP_MSG_AGE_CORRECTIONS, MsgAgeCorrections, 'tow', 'age')
        self.init_callback('device_monitor', DeviceMonitor_V2_3_15,
                           SBP_MSG_DEVICE_MONITOR, MsgDeviceMonitor, 'dev_vin', 'cpu_vint', 'cpu_vaux',
                           'cpu_temperature', 'fe_temperature')

        if self.publish_covariances:
            self.handler.add_callback(self.cb_sbp_pos_llh_cov, msg_type=SBP_MSG_POS_LLH_COV)
            self.handler.add_callback(self.cb_sbp_pos_ecef_cov, msg_type=SBP_MSG_POS_ECEF_COV)
            self.handler.add_callback(self.cb_sbp_vel_ned_cov, msg_type=SBP_MSG_VEL_NED_COV)
            self.handler.add_callback(self.cb_sbp_vel_ecef_cov, msg_type=SBP_MSG_VEL_ECEF_COV)
            self.handler.add_callback(self.cb_sbp_baseline_ned_cov, msg_type=SBP_MSG_BASELINE_NED)

        # Raw IMU and Magnetometer measurements.
        if self.publish_raw_imu_and_mag:
            self.init_callback('imu_raw', ImuRawMulti,
                               SBP_MSG_IMU_RAW, MsgImuRaw,
                               'tow', 'tow_f', 'acc_x', 'acc_y', 'acc_z', 'gyr_x', 'gyr_y', 'gyr_z')
            self.handler.add_callback(self.cb_sbp_imu_raw, msg_type=SBP_MSG_IMU_RAW)
            self.handler.add_callback(self.cb_sbp_imu_aux, msg_type=SBP_MSG_IMU_AUX)
            self.init_callback('mag_raw', MagRaw,
                               SBP_MSG_MAG_RAW, MsgMagRaw, 'tow', 'tow_f', 'mag_x', 'mag_y', 'mag_z')
            self.handler.add_callback(self.cb_sbp_mag_raw, msg_type=SBP_MSG_MAG_RAW)

        # Only if debug mode
        if self.debug_mode:
            self.handler.add_callback(self.cb_sbp_base_pos_llh, msg_type=SBP_MSG_BASE_POS_LLH)

        # do not publish llh message, prefer publishing directly navsatfix_spp or navsatfix_rtk_fix.
        # self.init_callback('pos_llh', PosLlh,
        #                   SBP_MSG_POS_LLH, MsgPosLLH,
        #                   'tow', 'lat', 'lon', 'height', 'h_accuracy', 'v_accuracy', 'n_sats', 'flags')

        # Relay "corrections" messages via UDP if in base station mode.
        if self.base_station_mode:
            rospy.loginfo("Starting device in Base Station Mode")
            self.multicaster = UdpHelpers.SbpUdpMulticaster(self.udp_broadcast_addr, self.udp_port)
        else:
            rospy.loginfo("Starting device in Rover Mode")
            self.multicast_recv = UdpHelpers.SbpUdpMulticastReceiver(self.udp_port, self.multicast_callback)

    def init_num_corrections_msg(self):
        num_wifi_corrections = InfoWifiCorrections()
        num_wifi_corrections.header.seq = 0
        num_wifi_corrections.received_corrections = 0
        num_wifi_corrections.latency = -1

        return num_wifi_corrections

    def init_receiver_state_msg(self):
        receiver_state_msg = ReceiverState_V2_4_1()
        receiver_state_msg.num_sat = 0  # Unknown.
        receiver_state_msg.rtk_mode_fix = False  # Unknown.
        receiver_state_msg.sat = []  # Unknown.
        receiver_state_msg.cn0 = []  # Unknown.
        receiver_state_msg.system_error = 255  # Unknown.
        receiver_state_msg.io_error = 255  # Unknown.
        receiver_state_msg.swift_nap_error = 255  # Unknown.
        receiver_state_msg.external_antenna_present = 255  # Unknown.
        receiver_state_msg.num_gps_sat = 0  # Unknown.
        receiver_state_msg.cn0_gps = []  # Unknown.
        receiver_state_msg.num_sbas_sat = 0  # Unknown.
        receiver_state_msg.cn0_sbas = []  # Unknown.
        receiver_state_msg.num_glonass_sat = 0  # Unknown.
        receiver_state_msg.cn0_glonass = []  # Unknown.
        receiver_state_msg.fix_mode = ReceiverState_V2_4_1.STR_FIX_MODE_UNKNOWN

        return receiver_state_msg

    def advertise_topics(self):
        """
        Advertise topics.
        :return: python dictionary, with topic names used as keys and publishers as values.
        """
        publishers = {}

        # Topics with covariances.
        if self.publish_covariances:
            publishers['pos_llh_cov'] = rospy.Publisher(rospy.get_name() + '/pos_llh_cov', NavSatFix, queue_size=10)
            publishers['pos_ecef_cov'] = rospy.Publisher(rospy.get_name() + '/pos_ecef_cov', PositionWithCovarianceStamped, queue_size=10)
            publishers['vel_ned_cov'] = rospy.Publisher(rospy.get_name() + '/vel_ned_cov', VelocityWithCovarianceStamped, queue_size=10)
            publishers['vel_ecef_cov'] = rospy.Publisher(rospy.get_name() + '/vel_ecef_cov', VelocityWithCovarianceStamped, queue_size=10)
            publishers['baseline_ned_cov'] = rospy.Publisher(rospy.get_name() + '/baseline_ned_cov', PositionWithCovarianceStamped, queue_size=10)

            publishers['pos_ecef_cov_viz'] = rospy.Publisher(rospy.get_name() + '/pos_ecef_cov_viz', Marker, queue_size=10)
            publishers['baseline_ned_cov_viz'] = rospy.Publisher(rospy.get_name() + '/baseline_ned_cov_viz', Marker, queue_size=10)

        publishers['rtk_fix'] = rospy.Publisher(rospy.get_name() + '/navsatfix_rtk_fix',
                                                NavSatFix, queue_size=10)
        publishers['rtk_float'] = rospy.Publisher(rospy.get_name() + '/navsatfix_rtk_float',
                                                NavSatFix, queue_size=10)
        publishers['spp'] = rospy.Publisher(rospy.get_name() + '/navsatfix_spp',
                                            NavSatFix, queue_size=10)
        publishers['best_fix'] = rospy.Publisher(rospy.get_name() + '/navsatfix_best_fix',
                                                 NavSatFix, queue_size=10)
        publishers['heartbeat'] = rospy.Publisher(rospy.get_name() + '/heartbeat',
                                                  Heartbeat, queue_size=10)
        publishers['measurement_state'] = rospy.Publisher(rospy.get_name() + '/measurement_state',
                                                       piksi_rtk_msgs.msg.MeasurementState_V2_4_1, queue_size=10)
        publishers['receiver_state'] = rospy.Publisher(rospy.get_name() + '/debug/receiver_state',
                                                       ReceiverState_V2_4_1, queue_size=10)
        publishers['ext_event'] = rospy.Publisher(rospy.get_name() + '/ext_event', ExtEvent, queue_size=10)
        # Do not publish llh message, prefer publishing directly navsatfix_spp or navsatfix_rtk_fix.
        # publishers['pos_llh'] = rospy.Publisher(rospy.get_name() + '/pos_llh',
        #                                        PosLlh, queue_size=10)
        publishers['vel_ned'] = rospy.Publisher(rospy.get_name() + '/vel_ned',
                                                VelNed, queue_size=10)
        publishers['log'] = rospy.Publisher(rospy.get_name() + '/log',
                                            Log, queue_size=10)
        publishers['uart_state'] = rospy.Publisher(rospy.get_name() + '/uart_state',
                                                   UartState_V2_3_15, queue_size=10)
        publishers['device_monitor'] = rospy.Publisher(rospy.get_name() + '/device_monitor',
                                                       DeviceMonitor_V2_3_15, queue_size=10)
        # Points in ENU frame.
        publishers['enu_pose_fix'] = rospy.Publisher(rospy.get_name() + '/enu_pose_fix',
                                                     PoseWithCovarianceStamped, queue_size=10)
        publishers['enu_point_fix'] = rospy.Publisher(rospy.get_name() + '/enu_point_fix',
                                                      PointStamped, queue_size=10)
        publishers['enu_transform_fix'] = rospy.Publisher(rospy.get_name() + '/enu_transform_fix',
                                                          TransformStamped, queue_size=10)
        publishers['enu_pose_spp'] = rospy.Publisher(rospy.get_name() + '/enu_pose_spp',
                                                     PoseWithCovarianceStamped, queue_size=10)
        publishers['enu_point_spp'] = rospy.Publisher(rospy.get_name() + '/enu_point_spp',
                                                      PointStamped, queue_size=10)
        publishers['enu_transform_spp'] = rospy.Publisher(rospy.get_name() + '/enu_transform_spp',
                                                          TransformStamped, queue_size=10)
        publishers['gps_time_multi'] = rospy.Publisher(rospy.get_name() + '/gps_time',
                                                       GpsTimeMulti, queue_size=10)
        publishers['baseline_ned_multi'] = rospy.Publisher(rospy.get_name() + '/baseline_ned',
                                                           BaselineNed, queue_size=10)
        publishers['utc_time_multi'] = rospy.Publisher(rospy.get_name() + '/utc_time',
                                                       UtcTimeMulti, queue_size=10)
        publishers['baseline_heading'] = rospy.Publisher(rospy.get_name() + '/baseline_heading',
                                                         BaselineHeading, queue_size=10)
        publishers['age_of_corrections'] = rospy.Publisher(rospy.get_name() + '/age_of_corrections',
                                                           AgeOfCorrections, queue_size=10)
        publishers['enu_pose_best_fix'] = rospy.Publisher(rospy.get_name() + '/enu_pose_best_fix',
                                                          PoseWithCovarianceStamped, queue_size=10)
        publishers['enu_pose_float'] = rospy.Publisher(rospy.get_name() + '/enu_pose_float',
                                                       PoseWithCovarianceStamped, queue_size=10)
        publishers['enu_point_float'] = rospy.Publisher(rospy.get_name() + '/enu_point_float',
                                                        PointStamped, queue_size=10)
        publishers['enu_transform_float'] = rospy.Publisher(rospy.get_name() + '/enu_transform_float',
                                                            TransformStamped, queue_size=10)


        # Raw IMU and Magnetometer measurements.
        if self.publish_raw_imu_and_mag:
            publishers['imu_raw'] = rospy.Publisher(rospy.get_name() + '/imu_raw',
                                                    ImuRawMulti, queue_size=10)
            publishers['mag_raw'] = rospy.Publisher(rospy.get_name() + '/mag_raw',
                                                    MagRaw, queue_size=10)
            publishers['imu'] = rospy.Publisher(rospy.get_name() + '/imu',
                                                    Imu, queue_size=10)
            publishers['mag'] = rospy.Publisher(rospy.get_name() + '/mag',
                                                    MagneticField, queue_size=10)

        # Topics published only if in "debug mode".
        if self.debug_mode:
            publishers['vel_ecef'] = rospy.Publisher(rospy.get_name() + '/vel_ecef',
                                                     VelEcef, queue_size=10)
            publishers['baseline_ecef_multi'] = rospy.Publisher(rospy.get_name() + '/baseline_ecef',
                                                                BaselineEcef, queue_size=10)
            publishers['dops_multi'] = rospy.Publisher(rospy.get_name() + '/dops',
                                                       DopsMulti, queue_size=10)
            publishers['pos_ecef_multi'] = rospy.Publisher(rospy.get_name() + '/pos_ecef',
                                                           PosEcef, queue_size=10)
            publishers['observation'] = rospy.Publisher(rospy.get_name() + '/observation',
                                                        Observation, queue_size=10)
            publishers['base_pos_llh'] = rospy.Publisher(rospy.get_name() + '/base_pos_llh',
                                                         BasePosLlh, queue_size=10)
            publishers['base_pos_ecef'] = rospy.Publisher(rospy.get_name() + '/base_pos_ecef',
                                                          BasePosEcef, queue_size=10)
        if not self.base_station_mode:
            publishers['wifi_corrections'] = rospy.Publisher(rospy.get_name() + '/debug/wifi_corrections',
                                                             InfoWifiCorrections, queue_size=10)

        return publishers

    def advertise_services(self):
        """
        Advertise service servers.
        :return: python dictionary, with service names used as keys and servers as values.
        """
        servers = {}

        servers['reset_piksi'] = rospy.Service(rospy.get_name() +
                                               '/reset_piksi',
                                               std_srvs.srv.SetBool,
                                               self.reset_piksi_service_callback)

        servers['settings_write'] = rospy.Service(rospy.get_name() +
                                                  '/settings_write',
                                                  SettingsWrite,
                                                  self.settings_write_server)

        servers['settings_read_req'] = rospy.Service(rospy.get_name() +
                                                     '/settings_read_req',
                                                     SettingsReadReq,
                                                     self.settings_read_req_server)

        servers['settings_read_resp'] = rospy.Service(rospy.get_name() +
                                                      '/settings_read_resp',
                                                      SettingsReadResp,
                                                      self.settings_read_resp_server)

        servers['settings_save'] = rospy.Service(rospy.get_name() +
                                                 '/settings_save',
                                                 std_srvs.srv.SetBool,
                                                 self.settings_save_callback)

        return servers

    def ping_base_station_over_wifi(self):
        """
        Ping base station periodically without blocking the driver.
        """
        ping_deadline_seconds = 3
        interval_between_pings_seconds = 5

        while not rospy.is_shutdown():
            # Send ping command.
            command = ["ping",
                       "-w", str(ping_deadline_seconds),  # deadline before stopping attempt
                       "-c", "1",  # number of pings to send
                       self.base_station_ip_for_latency_estimation]
            ping = subprocess.Popen(command, stdout=subprocess.PIPE)

            out, error = ping.communicate()
            # Search for 'min/avg/max/mdev' round trip delay time (rtt) numbers.
            matcher = re.compile("(\d+.\d+)/(\d+.\d+)/(\d+.\d+)/(\d+.\d+)")

            if matcher.search(out) == None:
                # No ping response within ping_deadline_seconds.
                # In python write and read operations on built-in type are atomic,
                # there's no need to use mutex.
                self.num_wifi_corrections.latency = -1
            else:
                groups_rtt = matcher.search(out).groups()
                avg_rtt = groups_rtt[1]
                # In python write and read operations on built-in type are atomic,
                # there's no need to use mutex.
                self.num_wifi_corrections.latency = float(avg_rtt)

            time.sleep(interval_between_pings_seconds)

    def make_callback(self, sbp_type, ros_message, pub, attrs):
        """
        Dynamic generator for callback functions for message types from
        the SBP library.
        Inputs: 'sbp_type' name of SBP message type.
                'ros_message' ROS message type with SBP format.
                'pub' ROS publisher for ros_message.
                'attrs' array of attributes in SBP/ROS message.
        Returns: callback function 'callback'.
        """

        def callback(msg, **metadata):
            if pub.get_num_connections() == 0:
                return
            sbp_message = sbp_type(msg)
            ros_message.header.stamp = rospy.Time.now()
            for attr in attrs:
                if attr == 'flags':
                    # Least significat three bits of flags indicate status.
                    if (sbp_message.flags & 0x07) == 0:
                        return  # Invalid message, do not publish it.

                setattr(ros_message, attr, getattr(sbp_message, attr))
            pub.publish(ros_message)

        return callback

    def init_callback(self, topic_name, ros_datatype, sbp_msg_type, callback_data_type, *attrs):
        """
        Initializes the callback function  for an SBP
        message type.
        Inputs: 'topic_name' name of ROS topic for publisher
                'ros_datatype' ROS custom message type
                'sbp_msg_type' name of SBP message type for callback function
                'callback_data_type' name of SBP message type for SBP library
                '*attrs' array of attributes in ROS/SBP message
        """
        # Check that required topic has been advertised.
        if topic_name in self.publishers:
            ros_message = ros_datatype()

            # Add callback function.
            pub = self.publishers[topic_name]
            callback_function = self.make_callback(callback_data_type, ros_message, pub, attrs)
            self.handler.add_callback(callback_function, msg_type=sbp_msg_type)

    def cb_sbp_obs(self, msg_raw, **metadata):
        if self.debug_mode and self.publishers['observation'].get_num_connections() > 0:
            msg = MsgObs(msg_raw)

            obs_msg = Observation()

            obs_msg.header.stamp = rospy.Time.now()
            if (self.use_gps_time):
                obs_msg.header.stamp = self.gps_time_to_utc(msg.header.t.wn, msg.header.t.tow, msg.header.t.ns_residual)

            obs_msg.tow = msg.header.t.tow
            obs_msg.ns_residual = msg.header.t.ns_residual
            obs_msg.wn = msg.header.t.wn
            obs_msg.n_obs = msg.header.n_obs

            obs_msg.P = []
            obs_msg.L_i = []
            obs_msg.L_f = []
            obs_msg.D_i = []
            obs_msg.D_f = []
            obs_msg.cn0 = []
            obs_msg.lock = []
            obs_msg.flags = []
            obs_msg.sid_sat = []
            obs_msg.sid_code = []

            for observation in msg.obs:
                obs_msg.P.append(observation.P)
                obs_msg.L_i.append(observation.L.i)
                obs_msg.L_f.append(observation.L.f)
                obs_msg.D_i.append(observation.D.i)
                obs_msg.D_f.append(observation.D.f)
                obs_msg.cn0.append(observation.cn0)
                obs_msg.lock.append(observation.lock)
                obs_msg.flags.append(observation.flags)
                obs_msg.sid_sat.append(observation.sid.sat)
                obs_msg.sid_code.append(observation.sid.code)

            self.publishers['observation'].publish(obs_msg)

        if self.base_station_mode:
            self.multicaster.sendSbpPacket(msg_raw)

    def cb_sbp_base_pos_llh(self, msg_raw, **metadata):
        if self.debug_mode:
            msg = MsgBasePosLLH(msg_raw)

            pose_llh_msg = BasePosLlh()
            pose_llh_msg.header.stamp = rospy.Time.now()

            pose_llh_msg.lat = msg.lat
            pose_llh_msg.lon = msg.lon
            pose_llh_msg.height = msg.height

            self.publishers['base_pos_llh'].publish(pose_llh_msg)

    def cb_sbp_base_pos_ecef(self, msg_raw, **metadata):
        if self.debug_mode and self.publishers['base_pos_ecef'].get_num_connections() > 0:
            msg = MsgBasePosECEF(msg_raw)

            pose_ecef_msg = BasePosEcef()
            pose_ecef_msg.header.stamp = rospy.Time.now()

            pose_ecef_msg.x = msg.x
            pose_ecef_msg.y = msg.y
            pose_ecef_msg.z = msg.z

            self.publishers['base_pos_ecef'].publish(pose_ecef_msg)

        if self.base_station_mode:
            self.multicaster.sendSbpPacket(msg_raw)

    def cb_sbp_uart_state(self, msg_raw, **metadata):
        if self.publishers['uart_state'].get_num_connections() == 0:
            return
        msg = MsgUartState(msg_raw)
        uart_state_msg = UartState_V2_3_15()

        uart_state_msg.latency_avg = msg.latency.avg
        uart_state_msg.latency_lmin = msg.latency.lmin
        uart_state_msg.latency_lmax = msg.latency.lmax
        uart_state_msg.latency_current = msg.latency.current
        uart_state_msg.obs_period_avg = msg.obs_period.avg
        uart_state_msg.obs_period_pmin = msg.obs_period.pmin
        uart_state_msg.obs_period_pmax = msg.obs_period.pmax
        uart_state_msg.obs_period_current = msg.obs_period.current

        self.publishers['uart_state'].publish(uart_state_msg)

    def cb_sbp_utc_time(self, msg_raw, **metadata):
        if not self.use_gps_time:
            return
        msg = MsgUtcTime(msg_raw)

        if msg.flags == 0:
            rospy.logwarn("GPS time invalid.")
            return

        t = datetime.datetime(msg.year, msg.month, msg.day, msg.hours, msg.minutes, msg.seconds)
        secs = (t - datetime.datetime(1970,1,1)).total_seconds()
        self.utc_times[msg.tow] = rospy.Time(int(secs), msg.ns)
        self.tow.append(msg.tow)

        if len(self.tow) > 100:
            # Start removing samples
            self.utc_times.pop(self.tow.popleft())

    def cb_sbp_ext_event(self, msg_raw, **metadata):
        if self.publishers['ext_event'].get_num_connections() == 0:
            return

        msg = MsgExtEvent(msg_raw)

        ext_event_msg = ExtEvent()
        ext_event_msg.stamp.data = self.gps_time_to_utc(msg.wn, msg.tow, msg.ns_residual)
        ext_event_msg.pin_value = msg.flags & 0b01
        ext_event_msg.quality = msg.flags & 0b10
        ext_event_msg.pin = msg.pin

        self.publishers['ext_event'].publish(ext_event_msg)

    def multicast_callback(self, msg, **metadata):
        if self.framer:

            # TODO (marco-tranzatto) probably next commented part should be completely deleted.
            # if self.debug_mode:
            #     # Test network delay by storing a fixed number of correction messages and retrieving the oldest one.
            #     # TODO (marco-tranzatto) check if we need to store even **metadata or not
            #     # self.received_corrections_fifo_stack.append([msg, **metadata])
            #     # oldest_correction = self.received_corrections_fifo_stack.popleft()
            #     self.received_corrections_fifo_stack.append(msg)
            #     oldest_correction = self.received_corrections_fifo_stack.popleft()
            #     self.framer(oldest_correction, **metadata)
            # else:
            #     self.framer(msg, **metadata)
            self.framer(msg, **metadata)

            # Publish debug message about wifi corrections, if enabled.
            self.num_wifi_corrections.header.seq += 1
            self.num_wifi_corrections.header.stamp = rospy.Time.now()
            self.num_wifi_corrections.received_corrections += 1
            if not self.base_station_mode and self.publishers['wifi_corrections'].get_num_connections() > 0:
                self.publishers['wifi_corrections'].publish(self.num_wifi_corrections)

        else:
            rospy.logwarn("Received external SBP msg, but Piksi not connected.")

    def cb_sbp_glonass_biases(self, msg_raw, **metadata):
        if self.base_station_mode:
            self.multicaster.sendSbpPacket(msg_raw)

    def cb_watchdog(self, event):
        if ((rospy.get_rostime() - self.watchdog_time).to_sec() > 10.0):
            rospy.logwarn("Heartbeat failed, watchdog triggered.")

            if self.base_station_mode:
                rospy.signal_shutdown("Watchdog triggered, was gps disconnected?")

    def gps_time_to_utc(self, wn, tow, ns_residual):
        epoch = datetime.datetime(1980,01,06)
        secs, msecs = divmod(tow, 1000)
        usec = ns_residual / 1000.0 # TODO(rikba): Handle nanoseconds.
        elapsed = datetime.timedelta(seconds=secs, microseconds=usec, milliseconds=msecs, weeks=wn)
        t_utc = leapseconds.gps_to_utc(epoch + elapsed)

        secs_utc = int((t_utc - datetime.datetime(1970,1,1)).total_seconds())
        nsecs_utc = (t_utc - datetime.datetime(1970,1,1)).microseconds * 10**3

        return rospy.Time(secs_utc, nsecs_utc)

    def tow_f_to_utc(self, tow, tow_f):
        epoch = datetime.date(1980, 1, 6)
        today = datetime.date.today()

        epoch_sunday = epoch - datetime.timedelta((epoch.weekday() + 1) % 7)
        today_sunday = today - datetime.timedelta((today.weekday() + 1) % 7)

        wn = (today_sunday - epoch_sunday).days / 7

        nsec = tow_f / 256.0 * 10**6
        return self.gps_time_to_utc(wn, tow, nsec)

    def tow_to_utc(self, tow):
        return self.tow_f_to_utc(tow, 0)

    def get_time_stamp(self, tow):
        stamp = rospy.Time.now()
        if self.use_gps_time:
            stamp = self.utc_times.get(tow, None)
            if stamp is None:
                rospy.logwarn("Cannot find GPS time stamp. Converting manually up to ms precision.")
                stamp = self.tow_to_utc(tow)
        return stamp

    def cb_sbp_pos_llh(self, msg_raw, **metadata):
        msg = MsgPosLLH(msg_raw)

        stamp = self.get_time_stamp(msg.tow)

        # Invalid messages.
        fix_mode = msg.flags & 0b111 # Lower 3 bits define fix mode.
        if fix_mode == PosLlhMulti.FIX_MODE_INVALID:
            return
        # SPP GPS messages.
        elif fix_mode == PosLlhMulti.FIX_MODE_SPP:
            self.publish_spp(msg.lat, msg.lon, msg.height, stamp, self.var_spp, NavSatStatus.STATUS_FIX)
        # Differential GNSS (DGNSS)
        elif fix_mode == PosLlhMulti.FIX_MODE_DGNSS:
            rospy.logwarn(
                "[cb_sbp_pos_llh]: case FIX_MODE_DGNSS was not implemented yet." +
                "Contact the package/repository maintainers.")
            # TODO what to do here?
            return
        # RTK messages.
        elif fix_mode == PosLlhMulti.FIX_MODE_FLOAT_RTK:
            if self.origin_enu_set:
                self.publish_rtk_float(msg.lat, msg.lon, msg.height, stamp, self.var_rtk_float)
            else:
                rospy.logwarn_throttle(5,
                    "[cb_sbp_pos_llh]: cannot publish float RTK because ENU origin is not set. " +
                    "Waiting for RTK fix.")
        elif fix_mode == PosLlhMulti.FIX_MODE_FIX_RTK:
            # Use first RTK fix to set origin ENU frame, if it was not set by rosparam.
            if not self.origin_enu_set:
                self.init_geodetic_reference(msg.lat, msg.lon, msg.height)

            self.publish_rtk_fix(msg.lat, msg.lon, msg.height, stamp, self.var_rtk_fix)
        # Dead reckoning
        elif fix_mode == PosLlhMulti.FIX_MODE_DEAD_RECKONING:
            rospy.logwarn(
                "[cb_sbp_pos_llh]: case FIX_MODE_DEAD_RECKONING was not implemented yet." +
                "Contact the package/repository maintainers.")
            return
        # SBAS Position
        elif fix_mode == PosLlhMulti.FIX_MODE_SBAS:
            self.publish_spp(msg.lat, msg.lon, msg.height, stamp, self.var_spp_sbas, NavSatStatus.STATUS_SBAS_FIX)
        else:
            rospy.logerr(
                "[cb_sbp_pos_llh]: Unknown case, you found a bug!" +
                "Contact the package/repository maintainers." +
                "Report: 'msg.flags & 0b111 = %d'" % (fix_mode))
            return

        # Update debug msg and publish.
        self.receiver_state_msg.rtk_mode_fix = True if (fix_mode == PosLlhMulti.FIX_MODE_FIX_RTK) else False

        if fix_mode == PosLlhMulti.FIX_MODE_INVALID:
            self.receiver_state_msg.fix_mode = ReceiverState_V2_4_1.STR_FIX_MODE_INVALID
        elif fix_mode == PosLlhMulti.FIX_MODE_SPP:
            self.receiver_state_msg.fix_mode = ReceiverState_V2_4_1.STR_FIX_MODE_SPP
        elif fix_mode == PosLlhMulti.FIX_MODE_DGNSS:
            self.receiver_state_msg.fix_mode = ReceiverState_V2_4_1.STR_FIX_MODE_DGNSS
        elif fix_mode == PosLlhMulti.FIX_MODE_FLOAT_RTK:
            self.receiver_state_msg.fix_mode = ReceiverState_V2_4_1.STR_FIX_MODE_FLOAT_RTK
        elif fix_mode == PosLlhMulti.FIX_MODE_FIX_RTK:
            self.receiver_state_msg.fix_mode = ReceiverState_V2_4_1.STR_FIX_MODE_FIXED_RTK
        elif fix_mode == PosLlhMulti.FIX_MODE_DEAD_RECKONING:
            self.receiver_state_msg.fix_mode = ReceiverState_V2_4_1.FIX_MODE_DEAD_RECKONING
        elif fix_mode == PosLlhMulti.FIX_MODE_SBAS:
            self.receiver_state_msg.fix_mode = ReceiverState_V2_4_1.STR_FIX_MODE_SBAS
        else:
            self.receiver_state_msg.fix_mode = ReceiverState_V2_4_1.STR_FIX_MODE_UNKNOWN

        self.publish_receiver_state_msg()

    def cb_sbp_pos_llh_cov(self, msg_raw, **metadata):
        if self.publishers['pos_llh_cov'].get_num_connections() == 0:
            return

        msg = MsgPosLLHCov(msg_raw)
        if msg.flags == PosLlhCov.FIX_MODE_INVALID:
            return

        # Set time stamp.
        stamp = self.get_time_stamp(msg.tow)

        # Publish NavSatFix.
        navsatfix_msg = NavSatFix()
        navsatfix_msg.header.stamp = stamp
        navsatfix_msg.header.frame_id = self.llh_frame
        navsatfix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_KNOWN
        # WARNING: We do not distinguish between the different fix types.
        if msg.flags == PosLlhCov.FIX_MODE_DEAD_RECKONING:
            navsatfix_msg.status.status = NavSatStatus.STATUS_NO_FIX
        elif msg.flags == PosLlhCov.FIX_MODE_SBAS:
            navsatfix_msg.status.status = NavSatStatus.STATUS_SBAS_FIX
        elif msg.flags == PosLlhCov.FIX_MODE_FLOAT_RTK or msg.flags == PosLlhCov.FIX_MODE_FIX_RTK:
            navsatfix_msg.status.status = NavSatStatus.STATUS_GBAS_FIX
        else:
            navsatfix_msg.status.status = NavSatStatus.STATUS_FIX

        navsatfix_msg.status.service = 0
        if self.receiver_state_msg.num_gps_sat > 0:
            navsatfix_msg.status.service = navsatfix_msg.status.service + NavSatStatus.SERVICE_GPS
        if self.receiver_state_msg.num_glonass_sat > 0:
            navsatfix_msg.status.service = navsatfix_msg.status.service + NavSatStatus.SERVICE_GLONASS
        if self.receiver_state_msg.num_bds_sat > 0:
            navsatfix_msg.status.service = navsatfix_msg.status.service + NavSatStatus.SERVICE_COMPASS
        if self.receiver_state_msg.num_gal_sat > 0:
            navsatfix_msg.status.service = navsatfix_msg.status.service + NavSatStatus.SERVICE_GALILEO
        navsatfix_msg.latitude = msg.lat
        navsatfix_msg.longitude = msg.lon
        navsatfix_msg.altitude = msg.height
        navsatfix_msg.position_covariance = [msg.cov_n_n, msg.cov_n_e, msg.cov_n_d,
                                             msg.cov_n_e, msg.cov_e_e, msg.cov_e_d,
                                             msg.cov_n_d, msg.cov_e_d, msg.cov_d_d]

        self.publishers['pos_llh_cov'].publish(navsatfix_msg)

    def cb_sbp_baseline_ned_cov(self, msg_raw, **metadata):
        if self.publishers['baseline_ned_cov'].get_num_connections() == 0 \
        and self.publishers['baseline_ned_cov_viz'].get_num_connections() == 0:
            return

        msg = MsgBaselineNED(msg_raw)
        if msg.flags == 0:
            return

        # Set time stamp.
        stamp = self.get_time_stamp(msg.tow)

        # Get position in m.
        x = msg.n * self.kFromMilli
        y = msg.e * self.kFromMilli
        z = msg.d * self.kFromMilli

        # Get horizontal and vertical covariances.
        cov_h_h = (msg.h_accuracy * self.kFromMilli)**2
        cov_v_v = (msg.v_accuracy * self.kFromMilli)**2

        if self.publishers['baseline_ned_cov'].get_num_connections() > 0:
            # Publish ecef.
            baseline_msg = PositionWithCovarianceStamped()
            baseline_msg.header.stamp = stamp
            baseline_msg.header.frame_id = self.base_ned_frame

            baseline_msg.position.position.x = x
            baseline_msg.position.position.y = y
            baseline_msg.position.position.z = z

            baseline_msg.position.covariance = [cov_h_h, 0.0, 0.0,
                                                0.0, cov_h_h, 0.0,
                                                0.0, 0.0, cov_v_v]

            self.publishers['baseline_ned_cov'].publish(baseline_msg)

        if self.publishers['baseline_ned_cov_viz'].get_num_connections() > 0:
            # https://answers.ros.org/question/11081/plot-a-gaussian-3d-representation-with-markers-in-rviz/
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = self.base_ned_frame
            marker.type = marker.SPHERE
            marker.action = marker.ADD

            cov = np.matrix([[cov_h_h, 0.0, 0.0],
                             [0.0, cov_h_h, 0.0],
                             [0.0, 0.0, cov_v_v]])

            (eig_values, eig_vectors) = np.linalg.eig(cov)

            R = eig_vectors.transpose()
            q = quaternion.from_rotation_matrix(R)

            marker.pose.orientation.x = q.x
            marker.pose.orientation.y = q.y
            marker.pose.orientation.z = q.z
            marker.pose.orientation.w = q.w

            marker.scale.x = math.sqrt(eig_values[0])
            marker.scale.y = math.sqrt(eig_values[1])
            marker.scale.z = math.sqrt(eig_values[2])

            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = z

            self.publishers['baseline_ned_cov_viz'].publish(marker)

    def cb_sbp_pos_ecef_cov(self, msg_raw, **metadata):
        if self.publishers['pos_ecef_cov'].get_num_connections() == 0 \
        and self.publishers['pos_ecef_cov_viz'].get_num_connections() == 0:
            return

        msg = MsgPosECEFCov(msg_raw)
        if msg.flags == PosLlhCov.FIX_MODE_INVALID:
            return

        # Set time stamp.
        stamp = self.get_time_stamp(msg.tow)

        if self.publishers['pos_ecef_cov'].get_num_connections() > 0:
            # Publish ecef.
            ecef_msg = PositionWithCovarianceStamped()
            ecef_msg.header.stamp = stamp
            ecef_msg.header.frame_id = self.ecef_frame

            ecef_msg.position.position.x = msg.x
            ecef_msg.position.position.y = msg.y
            ecef_msg.position.position.z = msg.z

            ecef_msg.position.covariance = [msg.cov_x_x, msg.cov_x_y, msg.cov_x_z,
                                            msg.cov_x_y, msg.cov_y_y, msg.cov_y_z,
                                            msg.cov_x_z, msg.cov_y_z, msg.cov_z_z]

            self.publishers['pos_ecef_cov'].publish(ecef_msg)

        if self.publishers['pos_ecef_cov_viz'].get_num_connections() > 0:
            # https://answers.ros.org/question/11081/plot-a-gaussian-3d-representation-with-markers-in-rviz/
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = self.ecef_frame
            marker.type = marker.SPHERE
            marker.action = marker.ADD

            cov = np.matrix([[msg.cov_x_x, msg.cov_x_y, msg.cov_x_z],
                             [msg.cov_x_y, msg.cov_y_y, msg.cov_y_z],
                             [msg.cov_x_z, msg.cov_y_z, msg.cov_z_z]])

            (eig_values, eig_vectors) = np.linalg.eig(cov)

            R = eig_vectors.transpose()
            q = quaternion.from_rotation_matrix(R)

            marker.pose.orientation.x = q.x
            marker.pose.orientation.y = q.y
            marker.pose.orientation.z = q.z
            marker.pose.orientation.w = q.w

            marker.scale.x = math.sqrt(eig_values[0])
            marker.scale.y = math.sqrt(eig_values[1])
            marker.scale.z = math.sqrt(eig_values[2])

            marker.pose.position.x = msg.x
            marker.pose.position.x = msg.y
            marker.pose.position.x = msg.z

            self.publishers['pos_ecef_cov_viz'].publish(marker)

    def cb_sbp_vel_ned_cov(self, msg_raw, **metadata):
        if self.publishers['vel_ned_cov'].get_num_connections() == 0:
            return

        msg = MsgVelNEDCov(msg_raw)
        if msg.flags == VelNedCov.VEL_MODE_INVALID:
            return

        # Set time stamp.
        stamp = self.get_time_stamp(msg.tow)

        # Publish NED velocity.
        vel_ned_msg = VelocityWithCovarianceStamped()
        vel_ned_msg.header.stamp = stamp
        vel_ned_msg.header.frame_id = self.antenna_ned_frame

        vel_ned_msg.velocity.velocity.x = msg.n * PiksiMulti.kFromMilli
        vel_ned_msg.velocity.velocity.y = msg.e * PiksiMulti.kFromMilli
        vel_ned_msg.velocity.velocity.z = msg.d * PiksiMulti.kFromMilli

        vel_ned_msg.velocity.covariance = [msg.cov_n_n, msg.cov_n_e, msg.cov_n_d,
                                           msg.cov_n_e, msg.cov_e_e, msg.cov_e_d,
                                           msg.cov_n_d, msg.cov_e_d, msg.cov_d_d]

        self.publishers['vel_ned_cov'].publish(vel_ned_msg)

    def cb_sbp_vel_ecef_cov(self, msg_raw, **metadata):
        if self.publishers['vel_ecef_cov'].get_num_connections() == 0:
            return

        msg = MsgVelECEFCov(msg_raw)
        if msg.flags == VelEcefCov.VEL_MODE_INVALID:
            return

        # Set time stamp.
        stamp = self.get_time_stamp(msg.tow)

        # Publish ECEF velocity.
        vel_ecef_msg = VelocityWithCovarianceStamped()
        vel_ecef_msg.header.stamp = stamp
        vel_ecef_msg.header.frame_id = self.ecef_frame

        vel_ecef_msg.velocity.velocity.x = msg.x * PiksiMulti.kFromMilli
        vel_ecef_msg.velocity.velocity.y = msg.y * PiksiMulti.kFromMilli
        vel_ecef_msg.velocity.velocity.z = msg.z * PiksiMulti.kFromMilli

        vel_ecef_msg.velocity.covariance = [msg.cov_x_x, msg.cov_x_y, msg.cov_x_z,
                                            msg.cov_x_y, msg.cov_y_y, msg.cov_y_z,
                                            msg.cov_x_z, msg.cov_y_z, msg.cov_z_z]

        self.publishers['vel_ecef_cov'].publish(vel_ecef_msg)

    def publish_spp(self, latitude, longitude, height, stamp, variance, navsatstatus_fix):
        self.publish_wgs84_point(latitude, longitude, height, stamp, variance, navsatstatus_fix,
                                 self.publishers['spp'],
                                 self.publishers['enu_pose_spp'], self.publishers['enu_point_spp'],
                                 self.publishers['enu_transform_spp'], self.publishers['best_fix'],
                                 self.publishers['enu_pose_best_fix'])

    def publish_rtk_float(self, latitude, longitude, height, stamp, variance):
        self.publish_wgs84_point(latitude, longitude, height, stamp, variance, NavSatStatus.STATUS_GBAS_FIX,
                                 self.publishers['rtk_float'],
                                 self.publishers['enu_pose_float'], self.publishers['enu_point_float'],
                                 self.publishers['enu_transform_float'], self.publishers['best_fix'],
                                 self.publishers['enu_pose_best_fix'])

    def publish_rtk_fix(self, latitude, longitude, height, stamp, variance):
        self.publish_wgs84_point(latitude, longitude, height, stamp, variance, NavSatStatus.STATUS_GBAS_FIX,
                                 self.publishers['rtk_fix'],
                                 self.publishers['enu_pose_fix'], self.publishers['enu_point_fix'],
                                 self.publishers['enu_transform_fix'], self.publishers['best_fix'],
                                 self.publishers['enu_pose_best_fix'])

    def publish_wgs84_point(self, latitude, longitude, height, stamp, variance, navsat_status, pub_navsatfix, pub_pose,
                            pub_point, pub_transform, pub_navsatfix_best_pose, pub_pose_best_fix):
        # Navsatfix message.
        if pub_navsatfix.get_num_connections() > 0 or \
        pub_navsatfix_best_pose.get_num_connections() > 0:
            navsatfix_msg = NavSatFix()
            navsatfix_msg.header.stamp = stamp
            navsatfix_msg.header.frame_id = self.navsatfix_frame_id
            navsatfix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            navsatfix_msg.status.service = NavSatStatus.SERVICE_GPS
            navsatfix_msg.latitude = latitude
            navsatfix_msg.longitude = longitude
            navsatfix_msg.altitude = height
            navsatfix_msg.status.status = navsat_status
            navsatfix_msg.position_covariance = [variance[0], 0, 0,
                                                 0, variance[1], 0,
                                                 0, 0, variance[2]]
            pub_navsatfix.publish(navsatfix_msg)
            pub_navsatfix_best_pose.publish(navsatfix_msg)

        if pub_pose.get_num_connections() == 0 and \
        pub_pose_best_fix.get_num_connections() == 0 and \
        pub_point.get_num_connections() == 0 and \
        pub_transform.get_num_connections() == 0:
            return

        # Local Enu coordinate.
        (east, north, up) = self.geodetic_to_enu(latitude, longitude, height)

        # Pose message.
        if pub_pose.get_num_connections() > 0 or \
        pub_pose_best_fix.get_num_connections() > 0:
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.stamp = stamp
            pose_msg.header.frame_id = self.enu_frame_id
            pose_msg.pose = self.enu_to_pose_msg(east, north, up, variance)
            pub_pose.publish(pose_msg)
            pub_pose_best_fix.publish(pose_msg)

        # Point message.
        if pub_point.get_num_connections() > 0:
            point_msg = PointStamped()
            point_msg.header.stamp = stamp
            point_msg.header.frame_id = self.enu_frame_id
            point_msg.point = self.enu_to_point_msg(east, north, up)
            pub_point.publish(point_msg)

        # Transform message.
        if pub_transform.get_num_connections() > 0:
            transform_msg = TransformStamped()
            transform_msg.header.stamp = stamp
            transform_msg.header.frame_id = self.enu_frame_id
            transform_msg.child_frame_id = self.transform_child_frame_id
            transform_msg.transform = self.enu_to_transform_msg(east, north, up)
            pub_transform.publish(transform_msg)

    def cb_sbp_heartbeat(self, msg_raw, **metadata):
        msg = MsgHeartbeat(msg_raw)

        # Let watchdag know messages are still arriving
        self.watchdog_time = rospy.get_rostime()

        # Start watchdog with 10 second timeout to ensure we keep getting gps
        if not self.messages_started:
            self.messages_started = True
            rospy.Timer(rospy.Duration(10), self.cb_watchdog)

        heartbeat_msg = Heartbeat()
        heartbeat_msg.header.stamp = rospy.Time.now()
        heartbeat_msg.system_error = msg.flags & 0x01
        heartbeat_msg.io_error = msg.flags & 0x02
        heartbeat_msg.swift_nap_error = msg.flags & 0x04
        heartbeat_msg.sbp_minor_version = (msg.flags & 0xFF00) >> 8
        heartbeat_msg.sbp_major_version = (msg.flags & 0xFF0000) >> 16
        heartbeat_msg.external_antenna_present = (msg.flags & 0x80000000) >> 31

        self.publishers['heartbeat'].publish(heartbeat_msg)

        # Update debug msg and publish.
        self.receiver_state_msg.system_error = heartbeat_msg.system_error
        self.receiver_state_msg.io_error = heartbeat_msg.io_error
        self.receiver_state_msg.swift_nap_error = heartbeat_msg.swift_nap_error
        self.receiver_state_msg.external_antenna_present = heartbeat_msg.external_antenna_present
        self.publish_receiver_state_msg()

        if self.base_station_mode:
            self.multicaster.sendSbpPacket(msg_raw)

    def cb_sbp_measurement_state(self, msg_raw, **metadata):
        msg = MsgMeasurementState(msg_raw)

        measurement_state_msg = piksi_rtk_msgs.msg.MeasurementState_V2_4_1()
        measurement_state_msg.header.stamp = rospy.Time.now()
        measurement_state_msg.sat = []
        measurement_state_msg.code = []
        measurement_state_msg.cn0 = []

        # Temporary variables for receiver state message.
        num_gps_sat = 0
        cn0_gps = []
        num_sbas_sat = 0
        cn0_sbas = []
        num_glonass_sat = 0
        cn0_glonass = []
        num_bds_sat = 0
        cn0_bds = []
        num_gal_sat = 0
        cn0_gal = []

        for single_measurement_state in msg.states:

            # Use satellites with valid cn0.
            if single_measurement_state.cn0 > 0.0:

                measurement_state_msg.sat.append(single_measurement_state.mesid.sat)
                measurement_state_msg.code.append(single_measurement_state.mesid.code)
                measurement_state_msg.cn0.append(single_measurement_state.cn0)

                # Receiver state fields.
                code = single_measurement_state.mesid.code
                if code == piksi_rtk_msgs.msg.MeasurementState_V2_4_1.CODE_GPS_L1CA or \
                   code == piksi_rtk_msgs.msg.MeasurementState_V2_4_1.CODE_GPS_L2CM or \
                   code == piksi_rtk_msgs.msg.MeasurementState_V2_4_1.CODE_GPS_L1P or \
                   code == piksi_rtk_msgs.msg.MeasurementState_V2_4_1.CODE_GPS_L2P:
                    num_gps_sat += 1
                    cn0_gps.append(single_measurement_state.cn0)

                elif code == piksi_rtk_msgs.msg.MeasurementState_V2_4_1.CODE_SBAS_L1CA:
                    num_sbas_sat += 1
                    cn0_sbas.append(single_measurement_state.cn0)

                elif code == piksi_rtk_msgs.msg.MeasurementState_V2_4_1.CODE_GLO_L1CA or \
                     code == piksi_rtk_msgs.msg.MeasurementState_V2_4_1.CODE_GLO_L2CA:
                    num_glonass_sat += 1
                    cn0_glonass.append(single_measurement_state.cn0)

                elif code == piksi_rtk_msgs.msg.MeasurementState_V2_4_1.CODE_BDS2_B1 or \
                     code == piksi_rtk_msgs.msg.MeasurementState_V2_4_1.CODE_BDS2_B2:
                    num_bds_sat += 1
                    cn0_bds.append(single_measurement_state.cn0)

                elif code == piksi_rtk_msgs.msg.MeasurementState_V2_4_1.CODE_GAL_E1B or \
                     code == piksi_rtk_msgs.msg.MeasurementState_V2_4_1.CODE_GAL_E7I:
                    num_gal_sat += 1
                    cn0_gal.append(single_measurement_state.cn0)

                else:
                    rospy.logwarn("[cb_sbp_measurement_state]: Unknown satellite code %d.", code)

        # Publish if there's at least one element in each array.
        if len(measurement_state_msg.sat) \
                and len(measurement_state_msg.code) \
                and len(measurement_state_msg.cn0):
            self.publishers['measurement_state'].publish(measurement_state_msg)

            # Update debug msg and publish.
            self.receiver_state_msg.num_sat = num_gps_sat + num_sbas_sat + num_glonass_sat + num_bds_sat + num_gal_sat
            self.receiver_state_msg.sat = measurement_state_msg.sat
            self.receiver_state_msg.cn0 = measurement_state_msg.cn0
            self.receiver_state_msg.num_gps_sat = num_gps_sat
            self.receiver_state_msg.cn0_gps = cn0_gps
            self.receiver_state_msg.num_sbas_sat = num_sbas_sat
            self.receiver_state_msg.cn0_sbas = cn0_sbas
            self.receiver_state_msg.num_glonass_sat = num_glonass_sat
            self.receiver_state_msg.cn0_glonass = cn0_glonass
            self.receiver_state_msg.num_bds_sat = num_bds_sat
            self.receiver_state_msg.cn0_bds = cn0_bds
            self.receiver_state_msg.num_gal_sat = num_gal_sat
            self.receiver_state_msg.cn0_gal = cn0_gal

            self.publish_receiver_state_msg()

    def publish_receiver_state_msg(self):
        if self.publishers['receiver_state'].get_num_connections() == 0:
            return
        self.receiver_state_msg.header.stamp = rospy.Time.now()
        self.publishers['receiver_state'].publish(self.receiver_state_msg)

    def init_geodetic_reference(self, latitude, longitude, altitude):
        if self.origin_enu_set:
            return

        self.latitude0 = math.radians(latitude)
        self.longitude0 = math.radians(longitude)
        self.altitude0 = altitude

        (self.initial_ecef_x, self.initial_ecef_y, self.initial_ecef_z) = self.geodetic_to_ecef(latitude, longitude,
                                                                                                altitude)
        # Compute ECEF to NED.
        phiP = math.atan2(self.initial_ecef_z,
                          math.sqrt(math.pow(self.initial_ecef_x, 2) + math.pow(self.initial_ecef_y, 2)))
        self.ecef_to_ned_matrix = self.n_re(phiP, self.longitude0)

        self.origin_enu_set = True

        rospy.loginfo("Origin ENU frame set to: %.6f, %.6f, %.2f" % (latitude, longitude, altitude))

    def geodetic_to_ecef(self, latitude, longitude, altitude):
        # Convert geodetic coordinates to ECEF.
        # http://code.google.com/p/pysatel/source/browse/trunk/coord.py?r=22
        lat_rad = math.radians(latitude)
        lon_rad = math.radians(longitude)
        xi = math.sqrt(1 - PiksiMulti.kFirstEccentricitySquared * math.sin(lat_rad) * math.sin(lat_rad))
        x = (PiksiMulti.kSemimajorAxis / xi + altitude) * math.cos(lat_rad) * math.cos(lon_rad)
        y = (PiksiMulti.kSemimajorAxis / xi + altitude) * math.cos(lat_rad) * math.sin(lon_rad)
        z = (PiksiMulti.kSemimajorAxis / xi * (1 - PiksiMulti.kFirstEccentricitySquared) + altitude) * math.sin(lat_rad)

        return x, y, z

    def ecef_to_ned(self, x, y, z):
        # Converts ECEF coordinate position into local-tangent-plane NED.
        # Coordinates relative to given ECEF coordinate frame.
        vect = np.array([0.0, 0.0, 0.0])
        vect[0] = x - self.initial_ecef_x
        vect[1] = y - self.initial_ecef_y
        vect[2] = z - self.initial_ecef_z
        ret = self.ecef_to_ned_matrix.dot(vect)
        n = ret[0]
        e = ret[1]
        d = -ret[2]

        return n, e, d

    def geodetic_to_enu(self, latitude, longitude, altitude):
        # Geodetic position to local ENU frame
        (x, y, z) = self.geodetic_to_ecef(latitude, longitude, altitude)
        (north, east, down) = self.ecef_to_ned(x, y, z)

        # Return East, North, Up coordinate.
        return east, north, -down

    def n_re(self, lat_radians, lon_radians):
        s_lat = math.sin(lat_radians)
        s_lon = math.sin(lon_radians)
        c_lat = math.cos(lat_radians)
        c_lon = math.cos(lon_radians)

        ret = np.eye(3)
        ret[0, 0] = -s_lat * c_lon
        ret[0, 1] = -s_lat * s_lon
        ret[0, 2] = c_lat
        ret[1, 0] = -s_lon
        ret[1, 1] = c_lon
        ret[1, 2] = 0.0
        ret[2, 0] = c_lat * c_lon
        ret[2, 1] = c_lat * s_lon
        ret[2, 2] = s_lat

        return ret

    def enu_to_pose_msg(self, east, north, up, variance):
        pose_msg = PoseWithCovariance()

        # Fill covariance using variance parameter of GPS.
        pose_msg.covariance[6 * 0 + 0] = variance[0]
        pose_msg.covariance[6 * 1 + 1] = variance[1]
        pose_msg.covariance[6 * 2 + 2] = variance[2]

        # Fill pose section.
        pose_msg.pose.position.x = east
        pose_msg.pose.position.y = north
        pose_msg.pose.position.z = up

        # GPS points do not have orientation
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 1.0

        return pose_msg

    def enu_to_point_msg(self, east, north, up):
        point_msg = Point()

        # Fill pose section.
        point_msg.x = east
        point_msg.y = north
        point_msg.z = up

        return point_msg

    def enu_to_transform_msg(self, east, north, up):
        transform_msg = Transform()

        # Fill message.
        transform_msg.translation.x = east
        transform_msg.translation.y = north
        transform_msg.translation.z = up

        # Set orientation to unit quaternion as it does not really metter.
        transform_msg.rotation.x = 0.0
        transform_msg.rotation.y = 0.0
        transform_msg.rotation.z = 0.0
        transform_msg.rotation.w = 1.0

        return transform_msg

    def reset_piksi_service_callback(self, request):
        response = std_srvs.srv.SetBoolResponse()

        if request.data:
            # Send reset message.
            reset_sbp = SBP(SBP_MSG_RESET)
            reset_sbp.payload = ''
            reset_msg = reset_sbp.pack()
            self.driver.write(reset_msg)

            rospy.logwarn("Swift receiver hard reset via rosservice call.")

            # Init messages with "memory".
            self.receiver_state_msg = self.init_receiver_state_msg()
            self.num_wifi_corrections = self.init_num_corrections_msg()

            response.success = True
            response.message = "Swift receiver reset command sent."
        else:
            response.success = False
            response.message = "Swift receiver reset command not sent."

        return response

    def settings_write_server(self, request):
        response = SettingsWriteResponse()

        self.settings_write(request.section_setting, request.setting, request.value)
        response.success = True
        response.message = "Settings written. Please use service 'settings_read_req' if you want to double check."

        return response

    def settings_read_req_server(self, request):
        response = SettingsReadReqResponse()

        # Make sure we do not have any old setting in memory.
        self.clear_last_setting_read()
        self.settings_read_req(request.section_setting, request.setting)
        response.success = True
        response.message = "Read-request sent. Please use 'settings_read_resp' to get the response."

        return response

    def settings_read_resp_server(self, request):
        response = SettingsReadRespResponse()

        if self.last_section_setting_read and self.last_setting_read and self.last_value_read:
            response.success = True
            response.message = ""
            response.section_setting = self.last_section_setting_read
            response.setting = self.last_setting_read
            response.value = self.last_value_read
        else:
            response.success = False
            response.message = "Please trigger a new 'settings_read_req' via service call."
            response.section_setting = []
            response.setting = []
            response.value = []

        self.clear_last_setting_read()

        return response

    def settings_save_callback(self, request):
        response = std_srvs.srv.SetBoolResponse()

        if request.data:
            self.settings_save()
            response.success = True
            response.message = "Swift receiver settings have been saved to flash."
        else:
            response.success = False
            response.message = "Please pass 'true' to this service call to explicitly save to flash the local settings."

        return response

    def get_installed_sbp_version(self):
        command = ["pip", "show", "sbp"]
        pip_show_output = subprocess.Popen(command, stdout=subprocess.PIPE)
        out, error = pip_show_output.communicate()

        # Search for version number, output assumed in the form "Version: X.X.X"
        version_output = re.search("Version: \d+.\d+.\d+", out)

        if version_output is None:
            # No version found
            rospy.logfatal("No SBP library found. Please install it by using script in 'install' folder.")
            rospy.signal_shutdown("No SBP library found. Please install it by using script in 'install' folder.")
            return -1
        else:
            # extract version number
            version_output_string = version_output.group()
            version_number = re.search("\d+.\d+.\d+", version_output_string)
            return version_number.group()

    def settings_write(self, section_setting, setting, value):
        """
        Write the defined configuration to Swift receiver.
        """
        setting_string = '%s\0%s\0%s\0' % (section_setting, setting, value)
        write_msg = MsgSettingsWrite(setting=setting_string)
        self.framer(write_msg)

    def settings_save(self):
        """
        Save settings message persists the device's current settings
        configuration to its on-board flash memory file system.
        """
        save_msg = MsgSettingsSave()
        self.framer(save_msg)

    def settings_read_req(self, section_setting, setting):
        """
        Request a configuration value to Swift receiver.
        """
        setting_string = '%s\0%s\0' % (section_setting, setting)
        read_req_msg = MsgSettingsReadReq(setting=setting_string)
        self.framer(read_req_msg)

    def cb_settings_read_resp(self, msg_raw, **metadata):
        """
        Response to a settings_read_req.
        """
        msg = MsgSettingsReadResp(msg_raw)
        setting_string = msg.setting.split('\0')
        self.last_section_setting_read = setting_string[0]
        self.last_setting_read = setting_string[1]
        self.last_value_read = setting_string[2]

    def settings_read_by_index_req(self, index):
        """
        Request a configuration value to Swift receiver by parameter index number.
        """
        read_req_by_index_msg = MsgSettingsReadByIndexReq(index=index)
        self.framer(read_req_by_index_msg)

    def cb_sbp_settings_read_by_index_resp(self, msg_raw, **metadata):
        """
        Response to a settings_read_by_index_req.
        """
        msg = MsgSettingsReadByIndexResp(msg_raw)
        setting_string = msg.setting.split('\0')
        self.last_section_setting_read = setting_string[0]
        self.last_setting_read = setting_string[1]
        self.last_value_read = setting_string[2]

    def cb_sbp_imu_raw(self, msg_raw, **metadata):
        msg = MsgImuRaw(msg_raw)

        if not self.has_imu_scale:
            rospy.logwarn_throttle(10, "IMU scale unknown.")
            return

        if msg.tow & (1 << (32 - 1)) and self.use_gps_time:
            rospy.logwarn("IMU time unknown.")
            return

        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        if (self.use_gps_time):
            imu_msg.header.stamp = self.tow_f_to_utc(msg.tow, msg.tow_f)

        imu_msg.header.frame_id = 'piksi_imu'
        imu_msg.orientation.w = 1.0

        imu_msg.angular_velocity.x = msg.gyr_x * self.gyro_scale
        imu_msg.angular_velocity.y = msg.gyr_y * self.gyro_scale
        imu_msg.angular_velocity.z = msg.gyr_z * self.gyro_scale

        imu_msg.linear_acceleration.x = msg.acc_x * self.acc_scale
        imu_msg.linear_acceleration.y = msg.acc_y * self.acc_scale
        imu_msg.linear_acceleration.z = msg.acc_z * self.acc_scale

        self.publishers['imu'].publish(imu_msg)

    def cb_sbp_imu_aux(self, msg_raw, **metadata):
        msg = MsgImuAux(msg_raw)

        if msg.imu_type != 0:
            rospy.logwarn("Unkown IMU type.")
            self.has_imu_scale = False
            return

        # Scale accelerometer.
        acc_conf = msg.imu_conf & 0b1111 # Lower 4 bits.
        acc_range = 2**(acc_conf+1) # 2 to 16 g
        self.acc_scale = acc_range * PiksiMulti.kAccPrescale

        # Scale gyroscope.
        gyro_conf = msg.imu_conf >> 4 # Upper 4 bits.
        gyro_range = 2000 / (2**gyro_conf) # 125 to 2000 dps
        self.gyro_scale = gyro_range * PiksiMulti.kGyroPrescale

        if not self.has_imu_scale:
            rospy.loginfo("Received IMU scale.")
        self.has_imu_scale = True  

    def cb_sbp_mag_raw(self, msg_raw, **metadata):
        msg = MsgMagRaw(msg_raw)

        if msg.tow & (1 << (32 - 1)) and self.use_gps_time:
            rospy.logwarn("MAG time unknown.")
            return

        mag_msg = MagneticField()
        mag_msg.header.stamp = rospy.Time.now()
        if (self.use_gps_time):
            mag_msg.header.stamp = self.tow_f_to_utc(msg.tow, msg.tow_f)

        mag_msg.header.frame_id = 'piksi_imu'

        mag_msg.magnetic_field.x = msg.mag_x * PiksiMulti.kMagScaleXY
        mag_msg.magnetic_field.y = msg.mag_y * PiksiMulti.kMagScaleXY
        mag_msg.magnetic_field.z = msg.mag_z * PiksiMulti.kMagScaleZ

        self.publishers['mag'].publish(mag_msg)

    def clear_last_setting_read(self):
        self.last_section_setting_read = []
        self.last_setting_read = []
        self.last_value_read = []
