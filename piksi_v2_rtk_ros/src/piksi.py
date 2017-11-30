#!/usr/bin/env python

#
#  Title:        piksi.py
#  Description:  ROS Driver for the Piksi V2 RTK GPS module
#  Dependencies: libsbp (https://github.com/swift-nav/libsbp), tested with v1.2.1
#  Based on original work of https://bitbucket.org/Daniel-Eckert/piksi_node
#

import rospy
import math
import numpy as np
# Import message types
from sensor_msgs.msg import NavSatFix, NavSatStatus
from piksi_rtk_msgs.msg import *
from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped, PoseWithCovariance, Point, TransformStamped, \
    Transform
# Import Piksi SBP library
from sbp.client.drivers.pyserial_driver import PySerialDriver
from sbp.client import Handler, Framer
from sbp.navigation import *
from sbp.logging import *
from sbp.system import *
from sbp.tracking import *  # WARNING: tracking is part of the draft messages, could be removed in future releases of libsbp.
from sbp.piksi import *  # WARNING: piksi is part of the draft messages, could be removed in future releases of libsbp.
from sbp.observation import SBP_MSG_OBS, SBP_MSG_OBS_DEP_A, SBP_MSG_OBS_DEP_B, SBP_MSG_BASE_POS_LLH, \
    SBP_MSG_BASE_POS_ECEF
import sbp.version
# networking stuff
import UdpHelpers
import time
import subprocess
import re
import threading


class Piksi:
    LIB_SBP_VERSION = '1.2.1'  # sbp version used to test this driver

    # Geodetic Constants.
    kSemimajorAxis = 6378137
    kSemiminorAxis = 6356752.3142
    kFirstEccentricitySquared = 6.69437999014 * 0.001
    kSecondEccentricitySquared = 6.73949674228 * 0.001
    kFlattening = 1 / 298.257223563

    def __init__(self):
        # Print info.
        rospy.sleep(0.5)  # wait for a while for init to complete before printing
        rospy.loginfo(rospy.get_name() + " start")
        rospy.loginfo("libsbp version currently used: " + sbp.version.get_git_version())

        if Piksi.LIB_SBP_VERSION != sbp.version.get_git_version():
            rospy.logwarn("Lib SBP version in usage (%s) is different than the one used to test this driver (%s)!" % (
                sbp.version.get_git_version(), Piksi.LIB_SBP_VERSION))

        # Open a connection to Piksi.
        serial_port = rospy.get_param('~serial_port', '/dev/ttyUSB0')
        baud_rate = rospy.get_param('~baud_rate', 1000000)

        try:
            self.driver = PySerialDriver(serial_port, baud=baud_rate)
        except SystemExit:
            rospy.logerr("Piksi not found on serial port '%s'", serial_port)

        # Create a handler to connect Piksi driver to callbacks.
        self.framer = Framer(self.driver.read, self.driver.write, verbose=True)
        self.handler = Handler(self.framer)

        self.debug_mode = rospy.get_param('~debug_mode', False)
        if self.debug_mode:
            rospy.loginfo("Piksi driver started in debug mode, every available topic will be published.")
        else:
            rospy.loginfo("Piksi driver started in normal mode.")

        # Corrections over WiFi settings.
        self.base_station_mode = rospy.get_param('~base_station_mode', False)
        self.udp_broadcast_addr = rospy.get_param('~broadcast_addr', '255.255.255.255')
        self.udp_port = rospy.get_param('~broadcast_port', 26078)
        self.base_station_ip_for_latency_estimation = rospy.get_param(
            '~base_station_ip_for_latency_estimation',
            '10.10.50.1')
        # Subscribe to OBS messages and relay them via UDP if in base station mode.
        if self.base_station_mode:
            rospy.loginfo("Starting in base station mode")
            self._multicaster = UdpHelpers.SbpUdpMulticaster(self.udp_broadcast_addr, self.udp_port)

            self.handler.add_callback(self.callback_sbp_obs, msg_type=SBP_MSG_OBS)
            self.handler.add_callback(self.callback_sbp_obs_dep_a, msg_type=SBP_MSG_OBS_DEP_A)
            self.handler.add_callback(self.callback_sbp_obs_dep_b, msg_type=SBP_MSG_OBS_DEP_B)
            # not sure if SBP_MSG_BASE_POS_LLH or SBP_MSG_BASE_POS_ECEF is better?
            self.handler.add_callback(self.callback_sbp_base_pos_llh, msg_type=SBP_MSG_BASE_POS_LLH)
            self.handler.add_callback(self.callback_sbp_base_pos_ecef, msg_type=SBP_MSG_BASE_POS_ECEF)
        else:
            rospy.loginfo("Starting in client station mode")
            self._multicast_recv = UdpHelpers.SbpUdpMulticastReceiver(self.udp_port, self.multicast_callback)

        # Navsatfix settings.
        self.var_spp = rospy.get_param('~var_spp', [25.0, 25.0, 64.0])
        self.var_rtk_float = rospy.get_param('~var_rtk_float', [25.0, 25.0, 64.0])
        self.var_rtk_fix = rospy.get_param('~var_rtk_fix', [0.0049, 0.0049, 0.01])
        self.navsatfix_frame_id = rospy.get_param('~navsatfix_frame_id', 'gps')

        # Local ENU frame settings
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
                '~altitude0_deg'):
            latitude0 = rospy.get_param('~latitude0_deg')
            longitude0 = rospy.get_param('~longitude0_deg')
            altitude0 = rospy.get_param('~altitude0_deg')

            # Set origin ENU frame to coordinate specified by rosparam.
            self.init_geodetic_reference(latitude0, longitude0, altitude0)
            rospy.loginfo("Origin ENU frame set by rosparam.")

        # Advertise topics.
        self.publishers = self.advertise_topics()

        # Create callbacks.
        self.create_callbacks()

        # Init messages with "memory".
        self.receiver_state_msg = self.init_receiver_state_msg()
        self.num_wifi_corrections = self.init_num_corrections_msg()

        # corrections over wifi message, if we are not the base station.
        if not self.base_station_mode:
            # start new thread to periodically ping base station.
            threading.Thread(target=self.ping_base_station_over_wifi).start()

        self.handler.start()

        # Spin.
        rospy.spin()

    def create_callbacks(self):
        # Callbacks "manually" implemented.
        self.handler.add_callback(self.navsatfix_callback, msg_type=SBP_MSG_POS_LLH)
        self.handler.add_callback(self.heartbeat_callback, msg_type=SBP_MSG_HEARTBEAT)
        self.handler.add_callback(self.tracking_state_callback, msg_type=SBP_MSG_TRACKING_STATE)
        # for now use deprecated uart_msg, as the latest one doesn't seem to work properly with libspb 1.2.1
        self.handler.add_callback(self.uart_state_callback, msg_type=SBP_MSG_UART_STATE_DEPA)

        # Callbacks generated based on ROS messages definitions.
        self.init_callback('baseline_ecef', BaselineEcef,
                           SBP_MSG_BASELINE_ECEF, MsgBaselineECEF,
                           'tow', 'x', 'y', 'z', 'accuracy', 'n_sats', 'flags')
        self.init_callback('baseline_ned', BaselineNed,
                           SBP_MSG_BASELINE_NED, MsgBaselineNED,
                           'tow', 'n', 'e', 'd', 'h_accuracy', 'v_accuracy', 'n_sats', 'flags')
        self.init_callback('dops', Dops,
                           SBP_MSG_DOPS, MsgDops, 'tow', 'gdop', 'pdop', 'tdop', 'hdop', 'vdop')
        self.init_callback('gps_time', GpsTime,
                           SBP_MSG_GPS_TIME, MsgGPSTime, 'wn', 'tow', 'ns', 'flags')
        self.init_callback('pos_ecef', PosEcef,
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

        # do not publish llh message, prefer publishing directly navsatfix_spp or navsatfix_rtk_fix.
        # self.init_callback('pos_llh', PosLlh,
        #                   SBP_MSG_POS_LLH, MsgPosLLH,
        #                   'tow', 'lat', 'lon', 'height', 'h_accuracy', 'v_accuracy', 'n_sats', 'flags')

    def init_num_corrections_msg(self):
        num_wifi_corrections = InfoWifiCorrections()
        num_wifi_corrections.header.seq = 0
        num_wifi_corrections.received_corrections = 0
        num_wifi_corrections.latency = -1

        return num_wifi_corrections

    def init_receiver_state_msg(self):
        receiver_state_msg = ReceiverState()
        receiver_state_msg.num_sat = 0  # Unkown
        receiver_state_msg.rtk_mode_fix = False  # Unkown
        receiver_state_msg.sat = []  # Unkown
        receiver_state_msg.cn0 = []  # Unkown
        receiver_state_msg.tracking_running = []  # Unkown
        receiver_state_msg.system_error = 255  # Unkown
        receiver_state_msg.io_error = 255  # Unkown
        receiver_state_msg.swift_nap_error = 255  # Unkown
        receiver_state_msg.external_antenna_present = 255  # Unkown

        return receiver_state_msg

    def advertise_topics(self):
        """
        Adverties topics.
        :return: python dictionary, with topic names used as keys and publishers as values.
        """
        publishers = {}

        publishers['rtk_fix'] = rospy.Publisher(rospy.get_name() + '/navsatfix_rtk_fix',
                                                NavSatFix, queue_size=10)
        publishers['spp'] = rospy.Publisher(rospy.get_name() + '/navsatfix_spp',
                                            NavSatFix, queue_size=10)
        publishers['heartbeat'] = rospy.Publisher(rospy.get_name() + '/heartbeat',
                                                  Heartbeat, queue_size=10)
        publishers['tracking_state'] = rospy.Publisher(rospy.get_name() + '/tracking_state',
                                                       TrackingState, queue_size=10)
        publishers['receiver_state'] = rospy.Publisher(rospy.get_name() + '/debug/receiver_state',
                                                       ReceiverState, queue_size=10)
        publishers['uart_state'] = rospy.Publisher(rospy.get_name() + '/debug/uart_state',
                                                   UartState, queue_size=10)
        publishers['baseline_ned'] = rospy.Publisher(rospy.get_name() + '/baseline_ned',
                                                     BaselineNed, queue_size=10)
        publishers['gps_time'] = rospy.Publisher(rospy.get_name() + '/gps_time',
                                                 GpsTime, queue_size=10)
        # do not publish llh message, prefer publishing directly navsatfix_spp or navsatfix_rtk_fix.
        # publishers['pos_llh'] = rospy.Publisher(rospy.get_name() + '/pos_llh',
        #                                        PosLlh, queue_size=10)
        publishers['vel_ned'] = rospy.Publisher(rospy.get_name() + '/vel_ned',
                                                VelNed, queue_size=10)
        publishers['log'] = rospy.Publisher(rospy.get_name() + '/log',
                                            Log, queue_size=10)
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

        # Topics published only if in "debug mode"
        if self.debug_mode:
            publishers['rtk_float'] = rospy.Publisher(rospy.get_name() + '/navsatfix_rtk_float',
                                                      NavSatFix, queue_size=10)
            publishers['baseline_ecef'] = rospy.Publisher(rospy.get_name() + '/baseline_ecef',
                                                          BaselineEcef, queue_size=10)
            publishers['dops'] = rospy.Publisher(rospy.get_name() + '/dops',
                                                 Dops, queue_size=10)
            publishers['pos_ecef'] = rospy.Publisher(rospy.get_name() + '/pos_ecef',
                                                     PosEcef, queue_size=10)
            publishers['vel_ecef'] = rospy.Publisher(rospy.get_name() + '/vel_ecef',
                                                     VelEcef, queue_size=10)
            publishers['enu_pose_float'] = rospy.Publisher(rospy.get_name() + '/enu_pose_float',
                                                           PoseWithCovarianceStamped, queue_size=10)
            publishers['enu_point_float'] = rospy.Publisher(rospy.get_name() + '/enu_point_float',
                                                            PointStamped, queue_size=10)
            publishers['enu_transform_float'] = rospy.Publisher(rospy.get_name() + '/enu_transform_float',
                                                                TransformStamped, queue_size=10)

        if not self.base_station_mode:
            publishers['wifi_corrections'] = rospy.Publisher(rospy.get_name() + '/debug/wifi_corrections',
                                                             InfoWifiCorrections, queue_size=10)

        return publishers

    def ping_base_station_over_wifi(self):
        """
        Ping base station periodically without blocking the driver.
        """
        ping_deadline_seconds = 3
        interval_between_pings_seconds = 5

        while not rospy.is_shutdown():
            # send ping command.
            command = ["ping",
                       "-w", str(ping_deadline_seconds),  # deadline before stopping attempt
                       "-c", "1",  # number of pings to send
                       self.base_station_ip_for_latency_estimation]
            ping = subprocess.Popen(command, stdout=subprocess.PIPE)

            out, error = ping.communicate()
            # search for 'min/avg/max/mdev' round trip delay time (rtt) numbers.
            matcher = re.compile("(\d+.\d+)/(\d+.\d+)/(\d+.\d+)/(\d+.\d+)")

            if matcher.search(out) == None:
                # no ping response within ping_deadline_seconds.
                # in python write and read operations on built-in type are atomic,
                # there's no need to use mutex.
                self.num_wifi_corrections.latency = -1
            else:
                groups_rtt = matcher.search(out).groups()
                avg_rtt = groups_rtt[1]
                # in python write and read operations on built-in type are atomic,
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
            sbp_message = sbp_type(msg)
            ros_message.header.stamp = rospy.Time.now()
            for attr in attrs:
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
        # check that required topic has been advertised
        if topic_name in self.publishers:
            ros_message = ros_datatype()

            # Add callback function
            pub = self.publishers[topic_name]
            callback_function = self.make_callback(callback_data_type, ros_message, pub, attrs)
            self.handler.add_callback(callback_function, msg_type=sbp_msg_type)

    def callback_sbp_obs(self, msg, **metadata):
        # rospy.logwarn("CALLBACK SBP OBS")
        self._multicaster.sendSbpPacket(msg)

    def callback_sbp_obs_dep_a(self, msg, **metadata):
        # rospy.logwarn("CALLBACK SBP OBS DEP A")
        self._multicaster.sendSbpPacket(msg)

    def callback_sbp_obs_dep_b(self, msg, **metadata):
        # rospy.logwarn("CALLBACK SBP OBS DEP B")
        self._multicaster.sendSbpPacket(msg)

    def callback_sbp_base_pos_llh(self, msg, **metadata):
        # rospy.logwarn("CALLBACK SBP OBS BASE LLH")
        self._multicaster.sendSbpPacket(msg)

    def callback_sbp_base_pos_ecef(self, msg, **metadata):
        # rospy.logwarn("CALLBACK SBP OBS BASE LLH")
        self._multicaster.sendSbpPacket(msg)

    def multicast_callback(self, msg, **metadata):
        # rospy.logwarn("MULTICAST Callback")
        if self.framer:
            self.framer(msg, **metadata)

            # publish debug message about wifi corrections, if enabled
            self.num_wifi_corrections.header.seq += 1
            now = rospy.get_rostime()
            self.num_wifi_corrections.header.stamp = rospy.Time.now()
            self.num_wifi_corrections.received_corrections += 1
            if not self.base_station_mode:
                self.publishers['wifi_corrections'].publish(self.num_wifi_corrections)

        else:
            rospy.logwarn("Received external SBP msg, but Piksi not connected.")

    def navsatfix_callback(self, msg_raw, **metadata):
        msg = MsgPosLLH(msg_raw)

        # SPP GPS messages.
        if msg.flags == 0:
            self.publish_spp(msg.lat, msg.lon, msg.height)

        # RTK GPS messages.
        elif msg.flags == 1 or msg.flags == 2:

            if msg.flags == 2 and self.debug_mode:  # RTK float only in debug mode.
                self.publish_rtk_float(msg.lat, msg.lon, msg.height)
            else:  # RTK fix.
                # Use first RTK fix to set origin ENU frame, if it was not set by rosparam
                if not self.origin_enu_set:
                    self.init_geodetic_reference(msg.lat, msg.lon, msg.height)

                self.publish_rtk_fix(msg.lat, msg.lon, msg.height)

            # Update debug msg and publish.
            self.receiver_state_msg.rtk_mode_fix = True if (msg.flags == 1) else False
            self.publish_receiver_state_msg()

    def publish_spp(self, latitude, longitude, height):
        self.publish_gps_point(latitude, longitude, height, self.var_spp, NavSatStatus.STATUS_FIX,
                               self.publishers['spp'],
                               self.publishers['enu_pose_spp'], self.publishers['enu_point_spp'],
                               self.publishers['enu_transform_spp'])

    def publish_rtk_float(self, latitude, longitude, height):
        self.publish_gps_point(latitude, longitude, height, self.var_rtk_float, NavSatStatus.STATUS_GBAS_FIX,
                               self.publishers['rtk_float'],
                               self.publishers['enu_pose_float'], self.publishers['enu_point_float'],
                               self.publishers['enu_transform_float'])

    def publish_rtk_fix(self, latitude, longitude, height):
        self.publish_gps_point(latitude, longitude, height, self.var_rtk_fix, NavSatStatus.STATUS_GBAS_FIX,
                               self.publishers['rtk_fix'],
                               self.publishers['enu_pose_fix'], self.publishers['enu_point_fix'],
                               self.publishers['enu_transform_fix'])

    def publish_gps_point(self, latitude, longitude, height, variance, status, pub_navsatfix, pub_pose, pub_point,
                          pub_transform):
        # Navsatfix message.
        navsatfix_msg = NavSatFix()
        navsatfix_msg.header.stamp = rospy.Time.now()
        navsatfix_msg.header.frame_id = self.navsatfix_frame_id
        navsatfix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        navsatfix_msg.status.service = NavSatStatus.SERVICE_GPS
        navsatfix_msg.latitude = latitude
        navsatfix_msg.longitude = longitude
        navsatfix_msg.altitude = height
        navsatfix_msg.status.status = status
        navsatfix_msg.position_covariance = [variance[0], 0, 0,
                                             0, variance[1], 0,
                                             0, 0, variance[2]]
        # Local Enu coordinate
        (east, north, up) = self.geodetic_to_enu(latitude, longitude, height)

        # Pose message.
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = navsatfix_msg.header.stamp
        pose_msg.header.frame_id = self.enu_frame_id
        pose_msg.pose = self.enu_to_pose_msg(east, north, up, variance)

        # Point message.
        point_msg = PointStamped()
        point_msg.header.stamp = navsatfix_msg.header.stamp
        point_msg.header.frame_id = self.enu_frame_id
        point_msg.point = self.enu_to_point_msg(east, north, up)

        # Transform message.
        transform_msg = TransformStamped()
        transform_msg.header.stamp = navsatfix_msg.header.stamp
        transform_msg.header.frame_id = self.enu_frame_id
        transform_msg.child_frame_id = self.transform_child_frame_id
        transform_msg.transform = self.enu_to_transform_msg(east, north, up)

        # Publish.
        pub_navsatfix.publish(navsatfix_msg)
        pub_pose.publish(pose_msg)
        pub_point.publish(point_msg)
        pub_transform.publish(transform_msg)

    def heartbeat_callback(self, msg_raw, **metadata):
        msg = MsgHeartbeat(msg_raw)

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

    def tracking_state_callback(self, msg_raw, **metadata):
        msg = MsgTrackingState(msg_raw)

        tracking_state_msg = TrackingState()
        tracking_state_msg.header.stamp = rospy.Time.now()
        tracking_state_msg.state = []
        tracking_state_msg.sat = []
        tracking_state_msg.code = []
        tracking_state_msg.cn0 = []

        for single_tracking_state in msg.states:
            # take only running tracking.
            track_running = single_tracking_state.state & 0x01
            if track_running:
                tracking_state_msg.state.append(single_tracking_state.state)
                tracking_state_msg.sat.append(single_tracking_state.sid.sat)
                tracking_state_msg.code.append(single_tracking_state.sid.code)
                tracking_state_msg.cn0.append(single_tracking_state.cn0)

        # publish if there's at least one element in each array.
        if len(tracking_state_msg.state) \
                and len(tracking_state_msg.sat) \
                and len(tracking_state_msg.code) \
                and len(tracking_state_msg.cn0):

            self.publishers['tracking_state'].publish(tracking_state_msg)

            # Update debug msg and publish.
            self.receiver_state_msg.num_sat = 0  # count number of satellites used to track
            for tracking_running in tracking_state_msg.state:
                self.receiver_state_msg.num_sat += tracking_running

            self.receiver_state_msg.sat = tracking_state_msg.sat
            self.receiver_state_msg.cn0 = tracking_state_msg.cn0
            self.receiver_state_msg.tracking_running = tracking_state_msg.state
            self.publish_receiver_state_msg()

    def publish_receiver_state_msg(self):
        self.receiver_state_msg.header.stamp = rospy.Time.now()
        self.publishers['receiver_state'].publish(self.receiver_state_msg)

    def uart_state_callback(self, msg_raw, **metadata):
        # for now use deprecated uart_msg, as the latest one doesn't seem to work properly with libspb 1.2.1
        msg = MsgUartStateDepa(msg_raw)

        uart_state_msg = UartState()
        uart_state_msg.header.stamp = rospy.Time.now()

        uart_state_msg.uart_a_tx_throughput = msg.uart_a.tx_throughput
        uart_state_msg.uart_a_rx_throughput = msg.uart_a.rx_throughput
        uart_state_msg.uart_a_crc_error_count = msg.uart_a.crc_error_count
        uart_state_msg.uart_a_io_error_count = msg.uart_a.io_error_count
        uart_state_msg.uart_a_tx_buffer_level = msg.uart_a.tx_buffer_level
        uart_state_msg.uart_a_rx_buffer_level = msg.uart_a.rx_buffer_level

        uart_state_msg.uart_b_tx_throughput = msg.uart_b.tx_throughput
        uart_state_msg.uart_b_rx_throughput = msg.uart_b.rx_throughput
        uart_state_msg.uart_b_crc_error_count = msg.uart_b.crc_error_count
        uart_state_msg.uart_b_io_error_count = msg.uart_b.io_error_count
        uart_state_msg.uart_b_tx_buffer_level = msg.uart_b.tx_buffer_level
        uart_state_msg.uart_b_rx_buffer_level = msg.uart_b.rx_buffer_level

        uart_state_msg.uart_ftdi_tx_throughput = msg.uart_ftdi.tx_throughput
        uart_state_msg.uart_ftdi_rx_throughput = msg.uart_ftdi.rx_throughput
        uart_state_msg.uart_ftdi_crc_error_count = msg.uart_ftdi.crc_error_count
        uart_state_msg.uart_ftdi_io_error_count = msg.uart_ftdi.io_error_count
        uart_state_msg.uart_ftdi_tx_buffer_level = msg.uart_ftdi.tx_buffer_level
        uart_state_msg.uart_ftdi_rx_buffer_level = msg.uart_ftdi.rx_buffer_level

        uart_state_msg.latency_avg = msg.latency.avg
        uart_state_msg.latency_lmin = msg.latency.lmin
        uart_state_msg.latency_lmax = msg.latency.lmax
        uart_state_msg.latency_current = msg.latency.current

        uart_state_msg.obs_period_avg = 0.0 # not implemented in deprecated UartStateA
        uart_state_msg.obs_period_pmin = 0.0 # not implemented in deprecated UartStateA
        uart_state_msg.obs_period_pmax = 0.0 # not implemented in deprecated UartStateA
        uart_state_msg.obs_period_current = 0.0 # not implemented in deprecated UartStateA

        self.publishers['uart_state'].publish(uart_state_msg)

    def init_geodetic_reference(self, latitude, longitude, altitude):
        if self.origin_enu_set:
            return

        self.latitude0 = math.radians(latitude)
        self.longitude0 = math.radians(longitude)
        self.altitude0 = altitude

        (self.initial_ecef_x, self.initial_ecef_y, self.initial_ecef_z) = self.geodetic_to_ecef(latitude, longitude,
                                                                                                altitude)
        # Compute ECEF to NED
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
        xi = math.sqrt(1 - Piksi.kFirstEccentricitySquared * math.sin(lat_rad) * math.sin(lat_rad))
        x = (Piksi.kSemimajorAxis / xi + altitude) * math.cos(lat_rad) * math.cos(lon_rad)
        y = (Piksi.kSemimajorAxis / xi + altitude) * math.cos(lat_rad) * math.sin(lon_rad)
        z = (Piksi.kSemimajorAxis / xi * (1 - Piksi.kFirstEccentricitySquared) + altitude) * math.sin(lat_rad)

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

        # return East, North, Up coordinate
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


# Main function.
if __name__ == '__main__':
    rospy.init_node('piksi')

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        piksi = Piksi()
    except rospy.ROSInterruptException:
        pass
