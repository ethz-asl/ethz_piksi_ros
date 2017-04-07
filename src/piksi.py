#!/usr/bin/env python

#
#  Title:        piksi.py
#  Description:  ROS Driver for the Piksi RTK GPS module
#  Dependencies: piksi_tool (https://github.com/swift-nav/piksi_tools), tested with v1.2.1
#  Based on original work of https://bitbucket.org/Daniel-Eckert/piksi_node
#

import rospy
import sys

# Import message types
from std_msgs.msg import Header
from sensor_msgs.msg import NavSatFix, NavSatStatus
from piksi_rtk_gps.msg import *

# Import Piksi SBP library
from sbp.client.drivers.pyserial_driver import PySerialDriver
from sbp.client import Handler, Framer
from sbp.navigation import *
from sbp.logging import *
from sbp.system import *
from sbp.tracking import *  # WARNING: tracking is part of the draft messages, could be removed in future releases of libsbp
from sbp.piksi import *  # WARNING: piksi is part of the draft messages, could be removed in future releases of libsbp
from sbp.observation import SBP_MSG_OBS, SBP_MSG_OBS_DEP_A, SBP_MSG_OBS_DEP_B, SBP_MSG_BASE_POS_LLH, SBP_MSG_BASE_POS_ECEF
import sbp.version

# networking stuff
import UdpHelpers
import time
import subprocess
import re
import threading, time

class Piksi:
    def __init__(self):

        # Print info
        rospy.sleep(0.5)  # wait for a while for init to complete before printing
        rospy.loginfo(rospy.get_name() + " start")
        rospy.loginfo("libsbp version currently used: " + sbp.version.get_git_version())

        # Open a connection to Piksi
        serial_port = rospy.get_param('~serial_port', '/dev/ttyUSB0')
        baud_rate = rospy.get_param('~baud_rate', 1000000)

        try:
            self.driver = PySerialDriver(serial_port, baud=baud_rate)
        except SystemExit:
            rospy.logerr("Piksi not found on serial port '%s'", serial_port)

        self.base_station_mode = rospy.get_param('~base_station_mode', False)
        self.udp_broadcast_addr = rospy.get_param('~broadcast_addr', '255.255.255.255')
        self.udp_port = rospy.get_param('~broadcast_port', 26078)

        # Create a handler to connect Piksi driver to callbacks
        self.framer = Framer(self.driver.read, self.driver.write, verbose=True)
        self.handler = Handler(self.framer)

        # Read settings
        self.var_spp = rospy.get_param('~var_spp', [33.0, 25.0, 64.0])
        self.var_rtk_float = rospy.get_param('~var_rtk_float', [33.0, 25.0, 64.0])
        self.var_rtk_fix = rospy.get_param('~var_rtk_fix', [0.0049, 0.0049, 0.01])

        self.base_station_ip_for_latency_estimation = rospy.get_param(
                                                            '~base_station_ip_for_latency_estimation',
                                                            '10.10.50.1')

        # Generate publisher and callback function for navsatfix messages
        self.pub_rtk_float = rospy.Publisher(rospy.get_name() + '/navsatfix_rtk_float',
                                             NavSatFix, queue_size=10)
        self.pub_rtk_fix = rospy.Publisher(rospy.get_name() + '/navsatfix_rtk_fix',
                                           NavSatFix, queue_size=10)
        self.pub_spp = rospy.Publisher(rospy.get_name() + '/navsatfix_spp',
                                           NavSatFix, queue_size=10)

        # Define fixed attributes of the NavSatFixed message
        self.navsatfix_msg = NavSatFix()
        self.navsatfix_msg.header.frame_id = rospy.get_param('~frame_id', 'gps')
        self.navsatfix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        self.navsatfix_msg.status.service = NavSatStatus.SERVICE_GPS

        self.handler.add_callback(self.navsatfix_callback, msg_type=SBP_MSG_POS_LLH)

        # Generate publisher and callback function for PiksiBaseline messages
        self.pub_piksibaseline = rospy.Publisher(rospy.get_name() + '/piksibaseline',
                                                 PiksiBaseline, queue_size=10)
        self.baseline_msg = PiksiBaseline()
        self.handler.add_callback(self.baseline_callback, msg_type=SBP_MSG_BASELINE_NED)

        # subscribe to OBS messages and relay them via UDP if in base station mode
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

        # Initialize Navigation messages
        self.init_callback_and_publisher('baseline_ecef', msg_baseline_ecef,
                                         SBP_MSG_BASELINE_ECEF, MsgBaselineECEF,
                                         'tow', 'x', 'y', 'z', 'accuracy', 'n_sats', 'flags')
        self.init_callback_and_publisher('baseline_ned', msg_baseline_ned,
                                         SBP_MSG_BASELINE_NED, MsgBaselineNED,
                                         'tow', 'n', 'e', 'd', 'h_accuracy', 'v_accuracy', 'n_sats', 'flags')
        self.init_callback_and_publisher('dops', msg_dops,
                                         SBP_MSG_DOPS, MsgDops, 'tow', 'gdop', 'pdop', 'tdop', 'hdop', 'vdop')
        self.init_callback_and_publisher('gps_time', msg_gps_time,
                                         SBP_MSG_GPS_TIME, MsgGPSTime, 'wn', 'tow', 'ns', 'flags')
        self.init_callback_and_publisher('pos_ecef', msg_pos_ecef,
                                         SBP_MSG_POS_ECEF, MsgPosECEF,
                                         'tow', 'x', 'y', 'z', 'accuracy', 'n_sats', 'flags')
        self.init_callback_and_publisher('pos_llh', msg_pos_llh,
                                         SBP_MSG_POS_LLH, MsgPosLLH,
                                         'tow', 'lat', 'lon', 'height', 'h_accuracy', 'v_accuracy', 'n_sats', 'flags')
        self.init_callback_and_publisher('vel_ecef', msg_vel_ecef,
                                         SBP_MSG_VEL_ECEF, MsgVelECEF,
                                         'tow', 'x', 'y', 'z', 'accuracy', 'n_sats', 'flags')
        self.init_callback_and_publisher('vel_ned', msg_vel_ned,
                                         SBP_MSG_VEL_NED, MsgVelNED,
                                         'tow', 'n', 'e', 'd', 'h_accuracy', 'v_accuracy', 'n_sats', 'flags')

        # Initialize Logging messages
        self.init_callback_and_publisher('log', msg_log,
                                         SBP_MSG_LOG, MsgLog, 'level', 'text')

        # Generate publisher and callback function for System messages
        self.pub_heartbeat = rospy.Publisher(rospy.get_name() + '/heartbeat',
                                             msg_heartbeat, queue_size=10)
        self.heartbeat_msg = msg_heartbeat()
        self.handler.add_callback(self.heartbeat_callback, msg_type=SBP_MSG_HEARTBEAT)

        # Generate publisher and callback function for Tracking messages
        self.pub_tracking_state = rospy.Publisher(rospy.get_name() + '/tracking_state',
                                                  msg_tracking_state, queue_size=10)
        self.tracking_state_msg = msg_tracking_state()
        self.handler.add_callback(self.tracking_state_callback, msg_type=SBP_MSG_TRACKING_STATE)

        # Generate publisher and callback function for Debug messages
        # init debug msg, required even tough publish_piksidebug is false to avoid run time errors
        self.debug_msg = PiksiDebug()
        self.debug_msg.num_sat = 0  # Unkown
        self.debug_msg.rtk_mode_fix = False  # Unkown
        self.debug_msg.sat = []  # Unkown
        self.debug_msg.cn0 = []  # Unkown
        self.debug_msg.tracking_running = []  # Unkown
        self.debug_msg.system_error = 255  # Unkown
        self.debug_msg.io_error = 255  # Unkown
        self.debug_msg.swift_nap_error = 255  # Unkown
        self.debug_msg.external_antenna_present = 255  # Unkown

        self.pub_piksidebug = rospy.Publisher(rospy.get_name() + '/debug/receiver_state',
                                              PiksiDebug, queue_size=10)

        # uart state
        # for now use deprecated uart_msg, as the latest one doesn't seem to work properly with libspb 1.2.1
        self.handler.add_callback(self.uart_state_callback, msg_type=SBP_MSG_UART_STATE_DEPA)
        self.pub_piksi_uart_state = rospy.Publisher(rospy.get_name() + '/debug/uart_state',
                                                    msg_uart_state, queue_size=10)

        # corrections over wifi message, if we are not the base station
        self.num_wifi_corrections = PiksiNumCorrections()
        self.num_wifi_corrections.header.seq = 0
        self.num_wifi_corrections.received_corrections = 0
        self.num_wifi_corrections.latency = -1
        if not self.base_station_mode:
            self.pub_piksi_wifi_corrections = rospy.Publisher(rospy.get_name() + '/debug/wifi_corrections',
                                                              PiksiNumCorrections, queue_size=10)
            # start new thread to periodically ping base station
            threading.Thread(target=self.ping_base_station_over_wifi).start()

        self.handler.start()

        # Spin
        rospy.spin()

    def ping_base_station_over_wifi(self):
        """
        Ping base station periodically without blocking the driver
        """

        ping_deadline_seconds = 3
        interval_between_pings_seconds = 5

        while not rospy.is_shutdown():
            # send ping command
            command = ["ping",
                       "-w", str(ping_deadline_seconds),  # deadline before stopping attempt
                       "-c", "1",  # number of pings to send
                       self.base_station_ip_for_latency_estimation]
            ping = subprocess.Popen(command, stdout = subprocess.PIPE)

            out, error = ping.communicate()
            # search for 'min/avg/max/mdev' rount trip delay time (rtt) numbers
            matcher = re.compile("(\d+.\d+)/(\d+.\d+)/(\d+.\d+)/(\d+.\d+)")

            if matcher.search(out) == None:
                # no ping response within ping_deadline_seconds
                # in python write and read operations on built-in type are atomic.
                # there's no need to use mutex
                self.num_wifi_corrections.latency = -1
            else:
                groups_rtt = matcher.search(out).groups()
                avg_rtt = groups_rtt[1]
                # in python write and read operations on built-in type are atomic.
                # there's no need to use mutex
                self.num_wifi_corrections.latency = float(avg_rtt)

            time.sleep(interval_between_pings_seconds)

    def make_callback(self, sbp_type, ros_message, pub, attrs):
        """
        Dynamic generator for callback functions for message types from
        the SBP library.
        Inputs: 'sbp_type' name of SBP message type
                'ros_message' ROS message type with SBP format
                'pub' ROS publisher for ros_message
                'attrs' array of attributes in SBP/ROS message
        Returns: callback function 'callback'
        """

        def callback(msg, **metadata):
            sbp_message = sbp_type(msg)
            for attr in attrs:
                setattr(ros_message, attr, getattr(sbp_message, attr))
            pub.publish(ros_message)

        return callback

    def init_callback_and_publisher(self, topic_name, ros_datatype, sbp_msg_type, callback_data_type, *attrs):
        """
        Initializes the callback function and ROS publisher for an SBP
        message type.
        Inputs: 'topic_name' name of ROS topic for publisher
                'ros_datatype' ROS custom message type
                'sbp_msg_type' name of SBP message type for callback function
                'callback_data_type' name of SBP message type for SBP library
                '*attrs' array of attributes in ROS/SBP message
        """
        if not rospy.has_param('~publish_' + topic_name):
            rospy.set_param('~publish_' + topic_name, False)
        if rospy.get_param('~publish_' + topic_name):
            pub = rospy.Publisher(rospy.get_name() + '/' + topic_name, ros_datatype, queue_size=10)
            ros_message = ros_datatype()

            # Add callback function
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
            self.num_wifi_corrections.header.stamp.secs = now.secs
            self.num_wifi_corrections.header.stamp.nsecs = now.nsecs
            self.num_wifi_corrections.received_corrections += 1
            self.pub_piksi_wifi_corrections.publish(self.num_wifi_corrections)

        else:
            rospy.logwarn("Received external SBP msg, but Piksi not connected.")

    def navsatfix_callback(self, msg_raw, **metadata):
        """
        Callback function for SBP_MSG_POS_LLH message types. Publishes
        NavSatFix messages.
        """
        msg = MsgPosLLH(msg_raw)

        self.navsatfix_msg.header.stamp = rospy.Time.now()
        self.navsatfix_msg.latitude = msg.lat
        self.navsatfix_msg.longitude = msg.lon
        self.navsatfix_msg.altitude = msg.height

        # SPP GPS messages
        if msg.flags == 0:
            self.navsatfix_msg.status.status = NavSatStatus.STATUS_FIX
            self.navsatfix_msg.position_covariance = [self.var_spp[0], 0, 0,
                                                      0, self.var_spp[1], 0,
                                                      0, 0, self.var_spp[2]]
            self.pub_spp.publish(self.navsatfix_msg)

        # RTK GPS messages
        elif msg.flags == 1 or msg.flags == 2:

            self.navsatfix_msg.status.status = NavSatStatus.STATUS_GBAS_FIX

            if msg.flags == 2:  # RTK float
                self.navsatfix_msg.position_covariance = [self.var_rtk_float[0], 0, 0,
                                                          0, self.var_rtk_float[1], 0,
                                                          0, 0, self.var_rtk_float[2]]
                self.pub_rtk_float.publish(self.navsatfix_msg)
            else:  # RTK fix
                self.navsatfix_msg.position_covariance = [self.var_rtk_fix[0], 0, 0,
                                                          0, self.var_rtk_fix[1], 0,
                                                          0, 0, self.var_rtk_fix[2]]
                self.pub_rtk_fix.publish(self.navsatfix_msg)

            # Update debug msg and publish
            self.debug_msg.rtk_mode_fix = True if (msg.flags == 1) else False
            self.publish_piksidebug_msg()

    def baseline_callback(self, msg_raw, **metadata):
        """
        Callback function for SBP_MSG_BASELINE_NED message types.
        Publishes PiksiBaseline messages.
        """
        msg = MsgBaselineNED(msg_raw)

        self.baseline_msg.header.stamp = rospy.Time.now()
        self.baseline_msg.baseline.x = msg.n
        self.baseline_msg.baseline.y = msg.e
        self.baseline_msg.baseline.z = msg.d
        self.baseline_msg.mode_fixed = msg.flags

        self.pub_piksibaseline.publish(self.baseline_msg)

    def heartbeat_callback(self, msg_raw, **metadata):
        """
        Callback function for SBP_MSG_HEARTBEAT message types.
        Publishes msg_heartbeat messages.
        """
        msg = MsgHeartbeat(msg_raw)

        self.heartbeat_msg.system_error = msg.flags & 0x01
        self.heartbeat_msg.io_error = msg.flags & 0x02
        self.heartbeat_msg.swift_nap_error = msg.flags & 0x04
        self.heartbeat_msg.sbp_minor_version = (msg.flags & 0xFF00) >> 8
        self.heartbeat_msg.sbp_major_version = (msg.flags & 0xFF0000) >> 16
        self.heartbeat_msg.external_antenna_present = (msg.flags & 0x80000000) >> 31

        self.pub_heartbeat.publish(self.heartbeat_msg)

        # Update debug msg and publish
        self.debug_msg.system_error = self.heartbeat_msg.system_error
        self.debug_msg.io_error = self.heartbeat_msg.io_error
        self.debug_msg.swift_nap_error = self.heartbeat_msg.swift_nap_error
        self.debug_msg.external_antenna_present = self.heartbeat_msg.external_antenna_present
        self.publish_piksidebug_msg()

    def tracking_state_callback(self, msg_raw, **metadata):
        """
        Callback function for SBP_MSG_TRACKING_STATE message types.
        Publishes msg_tracking_state messages.
        """
        msg = MsgTrackingState(msg_raw)

        self.tracking_state_msg.state = []
        self.tracking_state_msg.sat = []
        self.tracking_state_msg.code = []
        self.tracking_state_msg.cn0 = []

        for single_tracking_state in msg.states:
            # take only running tracking
            track_running = single_tracking_state.state & 0x01
            if track_running:
                self.tracking_state_msg.state.append(single_tracking_state.state)
                self.tracking_state_msg.sat.append(single_tracking_state.sid.sat)
                self.tracking_state_msg.code.append(single_tracking_state.sid.code)
                self.tracking_state_msg.cn0.append(single_tracking_state.cn0)

        # publish if there's at least one element in each array
        if len(self.tracking_state_msg.state) \
                and len(self.tracking_state_msg.sat) \
                and len(self.tracking_state_msg.code) \
                and len(self.tracking_state_msg.cn0):

            self.pub_tracking_state.publish(self.tracking_state_msg)

            # Update debug msg and publish
            self.debug_msg.num_sat = 0  # count number of satellites used to track
            for tracking_running in self.tracking_state_msg.state:
                self.debug_msg.num_sat += tracking_running

            self.debug_msg.sat = self.tracking_state_msg.sat
            self.debug_msg.cn0 = self.tracking_state_msg.cn0
            self.debug_msg.tracking_running = self.tracking_state_msg.state
            self.publish_piksidebug_msg()

    def publish_piksidebug_msg(self):
        """
        Callback function to publish PiksiDebug msg.
        """
        self.pub_piksidebug.publish(self.debug_msg)

    def uart_state_callback(self, msg_raw, **metadata):
        """
        Callback function for SBP_MSG_UART_STATE message types.
        Publishes msg_uart_state messages.
        """
        # for now use deprecated uart_msg, as the latest one doesn't seem to work properly with libspb 1.2.1
        msg = MsgUartStateDepa(msg_raw)

        uart_state_msg = msg_uart_state()

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

        uart_state_msg.latency_avg = msg.latency.avg
        uart_state_msg.latency_lmin = msg.latency.lmin
        uart_state_msg.latency_lmax = msg.latency.lmax
        uart_state_msg.latency_current = msg.latency.current

        self.pub_piksi_uart_state.publish(uart_state_msg)

# Main function.
if __name__ == '__main__':
    rospy.init_node('piksi')

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        piksi = Piksi()
    except rospy.ROSInterruptException:
        pass
