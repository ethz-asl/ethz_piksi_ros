#!/usr/bin/env python

#
#  Title:        piksi.py
#  Description:  ROS Driver for the Piksi RTK GPS module
#  Dependencies: libsbp (https://github.com/swift-nav/libsbp), tested with v1.2.1
#  Based on original work of https://bitbucket.org/Daniel-Eckert/piksi_node
#

import rospy
# Import message types
from sensor_msgs.msg import NavSatFix, NavSatStatus
from piksi_rtk_gps.msg import *
# Import Piksi SBP library
from sbp.client.drivers.pyserial_driver import PySerialDriver
from sbp.client import Handler, Framer
from sbp.navigation import *
from sbp.logging import *
from sbp.system import *
from sbp.tracking import *  # WARNING: tracking is part of the draft messages, could be removed in future releases of libsbp.
from sbp.piksi import *  # WARNING: piksi is part of the draft messages, could be removed in future releases of libsbp.
from sbp.observation import SBP_MSG_OBS, SBP_MSG_OBS_DEP_A, SBP_MSG_OBS_DEP_B, SBP_MSG_BASE_POS_LLH, SBP_MSG_BASE_POS_ECEF
import sbp.version
# networking stuff
import UdpHelpers
import time
import subprocess
import re
import threading

class Piksi:

    def __init__(self):
        # Print info.
        rospy.sleep(0.5)  # wait for a while for init to complete before printing
        rospy.loginfo(rospy.get_name() + " start")
        rospy.loginfo("libsbp version currently used: " + sbp.version.get_git_version())

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

        # Covariance settings.
        self.var_spp = rospy.get_param('~var_spp', [25.0, 25.0, 64.0])
        self.var_rtk_float = rospy.get_param('~var_rtk_float', [25.0, 25.0, 64.0])
        self.var_rtk_fix = rospy.get_param('~var_rtk_fix', [0.0049, 0.0049, 0.01])

        # Advertise topics.
        self.publishers = self.advertise_topics()

        # Create callbacks.
        self.create_callbacks()

        # Init messages with "memory".
        self.navsatfix_msg = self.init_navsatfix_msg()
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
        #self.init_callback('pos_llh', PosLlh,
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

    def init_navsatfix_msg(self):
        navsatfix_msg = NavSatFix()
        navsatfix_msg.header.frame_id = rospy.get_param('~frame_id', 'gps')
        navsatfix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        navsatfix_msg.status.service = NavSatStatus.SERVICE_GPS

        return navsatfix_msg

    def advertise_topics(self):
        """
        Adverties topics.
        :return: python dictionary, with topic names used as keys and publishers as values.
        """
        publishers = {}

        publishers['rtk_float'] = rospy.Publisher(rospy.get_name() + '/navsatfix_rtk_float',
                                                  NavSatFix, queue_size=10)
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
        publishers['baseline_ecef'] = rospy.Publisher(rospy.get_name() + '/baseline_ecef',
                                                      BaselineEcef, queue_size=10)
        publishers['baseline_ned'] = rospy.Publisher(rospy.get_name() + '/baseline_ned',
                                                     BaselineNed, queue_size=10)
        publishers['dops'] = rospy.Publisher(rospy.get_name() + '/dops',
                                             Dops, queue_size=10)
        publishers['gps_time'] = rospy.Publisher(rospy.get_name() + '/gps_time',
                                                 GpsTime, queue_size=10)
        publishers['pos_ecef'] = rospy.Publisher(rospy.get_name() + '/pos_ecef',
                                                 PosEcef, queue_size=10)
        # do not publish llh message, prefer publishing directly navsatfix_spp or navsatfix_rtk_fix.
        #publishers['pos_llh'] = rospy.Publisher(rospy.get_name() + '/pos_llh',
        #                                        PosLlh, queue_size=10)
        publishers['vel_ecef'] = rospy.Publisher(rospy.get_name() + '/vel_ecef',
                                                 VelEcef, queue_size=10)
        publishers['vel_ned'] = rospy.Publisher(rospy.get_name() + '/vel_ned',
                                                VelNed, queue_size=10)
        publishers['log'] = rospy.Publisher(rospy.get_name() + '/log',
                                            Log, queue_size=10)

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
            ping = subprocess.Popen(command, stdout = subprocess.PIPE)

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
        else:
            rospy.logerr(topic_name + " was not advertised correctly, it cannot be published.")

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
            if not self.base_station_mode:
                self.publishers['wifi_corrections'].publish(self.num_wifi_corrections)

        else:
            rospy.logwarn("Received external SBP msg, but Piksi not connected.")

    def navsatfix_callback(self, msg_raw, **metadata):
        msg = MsgPosLLH(msg_raw)

        self.navsatfix_msg.header.stamp = rospy.Time.now()
        self.navsatfix_msg.latitude = msg.lat
        self.navsatfix_msg.longitude = msg.lon
        self.navsatfix_msg.altitude = msg.height

        # SPP GPS messages.
        if msg.flags == 0:
            self.navsatfix_msg.status.status = NavSatStatus.STATUS_FIX
            self.navsatfix_msg.position_covariance = [self.var_spp[0], 0, 0,
                                                      0, self.var_spp[1], 0,
                                                      0, 0, self.var_spp[2]]
            self.publishers['spp'].publish(self.navsatfix_msg)

        # RTK GPS messages.
        elif msg.flags == 1 or msg.flags == 2:

            self.navsatfix_msg.status.status = NavSatStatus.STATUS_GBAS_FIX

            if msg.flags == 2:  # RTK float.
                self.navsatfix_msg.position_covariance = [self.var_rtk_float[0], 0, 0,
                                                          0, self.var_rtk_float[1], 0,
                                                          0, 0, self.var_rtk_float[2]]
                self.publishers['rtk_float'].publish(self.navsatfix_msg)
            else:  # RTK fix.
                self.navsatfix_msg.position_covariance = [self.var_rtk_fix[0], 0, 0,
                                                          0, self.var_rtk_fix[1], 0,
                                                          0, 0, self.var_rtk_fix[2]]
                self.publishers['rtk_fix'].publish(self.navsatfix_msg)

            # Update debug msg and publish.
            self.receiver_state_msg.rtk_mode_fix = True if (msg.flags == 1) else False
            self.publish_receiver_state_msg()

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

        uart_state_msg.latency_avg = msg.latency.avg
        uart_state_msg.latency_lmin = msg.latency.lmin
        uart_state_msg.latency_lmax = msg.latency.lmax
        uart_state_msg.latency_current = msg.latency.current

        self.publishers['uart_state'].publish(uart_state_msg)

# Main function.
if __name__ == '__main__':
    rospy.init_node('piksi')

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        piksi = Piksi()
    except rospy.ROSInterruptException:
        pass
