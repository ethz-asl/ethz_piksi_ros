#!/usr/bin/env python

#
#  Title:        piksi_rtk_kml.py
#  Description:  ROS node to write KML file from Piksi messages.
#

import rospy
import roslib.packages
import time
from datetime import datetime
from piksi_rtk_msgs.msg import *
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PointStamped


class PiksiRtkKml:
    kRosPackageName = "piksi_rtk_kml"
    kAvailableKmlLayouts = ["LineString", "Placemark"]

    def __init__(self):
        rospy.init_node('piksi_rtk_kml')
        # Check requested KML layout
        self.kml_layout = str(rospy.get_param('~kml_layout', self.kAvailableKmlLayouts[0]))
        if not self.kml_layout in self.kAvailableKmlLayouts:
            error_msg = "Requested KML layout %s is not available." % self.kml_layout
            rospy.logfatal(error_msg)
            rospy.signal_shutdown(error_msg)

        # KML file
        package_path = roslib.packages.get_pkg_dir(self.kRosPackageName)
        kml_file_name = datetime.utcfromtimestamp(rospy.get_time()).strftime('%Y-%m-%d-%H-%M-%S')
        self.kml_file_path = "%s/kml/%s.kml" % (package_path, kml_file_name)
        self.file_obj = open(self.kml_file_path, 'w')
        self.file_obj.write(self.kml_head(kml_file_name))

        # Settings.
        self.sampling_period = 1.0 / rospy.get_param('~sampling_frequency', 1.0)
        self.use_altitude_from_enu = rospy.get_param('~use_altitude_from_enu', False)
        self.extrude_point = rospy.get_param('~extrude_point', False)
        self.use_heading = rospy.get_param('~use_heading', True)
        self.placemarker_prefix_name = rospy.get_param('~placemarker_prefix_name', "WP")
        self.tessellate = rospy.get_param('~tessellate', 0)
        self.stick_points_to_ground = rospy.get_param('~stick_points_to_ground', False)
        self.kml_altitude_mode = rospy.get_param('~kml_altitude_mode', "absolute")

        if self.use_altitude_from_enu and self.kml_altitude_mode != "relativeToGround":
            rospy.logwarn("If you want to use altitude from enu_point_fix topic, \
                it is recommended to set parameter kml_altitude_mode = 'relativeToGround'")

        # Subscribe.
        rospy.Subscriber('piksi/navsatfix_rtk_fix', NavSatFix,
                         self.navsatfix_rtk_fix_callback)
        rospy.Subscriber('piksi/baseline_heading', BaselineHeading,
                         self.baseline_heading_callback)
        rospy.Subscriber('piksi/enu_point_fix', PointStamped,
                         self.enu_point_callback)

        # Variables.
        self.waypoint_counter = 0
        self.heading_received = False
        self.last_heading = 0.0
        self.time_last_writing = rospy.get_time()
        self.last_enu_altitude = 0.0
        self.first_navsatfix_received = False

        rospy.on_shutdown(self.close_kml_file_handler)

        rospy.spin()

    def navsatfix_rtk_fix_callback(self, msg):

        if not self.first_navsatfix_received:
            if self.kml_layout == "LineString":
                self.file_obj.write(self.kml_linestring_preamble())
            self.first_navsatfix_received = True

        if rospy.get_time() >= (self.time_last_writing + self.sampling_period):

            self.waypoint_counter = self.waypoint_counter + 1
            waypoint_name = self.placemarker_prefix_name + str(self.waypoint_counter)
            utc_time = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(msg.header.stamp.to_sec()))
            lat = msg.latitude
            lon = msg.longitude
            if self.use_altitude_from_enu:
                alt = self.last_enu_altitude
            else:
                alt = msg.altitude
            description = ''

            if self.heading_received:
                description = 'Receiver Baseline Heading: ' + str(self.last_heading / 1e3) + " [deg]."

            kml_string = ""
            if self.kml_layout == "LineString":
                kml_string = self.kml_linestring(lat, lon, alt)
            elif self.kml_layout == "Placemark":
                kml_string = self.kml_placemark(waypoint_name, utc_time, lat, lon, alt, description)

            self.file_obj.write(kml_string)

            self.time_last_writing = rospy.get_time()

    def baseline_heading_callback(self, msg):
        self.heading_received = True
        self.last_heading = msg.heading

    def enu_point_callback(self, msg):
        self.last_enu_altitude = msg.point.z

    def close_kml_file_handler(self):
        if self.kml_layout == "LineString":
            self.file_obj.write(self.kml_linestring_tail())
        self.file_obj.write(self.kml_tail())
        self.file_obj.close()
        rospy.loginfo(rospy.get_name() + ' KML file written in \'%s\'' % self.kml_file_path)

    # Adapted from: https://github.com/hitzg/bag_tools/blob/master/bag_to_kml.py
    def kml_head(self, name):
        return '''<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2" xmlns:gx="http://www.google.com/kml/ext/2.2" xmlns:kml="http://www.opengis.net/kml/2.2" xmlns:atom="http://www.w3.org/2005/Atom">
<Document>
    <name>%s</name>
''' % (name)

    # Adapted from: https://github.com/hitzg/bag_tools/blob/master/bag_to_kml.py
    def kml_tail(self):
        return '''</Document>
</kml>'''

    def kml_placemark(self, name, timestamp, lat, lon, alt=0, description=''):
        return '''
    <Placemark>
        <name>%s</name>
        <TimeStamp>
            <when>%s</when>
        </TimeStamp>
        <Point>
            <extrude>%s</extrude>
            <altitudeMode>%s</altitudeMode>
            <coordinates>%f, %f, %f</coordinates>
        </Point>
        <description>%s</description>
    </Placemark>
''' % (name, timestamp, self.extrude_point,
       self.kml_altitude_mode, lon, lat, alt, description)  # KML wants first lon and then lat.

    def kml_linestring(self, lat, lon, alt=0):
        # do not put space between numbers and comas!
        return '''
          %f,%f,%f''' % (lon, lat, alt) # KML wants first lon and then lat.

    def kml_linestring_preamble(self):
        return '''
    <Style id="yellowLineGreenPoly">
        <LineStyle>
            <color>ff00ffff</color>
            <width>25</width>
        </LineStyle>
        <PolyStyle>
            <color>7f00ff00</color>
        </PolyStyle>
    </Style>
    <Placemark>
      <name>Absolute Extruded</name>
      <description></description>
      <styleUrl>#yellowLineGreenPoly</styleUrl>
      <LineString>
        <extrude>%d</extrude>
        <tessellate>%d</tessellate>
        <altitudeMode>%s</altitudeMode>
        <coordinates>
''' % (self.extrude_point, self.tessellate, self.kml_altitude_mode)

    def kml_linestring_tail(self):
        return '''
        </coordinates>
      </LineString>
    </Placemark>
'''
