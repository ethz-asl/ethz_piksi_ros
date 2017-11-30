#!/usr/bin/env python

#
#  Title:        geodetic_survey.py
#  Description:  ROS node to average gps coordinates of an RTK base station.
#
import rospy
import math
# Import message types
from sensor_msgs.msg import NavSatFix, NavSatStatus


class GeodeticSurvey:
    def __init__(self):
        # Print info.
        rospy.sleep(0.5)  # wait for a while for init to complete before printing
        rospy.loginfo(rospy.get_name() + " start")
        # Register callback if you want to end node before time
        rospy.on_shutdown(self.shutdown_callback)

        # Subscribe
        rospy.Subscriber("navsatfix", NavSatFix,
                         self.navsatfix_callback)

        # Parameters.
        self.number_samples = rospy.get_param('~number_samples', 6000)

        # Init accumulator.
        self.lat_acc = 0.0
        self.lon_acc = 0.0
        self.alt_acc = 0.0
        self.received_samples = 0

        # Spin.
        rospy.spin()

    def shutdown_callback(self):
        self.compute_geodetic_position()

    def navsatfix_callback(self, msg):
        # Make sure data in is valid.
        if math.isnan(msg.latitude) or math.isnan(msg.longitude) or math.isnan(msg.altitude):
            rospy.logerr("Received not valid navsatifx message containing NAN data. Sample discarded.")
            return

        self.lat_acc += msg.latitude
        self.lon_acc += msg.longitude
        self.alt_acc += msg.altitude
        self.received_samples += 1

        rospy.loginfo("Received: [%.10f, %.10f, %.3f]; waiting for %d samples" % (
            msg.latitude, msg.longitude, msg.altitude, self.number_samples - self.received_samples))

        if self.received_samples >= self.number_samples:
            self.compute_geodetic_position()

    def compute_geodetic_position(self):
        if self.received_samples < 1:
            rospy.signal_shutdown("At least one sample is needed to compute base station position.")
            return

        lat = self.lat_acc / self.received_samples
        lon = self.lon_acc / self.received_samples
        alt = self.alt_acc / self.received_samples

        rospy.loginfo("+++ Final geodetic position: [%.10f, %.10f, %.3f], computed using %d samples +++" % (
            lat, lon, alt, self.received_samples))
        rospy.signal_shutdown("")


# Main function.
if __name__ == '__main__':
    rospy.init_node('geodetic_survey')

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        geodetic_survey = GeodeticSurvey()
    except rospy.ROSInterruptException:
        pass
