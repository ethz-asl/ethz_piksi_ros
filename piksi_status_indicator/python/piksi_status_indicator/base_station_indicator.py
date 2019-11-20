#!/usr/bin/env python

import rospy
from piksi_rtk_msgs.msg import PositionWithCovarianceStamped, ReceiverState_V2_4_1
from neostate.led_array import LEDArray


class BaseStationIndicator(object):
    def __init__(self):
        rospy.init_node("piksi_status_indicator")

        while not rospy.has_param("piksi_multi_cpp_base/num_desired_fixes"):
            rospy.logwarn_throttle(5, "[Piksi Status Indicator]: Waiting for <num_desired_fixes> param")
        self.desired_fixes_ = rospy.get_param("piksi_multi_cpp_base/num_desired_fixes")

        num_leds = rospy.get_param("~number_of_leds")
        self.indicator = LEDArray(num_leds)
        self.brightness = 0.1
        self.green_color_ = [0, 10, 0] 
        self.blue_color_ = [0, 0, 10] 

        self.kf_pos_sub_ = rospy.Subscriber(
            "/piksi_multi_cpp_base/base_station_receiver_0/position_sampler/kf_position",
            PositionWithCovarianceStamped,
            self.set_sampling_indicators,
            queue_size=1,
        )
        self.ml_pos_sub_ = rospy.Subscriber(
            "/piksi_multi_cpp_base/base_station_receiver_0/position_sampler/ml_position",
            PositionWithCovarianceStamped,
            self.set_finish_survey_indicators,
            queue_size=1,
        )
        self.recv_state_sub_ = rospy.Subscriber(
            "/piksi_multi_cpp_base/base_station_receiver_0/debug/receiver_state",
            ReceiverState_V2_4_1,
            self.set_statelllite_indicators,
            queue_size=1,
        )

    def run(self):
        rospy.loginfo("[Piksi Status Indicator]: Starting base station LED indicator..")
        rospy.spin()

    def set_sampling_indicators(self, msg):
        curr_seq = msg.header.seq

        # look for the first not blinking led:
        blink_id = next((idx for idx, val in enumerate(self.indicator.led_status) if val is 0), None)
        # Split leds depending on total desired fixes on how many leds are available
        if (blink_id is not None and
            msg.header.seq
            >= self.desired_fixes_ / self.indicator.get_num_leds() * blink_id
        ):
            # add new led blinking and set the one before to on
            self.indicator.set_led_status([blink_id], [4], [self.green_color_])
            if blink_id > 0:
                self.indicator.set_led_status([blink_id - 1], [-1], [self.green_color_])
            self.indicator.publish_led_status()

    def set_finish_survey_indicators(self, msg):
        # Survey finished. Set last led to on and wait for 3 seconds
        self.indicator.set_led_status(
            [self.indicator.get_num_leds() - 1], [-1], [self.green_color_]
        )
        self.indicator.publish_led_status()
        rospy.sleep(3)

    def set_statelllite_indicators(self, msg):
        pass
