#!/usr/bin/env python

import rospy
from piksi_rtk_msgs.msg import PositionWithCovarianceStamped, ReceiverState_V2_4_1
from neostate.msg import StatusLEDArray, StatusLED


class LEDArray(object):
    def __init__(self, num_leds):
        self.__num_of_leds = num_leds

        # LED status:
        #    0: off
        #   -1: on
        #   >0: blinking at frequency specified in array
        self.__led_status = [0] * self.__num_of_leds

        # Store RGB value for every LED
        self.__led_colors = [[0, 0, 0]] * self.__num_of_leds

    @property
    def led_status(self):
        return self.__led_status

    @property
    def led_colors(self):
        return self.__led_colors

    def set_led_status(self, ids, status, rgb_color):
        if len(ids) is not len(status) is not len(rgb_color):
            rospy.logwarn(
                "[Piksi Status Indicator]: " 
                + "Cannot assign values to ids as their array size is not the same."
                + "Not changing LED status."
            )
            return
        elif len(ids) > len(self.__led_status):
            rospy.logwarn(
                "[Piksi Status Indicator]:"
                + "It is not possible to assign more values than leds."
                + "Not changing LED status."
            )
            return

        for id, stat_val, color in zip(ids, status, rgb_color):
            self.__led_status[id] = stat_val
            if stat_val is not 0:
                rospy.logwarn("Setting led to color %i", id)
                self.__led_colors[id] = color
            else:
                rospy.logwarn("turning of led %i", id)
                # turn off led
                self.__led_colors[id] = [0, 0, 0]

    def get_num_leds(self):
        return self.__num_of_leds


class BaseStationIndicator(object):
    def __init__(self):
        rospy.init_node("piksi_status_indicator")
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

        self.ind_pub_ = rospy.Publisher("set_status_led_array", StatusLEDArray, queue_size=1, latch=True)

        while not rospy.has_param("piksi_multi_cpp_base/num_desired_fixes"):
            rospy.logwarn_throttle(5, "[Piksi Status Indicator]: Waiting for <num_desired_fixes> param")
        self.desired_fixes_ = rospy.get_param("piksi_multi_cpp_base/num_desired_fixes")

        num_leds = rospy.get_param("~number_of_leds")
        self.indicator = LEDArray(num_leds)
        self.brightness = 0.1
        self.green_color_ = [0, 10, 0] 
        self.blue_color_ = [0, 0, 10] 

    def run(self):
        rospy.loginfo("[Piksi Status Indicator]: Starting base station LED indicator..")
        rospy.spin()

    def update_indicator_state(self):
        msg = StatusLEDArray()
        for status, color in zip(
            self.indicator.led_status, self.indicator.led_colors
        ):
            led_stat_msg = StatusLED()
            led_stat_msg.red = color[0]
            led_stat_msg.green = color[1]
            led_stat_msg.blue = color[2]
            if status > 0:
                led_stat_msg.blinking = True
            else:
                led_stat_msg.blinking = False
            msg.LED_array.append(led_stat_msg)
        self.ind_pub_.publish(msg)

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
            self.update_indicator_state()

    def set_finish_survey_indicators(self, msg):
        # Survey finished. Set last led to on and wait for 3 seconds
        self.indicator.set_led_status(
            [self.indicator.get_num_leds() - 1], [-1], [self.green_color_]
        )
        self.update_indicator_state()
        rospy.sleep(3)

    def set_statelllite_indicators(self, msg):
        pass
