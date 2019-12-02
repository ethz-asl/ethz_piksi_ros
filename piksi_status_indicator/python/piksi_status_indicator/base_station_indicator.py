#!/usr/bin/env python

import rospy

from piksi_rtk_msgs.msg import PositionWithCovarianceStamped, ReceiverState_V2_6_5
from neostate.led_array import LEDArray


class BaseStationIndicator(object):
    """Base Station Indicator using the Neostate package.

    During survey the leds will increasingly turn green until the sampling has finished.
    Once finished the leds will stay green for 3 seconds and then use blue indicators
    to show how many satellites the base station is seeing (more than 35 satellites results
    in all leds turned on).
    If the blue indicators are blinking, then the base station fixed mode is not in SBAS. 
    """
    def __init__(self):
        rospy.init_node("piksi_status_indicator")

        while not rospy.has_param("piksi_multi_cpp_base/num_desired_fixes"):
            rospy.logwarn_throttle(5, "[Piksi Status Indicator]: Waiting for <num_desired_fixes> param")
        self.desired_fixes_ = rospy.get_param("piksi_multi_cpp_base/num_desired_fixes")

        num_leds = rospy.get_param("~number_of_leds")
        self.indicator = LEDArray(num_leds)
        
        self.finished_sampling = False
        self.finished_survey = False

        self.green_color_ = [0, 155, 0] 
        self.blue_color_ = [0, 0, 155] 
        
        #TODO(clanegge): Search for namespace of base station
        self.kf_pos_sub_ = rospy.Subscriber(
            "/piksi_multi_cpp_base/base_station_receiver_0/position_sampler/kf_position",
            PositionWithCovarianceStamped,
            self.set_sampling_indicators,
            queue_size=1,
        )

        self.recv_state_sub_ = rospy.Subscriber(
            "piksi_multi_cpp_base/base_station_receiver_0/ros/receiver_state",
            ReceiverState_V2_6_5,
            self.set_satellite_indicators,
            queue_size=1,
        )

        # Turn off all leds to indicate script is running
        self.indicator.turn_off_all_leds()


    def run(self):
        rospy.loginfo("[Piksi Status Indicator]: Starting base station LED indicator..")
        rospy.spin()
    
    def get_next_off_led(self):
        return  next((idx for idx, val in enumerate(self.indicator.led_status) if val is 0), None)

    def set_sampling_indicators(self, msg):
        if msg.header.seq == (self.desired_fixes_ -1):
            # Sampling finished 
            self.finished_sampling = True
            self.ml_pos_sub_ = rospy.Subscriber(
            "/piksi_multi_cpp_base/base_station_receiver_0/position_sampler/ml_position",
            PositionWithCovarianceStamped,
            self.set_finish_survey_indicators,
            queue_size=1,
            )

        if not self.finished_sampling:
            # look for the first not blinking led:
            next_id = self.get_next_off_led()
            # Split leds depending on total desired fixes on how many leds are available
            # as long as there are still some leds which are off
            if (next_id is not None and
                msg.header.seq
                >= self.desired_fixes_ / self.indicator.get_num_leds() * next_id
            ):
                # add new led blinking and set the one before to on
                self.indicator.set_led_status([next_id], [4], [self.green_color_])
                if next_id > 0:
                    self.indicator.set_led_status([next_id - 1], [-1], [self.green_color_])
                self.indicator.publish_led_status()

            if msg.header.seq is self.desired_fixes_ -1:
                # Survey finished
                self.finished_sampling = True

    def set_finish_survey_indicators(self, msg):
        # Set last led to on and wait for 3 seconds
        num_leds = self.indicator.get_num_leds()
        self.indicator.set_led_status(
            list(range(0,num_leds)), [-1] * num_leds, [self.green_color_] * num_leds
        )
        self.indicator.publish_led_status()
        rospy.sleep(3)
        # Survey finsihed can use leds for something else now
        self.indicator.turn_off_all_leds()
        self.finished_survey = True

    def set_satellite_indicators(self, msg):
        if(self.finished_survey):

            # Assume that 35 satellites is already a lot and threshold for all leds to be on
            if msg.num_sat > 35:
                desired_num_leds_on = self.indicator.get_num_leds()
            else:
                desired_num_leds_on = int(float(msg.num_sat)/35 * self.indicator.get_num_leds())
            last_on_led_id = self.get_next_off_led()
            if last_on_led_id is None:
                last_on_led_id = self.indicator.get_num_leds()
            # Check if leds are blinking or not
            change_blinking_status = False
            #TODO(clanegge): This does not seem to work
            """
            if self.indicator.led_status[0] > 0:
                # leds are blinking
                if msg.fix_mode == "SBAS":
                    change_blinking_status = True
            elif self.indicator.led_status[0] < 0:
                # leds are not blinking
                if msg.fix_mode != "SBAS":
                    change_blinking_status = True
            """
            # Change status only if required (change number of leds or change blinking status)
            if desired_num_leds_on != last_on_led_id or change_blinking_status:
                # Turn on desired leds
                led_status = [-1] * desired_num_leds_on
                
                # if we should blink leds change sign
                if change_blinking_status:
                    led_status = led_status * (-1)

                # define led ids (here just the range from 0 to desired led)
                led_id = list(range(0,desired_num_leds_on))
                
                # update and publish
                self.indicator.set_led_status(led_id, led_status, [self.blue_color_] * desired_num_leds_on)
                self.indicator.publish_led_status()

