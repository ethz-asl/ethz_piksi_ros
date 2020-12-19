#ifndef PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_EPHEMERIS_CALLBACK_HANDLER_H_
#define PIKSI_MULTI_CPP_SBP_CALLBACK_HANDLER_SBP_EPHEMERIS_CALLBACK_HANDLER_H_
#include <piksi_multi_cpp/sbp_callback_handler/sbp_lambda_callback_handler.h>
#include <piksi_multi_cpp/sbp_callback_handler/sbp_msg_type_callback_handler.h>
#include <ros/ros.h>

namespace piksi_multi_cpp {

/*
 * Subscribes to all 3 observation type callbacks simultaneously using
 * LambdaCallbackHandlers and redirects these messages to all registered
 * listeners.
 *
 */
class SBPEphemerisCallbackHandler
    : public SBPMsgTypeCallbackHandler {
 public:
  SBPEphemerisCallbackHandler(const ros::NodeHandle nh,
                                const std::shared_ptr<sbp_state_t>& state)
      : SBPMsgTypeCallbackHandler(nh, state),
        gps_ephemeris_handler_{getCallback<msg_ephemeris_gps_t>(),
                          SBP_MSG_EPHEMERIS_GPS, state},
        glo_ephemeris_handler_{getCallback<msg_ephemeris_glo_t>(), SBP_MSG_EPHEMERIS_GLO,
                          state},
        iono_delay_handler_{getCallback<msg_iono_t>(), SBP_MSG_IONO, state} {};

 private:
  /* Set up Lambda redirect callbacks for the three message types used for
   * converting sbp to rinex, such that this class gets any of the three callbacks and can
   * forward them to the observation listeners */
  SBPLambdaCallbackHandler<msg_ephemeris_gps_t> gps_ephemeris_handler_;
  SBPLambdaCallbackHandler<msg_ephemeris_glo_t> glo_ephemeris_handler_;
  SBPLambdaCallbackHandler<msg_iono_t> iono_delay_handler_;
};
}  // namespace piksi_multi_cpp

#endif
