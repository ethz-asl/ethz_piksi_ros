//
// Created by mpantic on 07.03.19.
//

#ifndef RQT_GPS_RTK_PLUGIN_SBPSTREAMDECODER_HPP
#define RQT_GPS_RTK_PLUGIN_SBPSTREAMDECODER_HPP
#include <vector>
#include <rqt_gps_rtk_plugin/SBPDecoder.hpp>

class SBPStreamDecoder {

  enum class ReceiverState : uint8_t {
    Ready = 0,  // nothing received yet.
    Synced = 1, // Sync token (0x55) received.
    HeaderFound = 2,  // complete header received.
    Received = 3  // message complete.
  };
 public:

  SBPStreamDecoder(const size_t buffer_size,
                   const std::set<SBP_MSG_TYPE>& active_types);

  // add a single byte and check if a valid message is in the buffer
  SBP_MSG_TYPE addToBuffer(uint8_t& data);

  // get current valid message
  // function defined in header due to templating implementation
  template<typename T>
  bool getCurrentMessage(T* out) {
    if (state_ != ReceiverState::Received) {
      return false;
    }

    bool success = SBPDecoder::decode<T>(buffer_, out);
    reset();
    return success;
  }

 private:

  // State machine transitions from state Ready.
  ReceiverState transitionFromReady();

  // State machine transitions from state Synched.
  ReceiverState transitionSynced();

  // State machine transitions from state Found.
  ReceiverState transitionHeaderFound();

  // Advance state machine..
  void checkStateTransitions();

  // reset state machine
  void reset();

  // handle buffer pointer
  inline void advanceBuffer();

  const std::set<SBP_MSG_TYPE> active_types_;
  std::vector<uint8_t> buffer_;
  size_t buffer_ptr_;
  ReceiverState state_;
  SBP_MSG_HEADER current_header_;

};

#endif //RQT_GPS_RTK_PLUGIN_SBPSTREAMDECODER_HPP
