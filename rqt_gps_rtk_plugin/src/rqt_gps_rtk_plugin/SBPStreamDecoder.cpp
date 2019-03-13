#include <rqt_gps_rtk_plugin/SBPStreamDecoder.hpp>

SBPStreamDecoder::SBPStreamDecoder(const size_t buffer_size,
                                   const std::set<SBP_MSG_TYPE>& active_types) :
    active_types_(active_types),
    buffer_(buffer_size),
    buffer_ptr_(0),
    state_(ReceiverState::Ready) {
}

// add a single byte and check if a valid message is in the buffer
SBP_MSG_TYPE SBPStreamDecoder::addToBuffer(uint8_t& data) {
  buffer_[buffer_ptr_] = data;
  checkStateTransitions();
  advanceBuffer();

  if (state_ != ReceiverState::Received) {
    return SBP_MSG_TYPE::INVALID;
  } else {
    return current_header_.message_type;
  }
}

// State machine transitions from state Ready.
SBPStreamDecoder::ReceiverState SBPStreamDecoder::transitionFromReady() {
  if (buffer_ptr_ == 0 &&
      SBPDecoder::isSync(buffer_[buffer_ptr_])) {
    return ReceiverState::Synced;
  } else {
    return ReceiverState::Ready;
  }
}

// State machine transitions from state Synched.
SBPStreamDecoder::ReceiverState SBPStreamDecoder::transitionSynced() {
  if (buffer_ptr_ >= sizeof(SBP_MSG_HEADER)) {

    if (SBPDecoder::validHeader(buffer_, &current_header_, active_types_)) {
      return ReceiverState::HeaderFound;
    } else {
      reset();
      return ReceiverState::Ready; //lost sync, no valid header
    }
  } else {
    return ReceiverState::Synced; // not enough data for header yet
  }
}

// State machine transitions from state Found.
SBPStreamDecoder::ReceiverState SBPStreamDecoder::transitionHeaderFound() {
  if (buffer_ptr_ >= SBPDecoder::getMessageSize(current_header_)) {
    if (SBPDecoder::checkMessage(buffer_)) {
      return ReceiverState::Received;
    } else {
      reset();
      return ReceiverState::Ready; // nothing received, reset
    }
  } else {
    return ReceiverState::HeaderFound; // not enough data yet
  }
}

// Advance state machine..
void SBPStreamDecoder::checkStateTransitions() {
  if (state_ == ReceiverState::Ready) {
    state_ = transitionFromReady();
  } else if (state_ == ReceiverState::Synced) {
    state_ = transitionSynced();
  } else if (state_ == ReceiverState::HeaderFound) {
    state_ = transitionHeaderFound();
  } else {
    // any other case (e.g. we are already in received, go back to ready)
    reset();
  }
}

// reset state machine
void SBPStreamDecoder::reset() {
  state_ = ReceiverState::Ready;
  buffer_ptr_ = 0;
  std::fill(buffer_.begin(), buffer_.end(), 0x00);
}

// handle buffer pointer
inline void SBPStreamDecoder::advanceBuffer() {
  if (buffer_ptr_ >= buffer_.size() - 1) {
    reset();
    return;
  }

  if (state_ == ReceiverState::Ready) {
    buffer_ptr_ = 0;
  } else {
    buffer_ptr_++;
  }
}
