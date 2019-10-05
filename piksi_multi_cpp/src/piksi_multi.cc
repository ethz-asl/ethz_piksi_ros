#include "piksi_multi_cpp/piksi_multi.h"

#include <libsbp/sbp.h>

namespace piksi_multi_cpp {

PiksiMulti::PiksiMulti(const ros::NodeHandle &nh,
                       const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private) {
  getROSParameters();
  advertiseTopics();
}

void PiksiMulti::getROSParameters() {}

void PiksiMulti::advertiseTopics() {}

} // namespace piksi_multi_cpp
