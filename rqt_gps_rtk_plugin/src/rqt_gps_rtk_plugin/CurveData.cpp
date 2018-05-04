#include "rqt_gps_rtk_plugin/CurveData.hpp"

namespace rqt_gps_rtk_plugin {

  CurveData::CurveData()
      : maxSignalValues_(10) {
    signalValues_.empty();
    signalValues_.set_capacity(maxSignalValues_);
  }

  void CurveData::setMaxSignalValues(const int& maxValue) {
    maxSignalValues_ = maxValue;
    signalValues_.set_capacity(maxValue);
  }

  void CurveData::appendSignalValue(const std::pair<double, double>& valuePair) {
    signalValues_.push_back(valuePair);
  }

  QVector<std::pair<double, double>> CurveData::getSignalValues() {
    QVector<std::pair<double, double>> values;
    for (const auto& value : signalValues_) {
      values.push_back(value);
    }
    return values;
  }

}  // namespace
