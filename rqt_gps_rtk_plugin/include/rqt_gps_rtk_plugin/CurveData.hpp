#pragma once

#include <boost/circular_buffer.hpp>
#include <QVector>

namespace rqt_gps_rtk_plugin {

  class CurveData
  {
   public:
    CurveData();
    ~CurveData() {};

    void appendSignalValue(const std::pair<double, double>& valuePair);
    void setMaxSignalValues(const int& maxValue);
    QVector<std::pair<double, double>> getSignalValues();

   private:

    // circular buffer containing the pair <time stamp, signal value>
    boost::circular_buffer<std::pair<double, double>> signalValues_;
    int maxSignalValues_;
  };

}  // namespace
