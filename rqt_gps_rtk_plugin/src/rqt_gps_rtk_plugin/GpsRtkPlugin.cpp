#include "../include/rqt_gps_rtk_plugin/GpsRtkPlugin.hpp"

#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <QGridLayout>

// std
#include <math.h>
#include <pthread.h>
#include <functional>

GpsRtkPlugin::GpsRtkPlugin()
    : rqt_gui_cpp::Plugin(),
      widget_(0),
      timeFirstSampleMovingWindow_(0),
      wifiCorrectionsAvgHz_(0),
      numCorrectionsFirstSampleMovingWindow_(0),
      maxTimeout_(5) {
  // Constructor is called first before initPlugin function, needless to say.

  // give QObjects reasonable names
  setObjectName("GpsRtkPlugin");
}

void GpsRtkPlugin::initPlugin(qt_gui_cpp::PluginContext& context) {
  // access standalone command line arguments
  QStringList argv = context.argv();
  // create QWidget
  widget_ = new QWidget();
  // extend the widget with all attributes and children from UI file
  ui_.setupUi(widget_);
  // add widget to the user interface
  context.addWidget(widget_);

  readParameters();
  initLabels();
  initSubscribers();

  //Init variables
  timeFirstSampleMovingWindow_ = ros::Time::now().sec;
  wifiCorrectionsAvgHz_ = 5;
  numCorrectionsFirstSampleMovingWindow_ = 0;
  altitudes_.erase(altitudes_.begin(), altitudes_.end());

  // initialize update worker
  updateWorker_.reset(std::make_unique(any_worker::Worker("GpsRtkPlugin::updateWorker", 1, std::bind(&GpsRtkPlugin::updateWorkerCb, this, std::placeholders::_1))));
  if (!updateWorker_->start()) {
    ROS_WARN_STREAM("[GpsRtkPlugin] Update worker could not be started. GUI information not properly updated.");
  }

  //init stamps
  lastMsgStamps_.setGlobalStamp(ros::Time::now().toSec());

  ROS_INFO("[GpsRtkPlugin] Initialized.");
}

void GpsRtkPlugin::shutdownPlugin() {
}

void GpsRtkPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const {
}

void GpsRtkPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings) {
}

void GpsRtkPlugin::readParameters() {
  getNodeHandle().param<std::string>("piksiReceiverStateTopic", piksiReceiverStateTopic_, "piksi/debug/receiver_state");
  ROS_INFO_STREAM("[GpsRtkPlugin] piksiReceiverStateTopic: " << piksiReceiverStateTopic_);

  getNodeHandle().param<std::string>("piksiBaselineNedTopic", piksiBaselineNedTopic_, "piksi/baseline_ned");
  ROS_INFO_STREAM("[GpsRtkPlugin] piksiBaselineNedTopic: " << piksiBaselineNedTopic_);

  getNodeHandle().param<std::string>("piksiWifiCorrectionsTopic", piksiWifiCorrectionsTopic_, "piksi/debug/wifi_corrections");
  ROS_INFO_STREAM("[GpsRtkPlugin] piksiWifiCorrectionsTopic: " << piksiWifiCorrectionsTopic_);

  getNodeHandle().param<std::string>("piksiNavsatfixRtkFixTopic", piksiNavsatfixRtkFixTopic_, "piksi/navsatfix_rtk_fix");
  ROS_INFO_STREAM("[GpsRtkPlugin] piksiNavsatfixRtkFixTopic: " << piksiNavsatfixRtkFixTopic_);

  getNodeHandle().param<std::string>("piksiTimeTopic", piksiTimeTopic_, "piksi/utc_time");
  ROS_INFO_STREAM("[GpsRtkPlugin] piksiTimeTopic: " << piksiTimeTopic_);

  getNodeHandle().param<std::string>("piksiAgeOfCorrectionsTopic", piksiAgeOfCorrectionsTopic_, "piksi/age_of_corrections");
  ROS_INFO_STREAM("[GpsRtkPlugin] piksiAgeOfCorrectionsTopic: " << piksiAgeOfCorrectionsTopic_);
}

void GpsRtkPlugin::initLabels() {

  // Tab "Status"
  ui_.label_nodeStatus->setText("N/A");
  ui_.label_baseline->setText("N/A");
  ui_.label_fixType->setText("N/A");
  ui_.label_navsatFixAlt->setText("N/A");
  ui_.label_numRtkSatellites->setText("N/A");
  ui_.label_numRtkSatellites_indicator->setText("");
  ui_.label_numSatellites->setText("N/A");
  ui_.label_numWifiCorrections->setText("N/A");
  ui_.label_pingBaseStation->setText("N/A");
  ui_.label_rateWifiCorrections->setText("N/A");
  ui_.label_ageOfCorrections->setText("N/A");

  // Tab "Debug"
  ui_.label_gpsSatellites->setText("N/A");
  ui_.label_gpsStrength->setText("N/A");
  ui_.label_sbasSatellites->setText("N/A");
  ui_.label_sbasStrength->setText("N/A");
  ui_.label_glonassSatellites->setText("N/A");
  ui_.label_glonassStrength->setText("N/A");
  ui_.label_ageOfCorrections->setText("N/A");
}

void GpsRtkPlugin::initSubscribers() {
  piksiReceiverStateSub_ = getNodeHandle().subscribe(piksiReceiverStateTopic_, 10, &GpsRtkPlugin::piksiReceiverStateCb, this);
  piksiBaselineNedSub_ = getNodeHandle().subscribe(piksiBaselineNedTopic_, 10, &GpsRtkPlugin::piksiBaselineNedCb, this);
  piksiWifiCorrectionsSub_ = getNodeHandle().subscribe(piksiWifiCorrectionsTopic_, 10, &GpsRtkPlugin::piksiWifiCorrectionsCb, this);
  piksiNavsatfixRtkFixSub_ = getNodeHandle().subscribe(piksiNavsatfixRtkFixTopic_, 10, &GpsRtkPlugin::piksiNavsatfixRtkFixCb, this);
  piksiHeartbeatSub_ = getNodeHandle().subscribe(piksiTimeTopic_, 10, &GpsRtkPlugin::piksiTimeCb, this);
  piksiAgeOfCorrectionsSub_ = getNodeHandle().subscribe(piksiAgeOfCorrectionsTopic_, 10, &GpsRtkPlugin::piksiAgeOfCorrectionsCb, this);
}

bool GpsRtkPlugin::updateWorkerCb(const any_worker::WorkerEvent& event) {
  double currentStamp = ros::Time::now().toSec();
  QString na = QString::fromStdString("N/A");
  QString color = QString::fromStdString("QLabel {background-color: rgb(152, 152, 152); color: rgb(92, 92, 92);}");
  if (currentStamp - lastMsgStamps_.receiverStateStamp_ > maxTimeout_) {
    QMetaObject::invokeMethod(ui_.label_fixType, "setText", Q_ARG(QString, na));
    QMetaObject::invokeMethod(ui_.label_fixType, "setStyleSheet", Q_ARG(QString, color));
    // Number of satellites
    QMetaObject::invokeMethod(ui_.label_numSatellites, "setText", Q_ARG(QString, na));
    // GPS number of satellites
    QMetaObject::invokeMethod(ui_.label_gpsSatellites, "setText", Q_ARG(QString, na));
    // GPS signal strength
    QMetaObject::invokeMethod(ui_.label_gpsStrength, "setText", Q_ARG(QString, na));
    // SBAS number of satellites
    QMetaObject::invokeMethod(ui_.label_sbasSatellites, "setText", Q_ARG(QString, na));
    // SBAS signal strength
    QMetaObject::invokeMethod(ui_.label_sbasStrength, "setText", Q_ARG(QString, na));
    // GLONASS number of satellites
    QMetaObject::invokeMethod(ui_.label_glonassSatellites, "setText", Q_ARG(QString, na));
    // GLONASS signal strength
    QMetaObject::invokeMethod(ui_.label_glonassStrength, "setText", Q_ARG(QString, na));
  }
  if (currentStamp - lastMsgStamps_.baselineNedStamp_ > maxTimeout_) {
    QMetaObject::invokeMethod(ui_.label_numRtkSatellites_indicator, "setText", Q_ARG(QString, na));
    QMetaObject::invokeMethod(ui_.label_numRtkSatellites_indicator, "setStyleSheet", Q_ARG(QString, color));
    QMetaObject::invokeMethod(ui_.label_baseline, "setText", Q_ARG(QString, na));
  }
  if (currentStamp - lastMsgStamps_.wifiCorrectionsStamp_ > maxTimeout_) {
    QMetaObject::invokeMethod(ui_.label_numWifiCorrections, "setText", Q_ARG(QString, na));
    QMetaObject::invokeMethod(ui_.label_rateWifiCorrections, "setText", Q_ARG(QString, na));
    QMetaObject::invokeMethod(ui_.label_pingBaseStation, "setText", Q_ARG(QString, na));
  }
  if (currentStamp - lastMsgStamps_.navsatfixRtkFixStamp_ > maxTimeout_) {
    QMetaObject::invokeMethod(ui_.label_navsatFixAlt, "setText", Q_ARG(QString, na));
  }
  return true;
}

void GpsRtkPlugin::piksiReceiverStateCb(const piksi_rtk_msgs::ReceiverState_V2_3_15& msg) {
  // Type of fix
  const QString fix_mode = QString::fromStdString(msg.fix_mode);
  QString color_fix_mode("");

  // Choose color for type of fix
  if (msg.fix_mode == msg.STR_FIX_MODE_INVALID) {
    color_fix_mode = "QLabel {background-color: rgb(239, 41, 41); color: rgb(92, 92, 92);}";
  } else if (msg.fix_mode == msg.STR_FIX_MODE_SPP) {
    color_fix_mode = "QLabel {background-color: rgb(255, 255, 255); color: rgb(2, 2, 255);}";
  } else if (msg.fix_mode == msg.STR_FIX_MODE_DGNSS) {
    color_fix_mode = "QLabel {background-color: rgb(255, 255, 255); color: rgb(5, 181, 255);}";
  } else if (msg.fix_mode == msg.STR_FIX_MODE_FLOAT_RTK) {
    color_fix_mode = "QLabel {background-color: rgb(255, 138, 138); color: rgb(191, 0, 191);}";
  } else if (msg.fix_mode == msg.STR_FIX_MODE_FIXED_RTK) {
    color_fix_mode = "QLabel {background-color: lime; color: rgb(255, 166, 2);}";
  } else if (msg.fix_mode == msg.STR_FIX_MODE_SBAS) {
    color_fix_mode = "QLabel {background-color: rgb(255, 255, 255); color: rgb(43, 255, 223);}";
  }
  else {
    // Unknown
    color_fix_mode = "QLabel {background-color: rgb(152, 152, 152); color: rgb(92, 92, 92);}";
  }

  QMetaObject::invokeMethod(ui_.label_fixType, "setText", Q_ARG(QString, fix_mode));
  /*
   * use 'best' available rtk state for the label
  if (msg.fix_mode == msg.STR_FIX_MODE_SPP or msg.fix_mode == msg.STR_FIX_MODE_SBAS or msg.fix_mode == msg.STR_FIX_MODE_FLOAT_RTK or msg.fix_mode == msg.STR_FIX_MODE_FIXED_RTK) {
    double currentStamp = ros::Time::now().toSec();
    if (currentStamp - lastMsgStamps_.rtkFixStamp_ < 2.0) {
      color_fix_mode = "QLabel {background-color: lime; color: rgb(255, 166, 2);}";
    } else if (currentStamp - lastMsgStamps_.rtkFloatStamp_ < 2.0) {
      color_fix_mode = "QLabel {background-color: rgb(255, 138, 138); color: rgb(191, 0, 191);}";
    } else if (currentStamp - lastMsgStamps_.rtkSbasStamp_ < 2.0) {
      color_fix_mode = "QLabel {background-color: rgb(255, 255, 255); color: rgb(43, 255, 223);}";
    } else {
      color_fix_mode = "QLabel {background-color: rgb(255, 255, 255); color: rgb(2, 2, 255);}";
    }
  }
  */
  QMetaObject::invokeMethod(ui_.label_fixType, "setStyleSheet", Q_ARG(QString, color_fix_mode));
  // Number of satellites
  QMetaObject::invokeMethod(ui_.label_numSatellites, "setText", Q_ARG(QString, QString::number(msg.num_sat)));
  // GPS number of satellites
  QMetaObject::invokeMethod(ui_.label_gpsSatellites, "setText", Q_ARG(QString, QString::number(msg.num_gps_sat)));
  // GPS signal strength
  QString signal_strength;
  vectorToString(scaleSignalStrength(msg.cn0_gps), &signal_strength);
  QMetaObject::invokeMethod(ui_.label_gpsStrength, "setText", Q_ARG(QString, signal_strength));
  // SBAS number of satellites
  QMetaObject::invokeMethod(ui_.label_sbasSatellites, "setText", Q_ARG(QString, QString::number(msg.num_sbas_sat)));
  // SBAS signal strength
  vectorToString(scaleSignalStrength(msg.cn0_sbas), &signal_strength);
  QMetaObject::invokeMethod(ui_.label_sbasStrength, "setText", Q_ARG(QString, signal_strength));
  // GLONASS number of satellites
  QMetaObject::invokeMethod(ui_.label_glonassSatellites, "setText", Q_ARG(QString, QString::number(msg.num_glonass_sat)));
  // GLONASS signal strength
  vectorToString(scaleSignalStrength(msg.cn0_glonass), &signal_strength);
  QMetaObject::invokeMethod(ui_.label_glonassStrength, "setText", Q_ARG(QString, signal_strength));

  //update last msg stamp
  lastMsgStamps_.receiverStateStamp_ = msg.header.stamp.toSec();
}

void GpsRtkPlugin::piksiBaselineNedCb(const piksi_rtk_msgs::BaselineNed& msg) {
  // Number of satellites used for RTK
  QMetaObject::invokeMethod(ui_.label_numRtkSatellites, "setText", Q_ARG(QString, QString::number(msg.n_sats)));

  std::string strStyle;
  std::string strText;
  if (msg.n_sats < 5) {
    strStyle = "QLabel {background-color: red;}";
    strText = "bad";
  } else if (msg.n_sats >= 5 and msg.n_sats <= 6) {
    strStyle = "QLabel {background-color: orange;}";
    strText = "ok";
  } else if (msg.n_sats > 6) {
    strStyle = "QLabel {background-color: rgb(78, 154, 6);}";
    strText = "good";
  }
  QMetaObject::invokeMethod(ui_.label_numRtkSatellites_indicator, "setText", Q_ARG(QString, QString::fromStdString(strText)));
  QMetaObject::invokeMethod(ui_.label_numRtkSatellites_indicator, "setStyleSheet", Q_ARG(QString, QString::fromStdString(strStyle)));

  // Baseline NED status
  double n = msg.n / 1e3;
  double e = msg.e / 1e3;
  double d = msg.d / 1e3;
  QString baseline;  // = "[" + QString::fromStdString(std::to_string(roundf(n*100)/100)) + ", " + QString::fromStdString(std::to_string(roundf(e*100)/100)) + ", " + QString::fromStdString(std::to_string(roundf(d*100)/100)) + "]";
  baseline.sprintf("[%.2f, %.2f, %.2f]", roundf(n * 100) / 100, roundf(e * 100) / 100, roundf(d * 100) / 100);
  QMetaObject::invokeMethod(ui_.label_baseline, "setText", Q_ARG(QString, baseline));

  //update last msg stamp
  lastMsgStamps_.baselineNedStamp_ = msg.header.stamp.toSec();
}

void GpsRtkPlugin::piksiWifiCorrectionsCb(const piksi_rtk_msgs::InfoWifiCorrections& msg) {
  // Number of received corrections
  QMetaObject::invokeMethod(ui_.label_numWifiCorrections, "setText", Q_ARG(QString, QString::number(msg.received_corrections)));

  // Compute rate corrections
  double widthTimeWindow = ros::Time::now().sec - timeFirstSampleMovingWindow_;
  if (widthTimeWindow >= wifiCorrectionsAvgHz_) {
    int samplesInTimeWindow = msg.received_corrections - numCorrectionsFirstSampleMovingWindow_;
    double avgCorrectionsHz = samplesInTimeWindow / widthTimeWindow;
    QMetaObject::invokeMethod(ui_.label_rateWifiCorrections, "setText", Q_ARG(QString, QString::number(std::round(avgCorrectionsHz), 'g', 2)));

    // reset and start anew
    numCorrectionsFirstSampleMovingWindow_ = msg.received_corrections;
    timeFirstSampleMovingWindow_ = ros::Time::now().sec;
  }

  // Ping base station
  QMetaObject::invokeMethod(ui_.label_pingBaseStation, "setText", Q_ARG(QString, QString::number(std::round(msg.latency), 'g', 2)));

  //update last msg stamp
  lastMsgStamps_.wifiCorrectionsStamp_ = msg.header.stamp.toSec();
}

void GpsRtkPlugin::piksiNavsatfixRtkFixCb(const sensor_msgs::NavSatFix& msg) {
  altitudes_.push_back(msg.altitude);
  double altitudeAvg = 0;
  for (std::vector<double>::iterator it = altitudes_.begin(); it != altitudes_.end(); it++) {
    altitudeAvg += *it;
  }
  altitudeAvg /= altitudes_.size();
  QString text;
  text.sprintf("%.2f", altitudeAvg);
  QMetaObject::invokeMethod(ui_.label_navsatFixAlt, "setText", Q_ARG(QString, text));

  //update last msg stamp
  lastMsgStamps_.navsatfixRtkFixStamp_ = msg.header.stamp.toSec();
}

void GpsRtkPlugin::piksiTimeCb(const piksi_rtk_msgs::UtcTimeMulti& msg) {
  //std::string time;
  QString time;
  time.sprintf("%02d:%02d:%02d", msg.hours, msg.minutes, msg.seconds);
  //time = std::to_string(msg.hours) + ":" + std::to_string(msg.minutes) + ":" + std::to_string(msg.seconds);
  QMetaObject::invokeMethod(ui_.label_nodeStatus, "setText", Q_ARG(QString, time));
}

void GpsRtkPlugin::piksiAgeOfCorrectionsCb(const piksi_rtk_msgs::AgeOfCorrections &msg) {
  double age_of_corrections = msg.age / 10.0;
  QString text;
  text.sprintf("%.1f", age_of_corrections);
  QMetaObject::invokeMethod(ui_.label_ageOfCorrections, "setText", Q_ARG(QString, text));
}
/*bool hasConfiguration() const
 {
 return true;
 }

 void triggerConfiguration()
 {
 // Usually used to open a dialog to offer the user a set of configuration
 }*/

PLUGINLIB_DECLARE_CLASS(rqt_gps_rtk_plugin, GpsRtkPlugin, GpsRtkPlugin, rqt_gui_cpp::Plugin)
