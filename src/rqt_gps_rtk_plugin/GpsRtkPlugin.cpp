#include "../include/rqt_gps_rtk_plugin/GpsRtkPlugin.hpp"

#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <QGridLayout>
#include <message_logger/message_logger.hpp>

GpsRtkPlugin::GpsRtkPlugin()
  : rqt_gui_cpp::Plugin(),
    widget_(0)
{
  // Constructor is called first before initPlugin function, needless to say.

  // give QObjects reasonable names
  setObjectName("GpsRtkPlugin");
}

void GpsRtkPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
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

  MELO_INFO("[GpsRtkPlugin] Initialized.");
}

void GpsRtkPlugin::shutdownPlugin()
{
}

void GpsRtkPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
}

void GpsRtkPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
}

void GpsRtkPlugin::readParameters() {
  getNodeHandle().param<std::string>("piksiReceiverStateTopic", piksiReceiverStateTopic_, "/piksi/debug/receiver_state");
  MELO_INFO_STREAM("[GpsRtkPlugin] piksiReceiverStateTopic: " << piksiReceiverStateTopic_);

  getNodeHandle().param<std::string>("piksiBaselineNedTopic", piksiBaselineNedTopic_, "/piksi/baseline_ned");
  MELO_INFO_STREAM("[GpsRtkPlugin] piksiBaselineNedTopic: " << piksiBaselineNedTopic_);

  getNodeHandle().param<std::string>("piksiWifiCorrectionsTopic", piksiWifiCorrectionsTopic_, "/piksi/debug/wifi_corrections");
  MELO_INFO_STREAM("[GpsRtkPlugin] piksiWifiCorrectionsTopic: " << piksiWifiCorrectionsTopic_);

  getNodeHandle().param<std::string>("piksiNavsatfixRtkFixTopic", piksiNavsatfixRtkFixTopic_, "/piksi/navsatfix_rtk_fix");
  MELO_INFO_STREAM("[GpsRtkPlugin] piksiNavsatfixRtkFixTopic: " << piksiNavsatfixRtkFixTopic_);

  getNodeHandle().param<std::string>("piksiHeartbeatTopic", piksiHeartbeatTopic_, "/piksi/heartbeat");
  MELO_INFO_STREAM("[GpsRtkPlugin] piksiHeartbeatTopic: " << piksiHeartbeatTopic_);
}

void GpsRtkPlugin::initLabels() {
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
}

void GpsRtkPlugin::initSubscribers() {
  piksiReceiverStateSub_ = getNodeHandle().subscribe(piksiReceiverStateTopic_, 10, &GpsRtkPlugin::piksiReceiverStateCb, this);
  piksiBaselineNedSub_ = getNodeHandle().subscribe(piksiBaselineNedTopic_, 10, &GpsRtkPlugin::piksiBaselineNedCb, this);
  piksiWifiCorrectionsSub_ = getNodeHandle().subscribe(piksiWifiCorrectionsTopic_, 10, &GpsRtkPlugin::piksiWifiCorrectionsCb, this);
  piksiNavsatfixRtkFixSub_ = getNodeHandle().subscribe(piksiNavsatfixRtkFixTopic_, 10, &GpsRtkPlugin::piksiNavsatfixRtkFixCb, this);
  piksiHeartbeatSub_ = getNodeHandle().subscribe(piksiHeartbeatTopic_, 10, &GpsRtkPlugin::piksiHeartbeatCb, this);
}

void GpsRtkPlugin::piksiReceiverStateCb(const piksi_rtk_msgs::ReceiverState& msg) {
  // Type of fix
  QMetaObject::invokeMethod(ui_.label_fixType, "setText", Q_ARG(QString, msg.rtk_mode_fix ? "Fix" : "Float"));
  QMetaObject::invokeMethod(ui_.label_fixType, "setStyleSheet", Q_ARG(QString, msg.rtk_mode_fix ?
        "QLabel {background-color: lime; color: rgb(211, 215, 207);}"
      : "QLabel {background-color: rgb(239, 41, 41); color: rgb(0, 0, 0);}"));
  // Number of satellites
  QMetaObject::invokeMethod(ui_.label_numSatellites, "setText", Q_ARG(QString, QString::number(msg.num_sat)));
}

void GpsRtkPlugin::piksiBaselineNedCb(const piksi_rtk_msgs::BaselineNed& msg) {
  // Number of satellites used for RTK
  QMetaObject::invokeMethod(ui_.label_numRtkSatellites, "setText", Q_ARG(QString, QString::number(msg.n_sats)));

  std::string style;
  if (msg.n_sats < 4) {
    style = "QLabel {background-color: red;}";
  } else if (msg.n_sats > 4 and msg.n_sats < 7) {
    style = "QLabel {background-color: orange;}";
  } else if (msg.n_sats > 7) {
    style = "QLabel {background-color: rgb(78, 154, 6);}";
  }
  QMetaObject::invokeMethod(ui_.label_numRtkSatellites_indicator, "setStyleSheet", Q_ARG(QString, QString::fromStdString(style)));

  // Baseline NED status
  double n = msg.n/1e3;
  double e = msg.e/1e3;
  double d = msg.d/1e3;
  std::string baseline = "[" + std::to_string(n) + ", " + std::to_string(e) + ", " + std::to_string(d) + "]";
  QMetaObject::invokeMethod(ui_.label_baseline, "setText", Q_ARG(QString, QString::fromStdString(baseline)));
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

}

void GpsRtkPlugin::piksiNavsatfixRtkFixCb(const sensor_msgs::NavSatFix& msg) {
  altitudes_.push_back(msg.altitude);
  double altitudeAvg = 0;
  for (std::vector<double>::iterator it=altitudes_.begin(); it!=altitudes_.end(); it++) {
    altitudeAvg += *it;
  }
  altitudeAvg /= altitudes_.size();
  QMetaObject::invokeMethod(ui_.label_navsatFixAlt, "setText", Q_ARG(QString, QString::number(altitudeAvg, 'g', 2)));
}

void GpsRtkPlugin::piksiHeartbeatCb(const piksi_rtk_msgs::Heartbeat& msg) {
  lastHeartbeatStamp_ = msg.header.stamp.sec;
  QMetaObject::invokeMethod(ui_.label_nodeStatus, "setText", Q_ARG(QString, QString::number(lastHeartbeatStamp_, 'g', 2)));
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
