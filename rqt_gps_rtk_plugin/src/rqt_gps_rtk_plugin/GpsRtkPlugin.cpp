#include "../include/rqt_gps_rtk_plugin/GpsRtkPlugin.hpp"

#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <QGridLayout>
#include <QVector>

#include <math.h>

GpsRtkPlugin::GpsRtkPlugin()
    : rqt_gui_cpp::Plugin(),
      widget_(0),
      timeFirstSampleMovingWindow_(0),
      wifiCorrectionsAvgHz_(0),
      numCorrectionsFirstSampleMovingWindow_(0),
      signalStrengthUpdateRate_(0.0),
      lastSignalStrengthStamp_(0.0),
      initialSignalStrengthStamp_(0.0),
      xRange_(30) {

  // give QObjects reasonable names
  setObjectName("GpsRtkPlugin");

  // set pens;
  pens_.push_back(QPen(Qt::blue));
  pens_.push_back(QPen(Qt::green));
  pens_.push_back(QPen(Qt::red));
  pens_.push_back(QPen(Qt::cyan));
  pens_.push_back(QPen(Qt::gray));
  pens_.push_back(QPen(Qt::darkBlue));
  pens_.push_back(QPen(Qt::darkGreen));
  pens_.push_back(QPen(Qt::darkRed));
  pens_.push_back(QPen(Qt::darkCyan));
  pens_.push_back(QPen(Qt::darkYellow));
  pens_.push_back(QPen(Qt::darkGray));
}

void GpsRtkPlugin::initPlugin(qt_gui_cpp::PluginContext& context) {
  // access standalone command line arguments
  QStringList argv = context.argv();
  // create QWidget
  widget_ = new QWidget();
  // extend the widget with all attributes and children from UI file
  ui_.setupUi(widget_);
  if (context.serialNumber() > 1) {
    widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
  }
  // add widget to the user interface
  context.addWidget(widget_);

  readParameters();
  initLabels();
  initSubscribers();

  // setup custom plot
  setupPlot();

  //Init variables
  timeFirstSampleMovingWindow_ = ros::Time::now().sec;
  wifiCorrectionsAvgHz_ = 5;
  numCorrectionsFirstSampleMovingWindow_ = 0;
  altitudes_.erase(altitudes_.begin(), altitudes_.end());
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

  getNodeHandle().param<double>("signalStrengthUpdateRate", signalStrengthUpdateRate_, 1.0);
  // ensure frequency is != 0
  if (signalStrengthUpdateRate_ == 0.0) {
    signalStrengthUpdateRate_ = 1.0;
    ROS_WARN_STREAM("[GpsRtkPlugin] Parameter signalStrengthUpdateRate [Hz] must not equal 0, set to 1 Hz.");
  }
  ROS_INFO_STREAM("[GpsRtkPlugin] signalStrengthUpdateRate: " << signalStrengthUpdateRate_);
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

void GpsRtkPlugin::piksiReceiverStateCb(const piksi_rtk_msgs::ReceiverState_V2_2_15& msg) {
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
  } else {
    // Unknown
    color_fix_mode = "QLabel {background-color: rgb(152, 152, 152); color: rgb(92, 92, 92);}";
  }

  QMetaObject::invokeMethod(ui_.label_fixType, "setText", Q_ARG(QString, fix_mode));
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

  /* update signal strength */
  // throttled update of custom plot at
  double signalStrengthStamp = msg.header.stamp.toSec();
  if (initialSignalStrengthStamp_ == 0.0) {
    initialSignalStrengthStamp_ = signalStrengthStamp;
  }
  double stamp = signalStrengthStamp - initialSignalStrengthStamp_;
  if ( (lastSignalStrengthStamp_ == 0.0) or (stamp >= (lastSignalStrengthStamp_ + 1.0 / signalStrengthUpdateRate_)) ) {
    // number of available signals
    int numSignals = msg.cn0_gps.size();
    // amount of graphs (in the plot)
    int numGraphs = ui_.customPlot->graphCount();
    // amount of curves
    int numCurves = curves_.size();

    // check if enough graphs/curves are available
    if (numCurves != numGraphs) {
      // something is very strange!
      ROS_ERROR("[GpsRtkPlugin] Bad handling of curves/graphs!");
      return;
    }

    // append graphs/curves if more signals exist
    if (numGraphs < numSignals) {
      for (int i = numGraphs; i < numSignals; ++i) {
        // add new curve
        rqt_gps_rtk_plugin::CurveData curve;
        curve.setMaxSignalValues(xRange_);
        curves_.push_back(curve);
        // add new graph to plot
        postToThread([this]{ ui_.customPlot->addGraph(); }, this);
        // set line style
        postToThread([this, i]{ ui_.customPlot->graph(i)->setLineStyle(QCPGraph::LineStyle::lsLine); }, this);
        // set color
        QPen pen = (i > pens_.size()) ? (pens_.at(i-pens_.size())) : (pens_.at(i));
        postToThread([this, i, pen]{ ui_.customPlot->graph(i)->setPen(pen); }, this);
      }

      // remove graphs/curves if too many exist
    } else if (numGraphs > numSignals) {
      for (int i = numGraphs; i > numSignals; --i) {
        // remove curve
        curves_.pop_back();
        // remove graph from plot
        postToThread([this, i]{ ui_.customPlot->removeGraph(i); }, this);
      }
    }

    // update signal values
    QVector<double> x, y;
    for (int i = 0; i < numGraphs; ++i) {
      // append data to curve
      std::pair<double, double> signalPair(msg.cn0_gps.at(i) / kSignalStrengthScalingFactor, stamp);
      curves_.at(i).appendSignalValue(signalPair);
      // add data to plot
      QVector<std::pair<double, double>> signalVec = curves_.at(i).getSignalValues();
      x.clear();
      y.clear();
      for (int l = 0; l < signalVec.size(); ++l) {
        x.push_back(signalVec.at(l).second);
        y.push_back(signalVec.at(l).first);
      }
      postToThread([this, i, x, y]{ ui_.customPlot->graph(i)->setData(x, y); }, this);
    }

    // update plot
    int minX, maxX;
    maxX = int(stamp);
    minX = (maxX - xRange_ < 0) ? (0) : (maxX - xRange_);
    postToThread([this, minX, maxX]{ ui_.customPlot->xAxis->setRange(minX, maxX); }, this);

    emit(replotRequested());
    // update stamp
    lastSignalStrengthStamp_ = stamp;
  }

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
  for (std::vector<double>::iterator it = altitudes_.begin(); it != altitudes_.end(); it++) {
    altitudeAvg += *it;
  }
  altitudeAvg /= altitudes_.size();
  QString text;
  text.sprintf("%.2f", altitudeAvg);
  QMetaObject::invokeMethod(ui_.label_navsatFixAlt, "setText", Q_ARG(QString, text));
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

void GpsRtkPlugin::setupPlot() {
  connect(this, SIGNAL(replotRequested()), ui_.customPlot, SLOT(replot()));
  ui_.customPlot->clearGraphs();
  // set axis labels
  ui_.customPlot->xAxis->setLabel("seconds");
  ui_.customPlot->yAxis->setLabel("dB-Hz");
  // set axes ranges
  ui_.customPlot->xAxis->setRange(0, 1);
  ui_.customPlot->yAxis->setRange(15, 80);
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
