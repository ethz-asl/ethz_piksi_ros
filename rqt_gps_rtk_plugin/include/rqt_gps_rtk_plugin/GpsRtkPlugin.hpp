#ifndef GPSRTKPLUGIN_H
#define GPSRTKPLUGIN_H

#include <rqt_gui_cpp/plugin.h>
#include <ui_gps_rtk_plugin.h>

#include <QWidget>
#include <QObject>
#include <QEvent>
#include <QCoreApplication>
#include <QPen>

#include <type_traits>
#include <algorithm>
#include <unistd.h>

#include <ros/ros.h>
#include <piksi_rtk_msgs/ReceiverState_V2_2_15.h>
#include <piksi_rtk_msgs/BaselineNed.h>
#include <piksi_rtk_msgs/InfoWifiCorrections.h>
#include <piksi_rtk_msgs/UtcTimeMulti.h>
#include <piksi_rtk_msgs/AgeOfCorrections.h>
#include <sensor_msgs/NavSatFix.h>

#include <rqt_gps_rtk_plugin/CurveData.hpp>
#include <rqt_gps_rtk_plugin/qcustomplot.h>

constexpr double kSignalStrengthScalingFactor = 4.0;

class GpsRtkPlugin : public rqt_gui_cpp::Plugin
{
  Q_OBJECT
 public:
  GpsRtkPlugin();
  virtual void initPlugin(qt_gui_cpp::PluginContext& context) override;
  virtual void shutdownPlugin() override;
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const override;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings) override;

  // Comment in to signal that the plugin has a way to configure it
  //bool hasConfiguration() const;
  //void triggerConfiguration();
 private:
  Ui::GpsRtkPlugin ui_;
  QWidget* widget_;

  void readParameters();
  void initLabels();
  void initSubscribers();

  template<typename T>
  void vectorToString(const std::vector<T> &vec, QString *pString) {
    *pString = "[";
    for (auto i = vec.begin(); i != vec.end(); ++i) {
      if (i != vec.begin()) {
        *pString += ", ";
      }
      *pString += QString::number(*i);
    }
    *pString += "]";
  }

  template<typename T>
  std::vector<T> scaleSignalStrength(const std::vector<T> &vec_signal_strength) {
    auto scaled_signal_strength = vec_signal_strength;
    for_each(scaled_signal_strength.begin(), scaled_signal_strength.end(), [](T &signal_strength) {signal_strength /= kSignalStrengthScalingFactor;});

    return scaled_signal_strength;
  }

  // method to call function in different thread
  // (workaround is to wrap function into an event and call an event instead of the function directly)
  template <typename F>
  static void postToThread(F && fun, QObject * obj = qApp) {
     struct Event : public QEvent {
        using Fun = typename std::decay<F>::type;
        Fun fun;
        Event(Fun && fun) : QEvent(QEvent::None), fun(std::move(fun)) {}
        Event(const Fun & fun) : QEvent(QEvent::None), fun(fun) {}
        ~Event() { fun(); }
     };
     QCoreApplication::postEvent(obj, new Event(std::forward<F>(fun)));
  }

  //subscribers
  ros::Subscriber piksiReceiverStateSub_;
  ros::Subscriber piksiBaselineNedSub_;
  ros::Subscriber piksiWifiCorrectionsSub_;
  ros::Subscriber piksiNavsatfixRtkFixSub_;
  ros::Subscriber piksiHeartbeatSub_;
  ros::Subscriber piksiAgeOfCorrectionsSub_;

  void piksiReceiverStateCb(const piksi_rtk_msgs::ReceiverState_V2_2_15& msg);
  void piksiBaselineNedCb(const piksi_rtk_msgs::BaselineNed& msg);
  void piksiWifiCorrectionsCb(const piksi_rtk_msgs::InfoWifiCorrections& msg);
  void piksiNavsatfixRtkFixCb(const sensor_msgs::NavSatFix& msg);
  void piksiTimeCb(const piksi_rtk_msgs::UtcTimeMulti& msg);
  void piksiAgeOfCorrectionsCb(const piksi_rtk_msgs::AgeOfCorrections& msg);

  std::string piksiReceiverStateTopic_;
  std::string piksiBaselineNedTopic_;
  std::string piksiWifiCorrectionsTopic_;
  std::string piksiNavsatfixRtkFixTopic_;
  std::string piksiTimeTopic_;
  std::string piksiAgeOfCorrectionsTopic_;

  double timeFirstSampleMovingWindow_;
  int wifiCorrectionsAvgHz_;
  int numCorrectionsFirstSampleMovingWindow_;
  std::vector<double> altitudes_;
  std::vector<rqt_gps_rtk_plugin::CurveData> curves_;
  double signalStrengthUpdateRate_; //[Hz] the update rate of the signal strength plot (in the debug tab)
  double lastSignalStrengthStamp_;
  double initialSignalStrengthStamp_;
  int xRange_;
  std::string objectName_;
  QVector<QPen> pens_;

  private slots:
    void setupPlot();

  signals:
   void replotRequested();


};

#endif // GPSRTKPLUGIN_H
