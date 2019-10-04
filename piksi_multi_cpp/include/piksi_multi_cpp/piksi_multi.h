#ifndef PIKSI_MULTI_CPP_PIKSI_MULTI_H_
#define PIKSI_MULTI_CPP_PIKSI_MULTI_H_

namespace piksi_multi_cpp {

class PiksiMulti {
public:
  PiksiMulti(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);

private:
  void getROSParameters();
  void advertiseTopics();

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  struct Parameters {
    Parameters() {}
  };

  Parameters params_;
};

} // namespace piksi_multi_cpp

#endif // PIKSI_MULTI_CPP_PIKSI_MULTI_H_
