#ifndef TELEOP_TWIST_JOY_TELEOP_TWIST_JOY_H
#define TELEOP_TWIST_JOY_TELEOP_TWIST_JOY_H
#include <std_msgs/String.h>
namespace ros { class NodeHandle; }
namespace teleop_twist_joy
{
class TeleopTwistJoy
{
public:
    TeleopTwistJoy();
  TeleopTwistJoy(ros::NodeHandle* nh, ros::NodeHandle* nh_param);

private:
  struct Impl;
  Impl* pimpl_;
  std::string robot_name_;
};

}

#endif 
