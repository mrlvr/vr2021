#include "ros/ros.h"
#include "teleop_twist_joy/teleop_twist_joy.h"
#include <stdlib.h>

int main(int argc, char *argv[])
{
  
  ROS_WARN("***Teleop Twist Joy Started");
  std::cout<<argc;  
  std::cout<<"\n";

  ros::init(argc, argv, "teleop_twist_joy_node");

  for (int i = 0; i < argc; ++i) {
	  std::cout<<argv[i];        
	  std::cout<<"\n";
  }

  ros::NodeHandle nh(""), nh_param("~");
  teleop_twist_joy::TeleopTwistJoy joy_teleop(&nh, &nh_param);

  ros::spin();
}
