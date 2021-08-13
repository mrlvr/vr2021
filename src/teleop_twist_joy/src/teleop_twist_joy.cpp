//SavExpIntel Joystick Controll

#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Int32.h"
#include "teleop_twist_joy/teleop_twist_joy.h"

#include <map>
#include <string>


namespace teleop_twist_joy
{

/**
 * Internal members of class. This is the pimpl idiom, and allows more flexibility in adding
 * parameters later without breaking ABI compatibility, for robots which link TeleopTwistJoy
 * directly into base nodes.
 */

struct TeleopTwistJoy::Impl
{
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    void joyControlCallback(const std_msgs::String::ConstPtr& cmd);
    void publishCommandVelocityBasedOnRobot(geometry_msgs::Twist cmd);
    void publishVictimCommand(int type);

    ros::Subscriber joy_sub;
    ros::Publisher cmd_vel_pub_robot;
    ros::Publisher syscommand_pub_robot;
    ros::Subscriber joy_control;

    ros::NodeHandle *node_handle;

    bool isRobotSelected;
    std::string robot_name;

    int enable_button;
    int high_speed_button;
    int alive_victim_button;
    int dead_victim_button;
    int ultra_speed_button ;

    std::map<std::string, int> axis_linear_map;
    std::map<std::string, double> scale_linear_map;
    std::map<std::string, double> scale_linear_turbo_map;
    std::map<std::string,double> scale_linear_us_map;

    std::map<std::string, int> axis_angular_map;
    std::map<std::string, double> scale_angular_map;
    std::map<std::string, double> scale_angular_turbo_map;
    std::map<std::string, double> scale_angular_us_map ;


    bool sent_disable_msg;
    bool sent_us_msg;

};

TeleopTwistJoy::TeleopTwistJoy(ros::NodeHandle* nh, ros::NodeHandle* nh_param)
{

    nh_param->param<std::string>("robot_name", robot_name_, "robot0");

    pimpl_ = new Impl;
    pimpl_->isRobotSelected = false;

    pimpl_->cmd_vel_pub_robot = nh->advertise<geometry_msgs::Twist>(robot_name_+"/cmd_vel", 1, true);


    pimpl_->joy_sub = nh->subscribe<sensor_msgs::Joy>("joy", 1, &TeleopTwistJoy::Impl::joyCallback, pimpl_);

    pimpl_->joy_control = nh->subscribe<std_msgs::String>(robot_name_+"/joy_control", 1, &TeleopTwistJoy::Impl::joyControlCallback, pimpl_);
    pimpl_->syscommand_pub_robot = nh->advertise<std_msgs::Int32>((robot_name_+"/victim_marker"), 1000);


    nh_param->param<int>("enable_button", pimpl_->enable_button, 5);
    nh_param->param<int>("high_speed_button", pimpl_->high_speed_button, 9);
    nh_param->param<int>("alive_victim_button",pimpl_->alive_victim_button,4);
    nh_param->param<int>("dead_victim_button",pimpl_->dead_victim_button,3);



    if (nh_param->getParam("axis_linear", pimpl_->axis_linear_map))
    {
        nh_param->getParam("axis_linear", pimpl_->axis_linear_map);
        nh_param->getParam("scale_linear", pimpl_->scale_linear_map);
        nh_param->getParam("scale_linear_turbo", pimpl_->scale_linear_turbo_map);
        nh_param->getParam("scale_linear_us", pimpl_ ->scale_linear_us_map) ;
    }

    else

    {
        nh_param->param<int>("axis_linear", pimpl_->axis_linear_map["x"], 1);
        nh_param->param<double>("scale_linear", pimpl_->scale_linear_map["x"], 0.6);
        nh_param->param<double>("scale_linear_turbo", pimpl_->scale_linear_turbo_map["x"], 2.0);
        nh_param->param<double>("scale_linear_us" , pimpl_ ->scale_angular_us_map["x"], 4);
    }

    if (nh_param->getParam("axis_angular", pimpl_->axis_angular_map))
    {
        nh_param->getParam("axis_angular", pimpl_->axis_angular_map);
        nh_param->getParam("scale_angular", pimpl_->scale_angular_map);
        nh_param->getParam("scale_angular_turbo", pimpl_->scale_angular_turbo_map);
    }
    else
    {
        nh_param->param<int>("axis_angular", pimpl_->axis_angular_map["yaw"], 0);
        nh_param->param<double>("scale_angular", pimpl_->scale_angular_map["yaw"], 2.0); //0.5

        nh_param->param<double>("scale_angular_turbo",
                                pimpl_->scale_angular_turbo_map["yaw"], pimpl_->scale_angular_map["yaw"]);

        nh_param->param<double>("scale_angular_us",
                                pimpl_->scale_angular_us_map["yaw"], pimpl_->scale_angular_map["yaw"]);
    }

    ROS_INFO_NAMED("TeleopTwistJoy", "Teleop enable button %i.", pimpl_->enable_button);
    ROS_INFO_COND_NAMED(pimpl_->high_speed_button >= 1, "TeleopTwistJoy",
                        "Turbo on button %i.", pimpl_->high_speed_button);

    for (std::map<std::string, int>::iterator it = pimpl_->axis_linear_map.begin();
         it != pimpl_->axis_linear_map.end(); ++it)
    {
        ROS_INFO_NAMED("TeleopTwistJoy", "Linear axis %s on %i at scale %f.",
                       it->first.c_str(), it->second, pimpl_->scale_linear_map[it->first]);
        ROS_INFO_COND_NAMED(pimpl_->high_speed_button >= 1, "TeleopTwistJoy",
                            "High Speed for linear axis %s is scale %f.", it->first.c_str(), pimpl_->scale_linear_turbo_map[it->first]);
    }

    for (std::map<std::string, int>::iterator it = pimpl_->axis_angular_map.begin();
         it != pimpl_->axis_angular_map.end(); ++it)
    {
        ROS_INFO_NAMED("TeleopTwistJoy", "Angular axis %s on %i at scale %f.",
                       it->first.c_str(), it->second, pimpl_->scale_angular_map[it->first]);
        ROS_INFO_COND_NAMED(pimpl_->high_speed_button >= 1, "TeleopTwistJoy",
                            "Turbo for angular axis %s is scale %f.", it->first.c_str(), pimpl_->scale_angular_turbo_map[it->first]);
    }

    pimpl_->sent_disable_msg = false;
    pimpl_->robot_name = robot_name_;
    pimpl_->node_handle = new ros::NodeHandle("~");

}
void TeleopTwistJoy::Impl::joyControlCallback(const std_msgs::String::ConstPtr& cmd)
{
    if(cmd->data=="start")
        isRobotSelected = true;
    else if (cmd->data=="stop")
        isRobotSelected = false;
}

void TeleopTwistJoy::Impl::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
    if(isRobotSelected)
    {
        // Initializes with zeros by default.

        geometry_msgs::Twist cmd_vel_msg;

        if (high_speed_button >= 0 && joy_msg->buttons[high_speed_button])
        {
            ROS_WARN("High Speed Mode");

            if (axis_linear_map.find("x") != axis_linear_map.end())
            {
                cmd_vel_msg.linear.x = joy_msg->axes[axis_linear_map["x"]] * scale_linear_turbo_map["x"];
            }


            if (axis_linear_map.find("y") != axis_linear_map.end())
            {
                cmd_vel_msg.linear.y = joy_msg->axes[axis_linear_map["y"]] * scale_linear_turbo_map["y"];
            }


            if  (axis_linear_map.find("z") != axis_linear_map.end())
            {
                cmd_vel_msg.linear.z = joy_msg->axes[axis_linear_map["z"]] * scale_linear_turbo_map["z"];
            }


            if  (axis_angular_map.find("yaw") != axis_angular_map.end())
            {
                cmd_vel_msg.angular.z = joy_msg->axes[axis_angular_map["yaw"]] * scale_angular_turbo_map["yaw"];
            }


            if  (axis_angular_map.find("pitch") != axis_angular_map.end())
            {
                cmd_vel_msg.angular.y = joy_msg->axes[axis_angular_map["pitch"]] * scale_angular_turbo_map["pitch"];
            }
            //publishVictimCommand

            if  (axis_angular_map.find("roll") != axis_angular_map.end())
            {
                cmd_vel_msg.angular.x = joy_msg->axes[axis_angular_map["roll"]] * scale_angular_turbo_map["roll"];
            }

            cmd_vel_pub_robot.publish(cmd_vel_msg);
            sent_disable_msg = false;

        }

        //Enab Button Part
        else if (joy_msg->buttons[enable_button])
        {
            ROS_INFO ("Normal Controll Mode") ;

            if  (axis_linear_map.find("x") != axis_linear_map.end())
            {
                cmd_vel_msg.linear.x = joy_msg->axes[axis_linear_map["x"]] * scale_linear_map["x"] * 0.96;
            }


            if  (axis_linear_map.find("y") != axis_linear_map.end())
            {
                cmd_vel_msg.linear.y = joy_msg->axes[axis_linear_map["y"]] * scale_linear_map["y"] * 0.3;
            }


            if  (axis_linear_map.find("z") != axis_linear_map.end())
            {
                cmd_vel_msg.linear.z = joy_msg->axes[axis_linear_map["z"]] * scale_linear_map["z"] * 0.3;
            }


            if  (axis_angular_map.find("yaw") != axis_angular_map.end())
            {
                cmd_vel_msg.angular.z = joy_msg->axes[axis_angular_map["yaw"]] * scale_angular_map["yaw"] * 0.3;
            }


            if  (axis_angular_map.find("pitch") != axis_angular_map.end())
            {
                cmd_vel_msg.angular.y = joy_msg->axes[axis_angular_map["pitch"]] * scale_angular_map["pitch"] * 0.3;
            }


            if  (axis_angular_map.find("roll") != axis_angular_map.end())
            {
                cmd_vel_msg.angular.x = joy_msg->axes[axis_angular_map["roll"]] * scale_angular_map["roll"] * 0.3; //0.3
            }

            publishCommandVelocityBasedOnRobot(cmd_vel_msg);


            //        cmd_vel_pub_robot_1.publish(cmd_vel_msg);
            sent_disable_msg = false;
            //ROS_INFO("disable message failure");
        }

        // when E.B Realeased Robot Stop Moving by send a single STOP command for robot

        else
        {
            if (!sent_disable_msg)
            {
                publishCommandVelocityBasedOnRobot(cmd_vel_msg);
                sent_disable_msg = true;
                ROS_WARN("Enable Button Released");
            }
        }
        if (joy_msg->buttons[alive_victim_button])
        {
           publishVictimCommand(1);
        }

        else if (joy_msg->buttons[dead_victim_button])
        {
           publishVictimCommand(2);
        }
    }

}
void TeleopTwistJoy::Impl::publishCommandVelocityBasedOnRobot(geometry_msgs::Twist cmd)
{
    cmd_vel_pub_robot.publish(cmd);
    
}

void TeleopTwistJoy::Impl::publishVictimCommand(int type)
{
    try
    {
//        ros::NodeHandle nh("~");
        ROS_DEBUG("MRL: Try to mark a victim");

        std_msgs::Int32 message;
        message.data = type;
        syscommand_pub_robot.publish(message);
//        syscommand_pub_robot.shutdown();
        ROS_DEBUG("MRL: Victim is marked");

    }
    catch(...)
    {
        ROS_DEBUG("MRL: Victim markring is faild");

    }
}




}
