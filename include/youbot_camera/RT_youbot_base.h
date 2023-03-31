#include <ros/ros.h>
#include "youbot_driver/youbot/YouBotBase.hpp"
#include "geometry_msgs/Twist.h"
#include <nav_msgs/Odometry.h>

namespace youbot_camera_suite
{
  class RT_youbot_base
  {
  public:
    RT_youbot_base(const std::string name, const std::string configFilePath = "../config/")
    {
      robot = new YouBotBase(name, configFilePath)
      robot->doJointCommutation();
      cmd_sub = nh.subscribe("cmd_vel", 1000, read);
      odom_pub = nh.advertise<nav_msgs::Odometry>("youbot_odom", 1000);
      header_seq = 0;
    }

    ~RT_youbot_base() 
    {
      delete robot;
    }

    void read(const geometry_msgs::Twist::ConstPtr& state);

    void write();

  private:
  std::string robot_name;
  std::string robot_configFilePath;

  ros::NodeHandle nh;

  ros::Subscriber cmd_sub;
  ros::Publisher  odom_pub;
  nav_msgs::Odometry robot_odom;
  nav_msgs::Odometry rgbd_odom

  youbot::YouBotBase* robot;
  int header_seq;
  quantity<si::length> x;
  quantity<si::length> y;
  quantity<plane_angle> phi_z;
  quantity<si::velocity>& velocity_x;
  quantity<si::velocity>& velocity_y;
  quantity<si::angular_velocity>& omega_z;
  };
} // namespace youbot_cam_suite
