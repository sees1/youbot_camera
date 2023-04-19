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
      robot = new youbot::YouBotBase(name, configFilePath);
      robot->doJointCommutation();
      cmd_sub = nh.subscribe("cmd_vel", 1000, &RT_youbot_base::subscriberCallBack,this);
      odom_pub = nh.advertise<nav_msgs::Odometry>("youbot_odom", 1000);
      header_seq = 0;
    }

    ~RT_youbot_base() 
    {
      delete robot;
    }

    void subscriberCallBack(const geometry_msgs::Twist::ConstPtr& state);

    void read();

    void write();
    
    void update();

    struct received_value
    {
      received_value()
      {
        Lvelocity = 0 * meter_per_second;
        Tvelocity = 0 * meter_per_second;
        angulVelocity = 0 * radian_per_second;
      }

      void set(const quantity<si::velocity>& LV,const quantity<si::velocity>& TV,const quantity<si::angular_velocity>& AV)
      {
        Lvelocity = LV;
        Tvelocity = TV;
        angulVelocity = AV;
      }

      quantity<si::velocity> Lvelocity;
      quantity<si::velocity> Tvelocity;
      quantity<si::angular_velocity> angulVelocity;
    };

    struct published_value
    {
      published_value()
      {
        x = 0 * meter;
        y = 0 * meter;
        phi_z = 0 * radian;
        velocity_x = 0 * meter_per_second;
        velocity_y = 0 * meter_per_second;
        omega_z = 0 * radian_per_second;
      }
      quantity<si::length> x;
      quantity<si::length> y;
      quantity<plane_angle> phi_z;
      quantity<si::velocity> velocity_x;
      quantity<si::velocity> velocity_y;
      quantity<si::angular_velocity> omega_z;
    };

  private:
  std::string robot_name;
  std::string robot_configFilePath;

  ros::NodeHandle nh;

  ros::Subscriber cmd_sub;
  ros::Publisher  odom_pub;
  nav_msgs::Odometry robot_odom;

  youbot::YouBotBase* robot;
  int header_seq;

  published_value pv;
  received_value rv;

};
} // namespace youbot_cam_suite
