#include "youbot_camera/RT_youbot_base.h"

namespace youbot_camera_suite
{
  void RT_youbot_base::write()
  {
    robot->getBasePosition(x,y,omega_z);
    odom_msg.header.seq += 1;
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame = "base_footprint";
    odom_msg.pose.pose.position.x = static_cast<double>(x);
    odom_msg.pose.pose.position.y = static_cast<double>(y);
    odom_msg.pose.pose.position.z = 0;
    odom_msg.pose.pose.orientation.x = 0;
    odom_msg.pose.pose.orientation.y = 0;
    odom_msg.pose.pose.orientation.z = static_cast<double>(phi_z);
    odom_msg.pose.pose.orientation.w = 1.0;
    for (int i = 0; i < 6; i++) {
      for (int j = 0; j < 6; j++) {
        pose_cov.covariance[i*6+j] = (i == j) ? 0.1 : 0.0;
      }
    }
    odom_msg.twist.twist.linear.x = static_cast<double>(velocity_x);
    odom_msg.twist.twist.linear.y = static_cast<double>(velocity_y);
    odom_msg.twist.twist.linear.z = 0;
    odom_msg.twist.twist.angular.x = 0;
    odom_msg.twist.twist.angular.y = 0;
    odom_msg.twist.twist.angular.z = static_cast<double>(omega_z);
    odom_pub.publish(odom_msg);
  }

  void RT_youbot_base::read(const geometry_msgs::Twist::ConstPtr& state);
  {
    const quantity<si::velocity>& Lvelocity = state->linear.x * meter_per_second;
    const quantity<si::velocity>& Tvelocity = state->linear.x * meter_per_second;
    const quantity<si::angular_velocity>& angulVelocity = state->angular.z * radian_per_second;
    
    robot->setBaseVelocity(Lvelocity, Tvelocity, angulVelocity);
  }
}
