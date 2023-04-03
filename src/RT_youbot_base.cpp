#include "youbot_camera/RT_youbot_base.h"

namespace youbot_camera_suite
{
  void RT_youbot_base::write()
  {
    nav_msgs::Odometry odom_msg;
    robot->getBasePosition(pv.x,pv.y,pv.phi_z);
    odom_msg.header.seq += 1;
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_footprint";
    odom_msg.pose.pose.position.x = pv.x.value();
    odom_msg.pose.pose.position.y = pv.y.value();
    odom_msg.pose.pose.position.z = 0;
    odom_msg.pose.pose.orientation.x = 0;
    odom_msg.pose.pose.orientation.y = 0;
    odom_msg.pose.pose.orientation.z = pv.phi_z.value();
    odom_msg.pose.pose.orientation.w = 1.0;
    for (int i = 0; i < 6; i++) {
      for (int j = 0; j < 6; j++) {
        odom_msg.pose.covariance[i*6+j] = (i == j) ? 0.1 : 0.0;
      }
    }
    robot->getBaseVelocity(pv.velocity_x, pv.velocity_y, pv.omega_z);
    odom_msg.twist.twist.linear.x = pv.velocity_x.value();
    odom_msg.twist.twist.linear.y = pv.velocity_y.value();
    odom_msg.twist.twist.linear.z = 0;
    odom_msg.twist.twist.angular.x = 0;
    odom_msg.twist.twist.angular.y = 0;
    odom_msg.twist.twist.angular.z = pv.omega_z.value();
    for (int i = 0; i < 6; i++) {
      for (int j = 0; j < 6; j++) {
        odom_msg.twist.covariance[i*6+j] = (i == j) ? 0.1 : 0.0;
      }
    }
    odom_pub.publish(odom_msg);
  }

  void RT_youbot_base::subscriberCallBack(const geometry_msgs::Twist::ConstPtr& state)
  {

    const quantity<si::velocity>& Lvelocity = state->linear.x * meter_per_second;
    const quantity<si::velocity>& Tvelocity = state->linear.y * meter_per_second;
    const quantity<si::angular_velocity>& angulVelocity = state->angular.z * radian_per_second;
    
    robot->setBaseVelocity(Lvelocity, Tvelocity, angulVelocity);

    rv.set(Lvelocity, Tvelocity, angulVelocity);
  }

  void RT_youbot_base::read()
  {
    ROS_INFO("cmd velocity is: LV = %d, TV = %d, AV = %d", rv.Lvelocity, rv.Tvelocity, rv.angulVelocity );
  }

  void RT_youbot_base::update()
  {
    this->read();
    this->write();
  }
}
