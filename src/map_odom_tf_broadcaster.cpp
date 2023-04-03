#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
 
int main(int argc, char** argv)
{
  ros::init(argc, argv, "map_odom_broadcaster");
  ros::NodeHandle node;

  tf2_ros::TransformBroadcaster tfb;
  geometry_msgs::TransformStamped transformStamped;


  transformStamped.header.frame_id = "map";
  transformStamped.child_frame_id = "odom";
  transformStamped.transform.translation.x = 0.5;
  transformStamped.transform.translation.y = 0.5;
  transformStamped.transform.translation.z = 0.0;
  // tf2::Quaternion q;
  // q.setRPY(0, 0, 0);
  // transformStamped.transform.rotation.x = q.x();
  // transformStamped.transform.rotation.y = q.y();
  // transformStamped.transform.rotation.z = q.z();
  // transformStamped.transform.rotation.w = q.w();

  transformStamped.transform.rotation.x = 0;
  transformStamped.transform.rotation.y = 0;
  transformStamped.transform.rotation.z = 0;
  transformStamped.transform.rotation.w = 1;


  ros::Rate rate(10.0);
  while (ros::ok())
  {
    transformStamped.header.stamp = ros::Time::now();
    tfb.sendTransform(transformStamped);
    rate.sleep();
  }
};
