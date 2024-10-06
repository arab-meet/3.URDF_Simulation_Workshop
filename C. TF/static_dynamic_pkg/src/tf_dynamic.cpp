#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
  
  void timerCallback(const ros::TimerEvent& event , tf2_ros::TransformBroadcaster& dynamic_broadcaster, geometry_msgs::TransformStamped& dynamic_transformStamped, float& last_x,float& x_increment)  {

  dynamic_transformStamped.header.stamp = ros::Time::now();
  dynamic_transformStamped.header.frame_id = "odom";
  dynamic_transformStamped.child_frame_id = "base_link";
  dynamic_transformStamped.transform.translation.x =x_increment + last_x;
  dynamic_transformStamped.transform.translation.y = 0.0;
  dynamic_transformStamped.transform.translation.z = 0.1;

  dynamic_transformStamped.transform.rotation.x = 0.0;
  dynamic_transformStamped.transform.rotation.y = 0.0;
  dynamic_transformStamped.transform.rotation.z = 0.0;
  dynamic_transformStamped.transform.rotation.w = 1.0;
  dynamic_broadcaster.sendTransform(dynamic_transformStamped);
  last_x = dynamic_transformStamped.transform.translation.x;

  ROS_INFO("TF dynamic has been published between %s and %s frames", dynamic_transformStamped.header.frame_id.c_str(),dynamic_transformStamped.child_frame_id.c_str());
  }



int main(int argc, char **argv)
{
  ros::init(argc,argv, "my_dynamic_tf2_broadcaster");

  static tf2_ros::StaticTransformBroadcaster static_broadcaster;
  tf2_ros::TransformBroadcaster dynamic_broadcaster;
  geometry_msgs::TransformStamped static_transformStamped;
  geometry_msgs::TransformStamped dynamic_transformStamped;
  float x_increment = 0.05;
  float last_x = 0.0;
  ros::NodeHandle node_handle;
 ros::Timer timer = node_handle.createTimer(ros::Duration(0.1), [&dynamic_broadcaster, &dynamic_transformStamped, &last_x,&x_increment](const ros::TimerEvent& event) {
    timerCallback(event, dynamic_broadcaster, dynamic_transformStamped, last_x,x_increment);
  });


  static_transformStamped.header.stamp = ros::Time::now();
  static_transformStamped.header.frame_id = "base_link";
  static_transformStamped.child_frame_id = "laser";
  static_transformStamped.transform.translation.x = 0.0;
  static_transformStamped.transform.translation.y = 0.0;
  static_transformStamped.transform.translation.z = 0.1;

  static_transformStamped.transform.rotation.x = 0.0;
  static_transformStamped.transform.rotation.y = 0.0;
  static_transformStamped.transform.rotation.z = 0.0;
  static_transformStamped.transform.rotation.w = 1.0;
  static_broadcaster.sendTransform(static_transformStamped);
  ROS_INFO("TF static has been published between %s and %s frames", static_transformStamped.header.frame_id.c_str(),static_transformStamped.child_frame_id.c_str());
  ros::spin();
  return 0;
};