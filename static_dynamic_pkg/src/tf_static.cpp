 #include <ros/ros.h>
 #include <tf2_ros/static_transform_broadcaster.h>
 #include <geometry_msgs/TransformStamped.h>

int main(int argc, char **argv)
{
  ros::init(argc,argv, "my_static_tf2_broadcaster");

  static tf2_ros::StaticTransformBroadcaster static_broadcaster;
  geometry_msgs::TransformStamped static_transformStamped;

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