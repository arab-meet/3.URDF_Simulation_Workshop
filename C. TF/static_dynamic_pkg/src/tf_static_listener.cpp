#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf2_listener");

  ros::NodeHandle node;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Rate rate(10.0);
  while (node.ok()){
    geometry_msgs::TransformStamped transformStamped;
    try{
      transformStamped = tfBuffer.lookupTransform("turtle2", "turtle1",
                               ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    rate.sleep();
  }
  return 0;
};