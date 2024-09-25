#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_example");
  ros::NodeHandle nh;

  tf::TransformBroadcaster br;
  tf::Transform transform;

  ros::Rate rate(10.0);

  while (nh.ok()){
    // Map to Base Link
    transform.setOrigin(tf::Vector3(1.0, 2.0, 0.0));  // Z is zero
    transform.setRotation(tf::Quaternion(0, 0, 0, 1)); // No rotation
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));

    // Base Link to Camera
    transform.setOrigin(tf::Vector3(0.5, 0.0, 0.0));  // Z is zero
    transform.setRotation(tf::Quaternion(0, 0, 0.707, 0.707)); // 90 degrees rotation about Z-axis in quaternion
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "camera"));

    // Camera to Apriltag
    transform.setOrigin(tf::Vector3(1.0, 0.0, 0.0));  // Z is zero
    transform.setRotation(tf::Quaternion(0, 0, 0, 1)); // No rotation
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera", "apriltag"));

    rate.sleep();
  }
  return 0;
}
