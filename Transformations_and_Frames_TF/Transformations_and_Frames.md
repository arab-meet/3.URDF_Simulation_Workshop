# Transformation_And_Frames(TF)

## What is tf?

tf (short for transform) is a powerful library in ROS (Robot Operating System) that deals with managing coordinate frames and transformations. It allows you to express the relationships between different coordinate systems in a robotic system. Here are some key points about tf:

**1.Coordinate Frames** :
  * A frame is a coordinate system. In ROS, frames are always 3D, right-handed, with the X-axis pointing forward, Y-axis to the left, and Z-axis upward.
  * Points within a frame are represented using `tf::Point`, which is equivalent to the bullet type `btVector3`.
  * The coordinates of a point p in a frame W are written as Wp [**1**](http://wiki.ros.org/tf/Overview/Transformations).

![frames and transforms](<images/frames and transforms.png>)

**2.Functionality:**
  * tf provides tools for managing transformations between frames, broadcasting coordinate frames, and listening to frame updates.
  * It’s essential for tasks like robot localization, sensor fusion, and motion planning.

# Robot State Publisher
The Robot State Publisher in ROS 1 is a crucial component that plays a significant role in robot modeling and visualization.



### Purpose and Functionality:
* The robot_state_publisher allows you to broadcast the state of a robot to the tf transform library.
* It internally maintains a kinematic model of the robot, which includes information about its links, joints, and transformations.
* Given the joint positions of the robot (obtained from the joint_states topic), the robot_state_publisher computes and broadcasts the 3D pose of each link in the robot.


![robot state publisher](<images/robot state publisher node.png>)


### How It Works:
* At startup, the robot_state_publisher is provided with a URDF (Unified Robot Description Format) model of the robot. This URDF describes the robot’s structure, including its links, joints, and transformations.
* It subscribes to the joint_states topic (of type sensor_msgs/JointState) to receive information about the robot’s joint positions.
* Using the joint positions, it calculates the forward kinematics of the robot, determining the 3D poses of each link.
* These calculated poses are then published via the tf system, making them available to other components in the ROS system.
### Key Features and Usage:
* The robot_state_publisher can be used both as a library (for custom applications) and as a ROS node.
* It’s commonly employed for visualization in tools like RViz, where it helps display the robot’s pose accurately.
* The package is well-tested and considered stable, with no major changes planned in the near future.
### Parameters and Topics:
* `robot_description`: Specifies the URDF XML robot description.
* `tf_prefix`: Sets the tf prefix for namespace-aware publishing of transforms.
* `publish_frequency`: Controls the frequency of state publishing (default: 50Hz).
* `ignore_timestamp`: If true, ignores the publish frequency and timestamp of joint_states, publishing a tf for each received joint_states (default: “false”).
* `use_tf_static`: Determines whether to use tf_static (used for static transforms).

*Note:All fixed transforms are future-dated by 0.5 seconds to ensure consistency.*

# Joint_State_Publisher

The joint_state_publisher in ROS 1 is a valuable tool for managing joint states within a robot model.


### Overview:
* The joint_state_publisher is a ROS package that publishes sensor_msgs/JointState messages for a robot described using the URDF (Unified Robot Description Format).
* Its primary responsibility is to continually publish values for all movable joints in the URDF to the /joint_states topic.

![joint state publisher](<images/joint state publisher.png>)

### How It Works:
* The package reads the robot_description parameter from the parameter server.
* It identifies all non-fixed joints in the robot model.
* For each of these joints, it constructs a JointState message containing the joint’s position, velocity, and effort (if available).
* These messages are then published to the /joint_states topic.

![joint state publisher inputs](<images/joint state publisher 2.png>)
### Data Input Sources:
 The joint_state_publisher can obtain joint state values from various sources:
* GUI: The GUI functionality (now in a separate package called joint_state_publisher_gui) allows manual input of joint positions via sliders.
* Subscribed JointState Messages: It subscribes to JointState messages from other nodes.
* Default Values: If no other sources provide a value, it falls back to default values.
### Usage Scenarios:
* Visualization: When combined with the robot_state_publisher, it helps visualize the robot’s pose accurately in tools like RViz.
* Simulation and Testing: Useful for simulating robot behavior or testing controllers without actual hardware.
* Multiple Publishers: When you have multiple publishers of JointState messages, the joint_state_publisher ensures a coherent view across all joint state topics 12.

# Acnolegment 
1. http://wiki.ros.org/tf/Overview/Transformations
2. The ROS Transform System (TF): https://www.youtube.com/watch?v=QyvHhY4Y_Y8