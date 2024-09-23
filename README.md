# 3.URDF_Simulation_Workshop

### contain all data related to the Workshop topic.

## [A - Gazebo](A.%20Gazebo/Gazebo.md)

**`Gazebo`** is a versatile simulation environment widely used in robotics and autonomous systems development. It provides a realistic 3D simulation of environments, allowing engineers and researchers to test robots in virtual scenarios before deploying them in the real world. Gazebo supports complex physics, sensor simulations, and interaction with ROS (Robot Operating System), making it a valuable tool for prototyping and testing robotic applications.

## [B - URDF](B.%20URDF/README.md)

The Unified Robot Description Format (URDF), complemented by Xacro, is a cornerstone in robotics, particularly within the ROS ecosystem. It provides a standardized approach to robot modeling, ensuring accurate representation of physical and functional characteristics. Joints are integral to this model, connecting links and defining their movement and interaction. With types like Fixed, Revolute, Continuous, and Prismatic, they dictate the relationship between parent and child links. Plugins, especially Gazebo plugins, further extend these capabilities, allowing for the simulation of complex behaviors and sensor interactions in a virtual environment, which is crucial for testing and development before actual deployment.


## [C. Transformation Basics](C.%20Transformation%20Basics/Transformations_and_Frames.md)

### Coordinate frame and transformation

### Why do needed frame transformation in AMR

### Multiple Frame

**Reference Frames**

- Map
- World
- Odom

**Other Frames**

- Base Link
- Laser
- camer frame

### Rigid Body Transformation in 2D

- Rotation and Transformation Matrix
- Transform A point Example

### TF in ROS

- concept
- package nodes

### TF tools in ROS

## [D -  Static and Dynamoc Transformation in ROS](D.%20Static%20and%20Dynamic%20%20transformations/Static.md)
### Static Transformations
- Fixed relationships between robot parts (e.g., sensor to base)
- Published on `/tf_static` topic
- Use `StaticTransformBroadcaster`

### Dynamic Transformations
- Changing relationships (e.g., robot to starting point)
- Published on `/tf` topic
- Use `TransformBroadcaster`

### Key Points
- Both use TF (Transform) broadcasters
- Include translation and rotation information
- Listeners retrieve transform data between frames
- TF tree represents frame hierarchy in robot system
[Static TF](D.%20Static%20and%20Dynamic%20%20transformations/Static.md)
[Dynamic TF](D.%20Static%20and%20Dynamic%20%20transformations/Dynamic.md)

## [E - Robot and Joint State publisher](E.%20robot%20and%20joint%20state%20publisher/robot_joint_state_publisher.md)

### Robot State Publisher
- Broadcasts robot state to tf library
- Uses URDF to compute and publish 3D poses of links

### Joint State Publisher
- Publishes JointState messages for non-fixed joints
- Sources: GUI, subscribed messages, or default values

### Key Points:
1. Robot State Publisher automates TF publishing from URDF
2. Non-fixed joints require additional joint state information
3. Joint State Publisher GUI useful for testing
4. Real-world/simulation use controller_manager for joint data
5. Essential for visualization and simulation accuracy

