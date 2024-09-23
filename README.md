# Transformation

## [C. Transformation Basics](A.%20Transformation%20Basics/Transformations_and_Frames.md)

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

## [D -  Static and Dynamoc Transformation in ROS](B.%20Static%20and%20Dynamic%20%20transformations/Static.md)
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
[Static TF](Transformation_Basics_and_Stactic_Transforms/Static.md)
[Dynamic TF](Transformation_Basics_and_Stactic_Transforms/Static.md)

## [E - Robot and Joint State publisher](C.%20robot%20and%20joint%20state%20publisher/robot_joint_state_publisher.md)

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

