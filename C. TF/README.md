## [A. Transformation Basics](<A. Transformation Basics/Transformations_and_Frames.md>)

- **Purpose**: Coordinate transformations are essential in Autonomous Mobile Robots (AMR) to relate different sensor data and robot positions across various reference frames.
- **Key Frames**: Include Map, World, Odometry (global); Base Link, Laser, Camera (robot-specific).
- **ROS TF**: Provides tools for managing and applying transformations between frames, crucial for integrating sensor data and robot control.

## B - Static and Dynamic Transformation in ROS

- **[Static Transformations](<B. Static and Dynamic  transformations/Static.md>)**: Fixed relationships (e.g., sensor to base), published on `/tf_static`, use `StaticTransformBroadcaster`.
- **[Dynamic Transformations](<B. Static and Dynamic  transformations/Dynamic.md>)**: Changing relationships (e.g., robot to starting point), published on `/tf`, use `TransformBroadcaster`.
- **Key Points**: Both use TF broadcasters, include translation and rotation, listeners retrieve data, TF tree represents frame hierarchy.

## [C - Robot and Joint State publisher](<C. robot and joint state publisher/robot_joint_state_publisher.md>)

- **Robot State Publisher**: Broadcasts robot state to tf library, uses URDF to compute and publish 3D poses of links.
- **Joint State Publisher**: Publishes JointState messages for non-fixed joints, sources include GUI, subscribed messages, or default values.
- **Key Points**: Automates TF publishing from URDF, handles non-fixed joints, useful for testing and visualization, real-world applications use controller_manager.
