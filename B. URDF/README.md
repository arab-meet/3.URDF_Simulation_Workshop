
# B. URDF

## [A. An Introduction to URDF, Links, and Xacro](<A. An Introduction to URDF, Links, and Xacro/An Introduction to URDF, Links, and Xacro.md>)


The Unified Robot Description Format (URDF) is indeed a pivotal element in robotics, especially within the ROS ecosystem. It serves as the backbone for defining and modeling robots, ensuring that the physical and functional attributes are accurately represented. The integration of Xacro enhances URDF by introducing adaptability and reusability, which are essential for managing large-scale and complex robot designs. This combination of URDF and Xacro underlines the importance of standardization and efficiency in the ever-evolving field of robotics.

## [B. Joints](<B.  joints/joints.md>)

Joints serve to connect two links in a robot and describes the kinematics and dynamics of them. The primary link is designated as the `<parent>`, and the second link becomes the `<child>`. There are four types of joints:  **Fixed** ,  **Revolute** ,  **Continuous** , and  **Prismatic** . Each type defines how the `<parent>` link is related to the `<child>` link.

## [C. Plugins](<C. Plugins/plugins.md>)


**plugins** are software components or modules that extend the functionality of a system by adding new features or capabilities without modifying the core software. Plugins are typically designed to be easily integrated and interchangeable, allowing developers to customize or enhance specific functionalities.**Gazebo plugins** are used to add custom behaviors, sensors, and controllers to robots in simulation. They allow users to simulate real-world scenarios, sensor data, and actuation within the Gazebo environment.

## [D. Exporting SolidWorks Design as URDF for ROS ](<D. sw2urdf/export_solid_as_urdf.md>)


We will discuss how to create a URDF from SolidWorks, including how to create coordinate systems for URDF export and set axis alignment for the wheels. After that, we’ll export the design to URDF and explore the structure or output package from SolidWorks. Lastly, we'll cover how to add the plugin and launch it in your environment.


## E. Tutorials

* [skid_steer_robot](<E. example/skid_steer_robot_example.md>)
* [differential_drive_robot](<E. example/differential_drive_robot_example.md>)


## [Next Topic →](<../C. TF/A. Transformation Basics/Transformations_and_Frames.md>)

## [↩Back to main](../README.md)
