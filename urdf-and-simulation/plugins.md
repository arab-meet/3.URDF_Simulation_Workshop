# Integrating Sensors and Plugins into URDF for Gazebo Simulation

# Introduction

This guide explains how to incorporate sensors and Gazebo plugins into a URDF (Unified Robot Description Format) file for use with ROS (Robot Operating System). It details the steps for adding and configuring a range of sensors—such as LIDAR, depth cameras, ultrasonic sensors, and IMUs—ensuring they work effectively within Gazebo simulations.

### Adding a LIDAR Sensor

##### URDF

```xml
 <link name="lidar_link">
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
    <box size="0.1 0.1 0.1"/>
      </geometry>
    </collision>

    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://robot_description/meshes/lidar.dae"/>
      </geometry>
    </visual>

    <inertial>
      <mass value="1e-5" />
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6" />
    </inertial>
  </link>
<joint name="lidar_joint" type="fixed">
    <axis xyz="0 1 0" />
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="lidar_link"/>
  </joint>
```

> `<mesh filename="package://robot_description/meshes/hokuyo.dae"/>`: Uses a mesh file (hokuyo.dae) from the specified package to visually represent the LIDAR.

#### Gazebo Plugin of 2D Lidar

```xml
<gazebo reference="lidar_link">
        <sensor type="ray" name="head_hokuyo_sensor">
            <pose>0 0 0 0 0 0</pose>
            <visualize>true</visualize>
            <update_rate>40</update_rate>
            <ray>
                <scan>
                    <horizontal>
                        <samples>720</samples>
                        <resolution>1</resolution>
                        <min_angle>-1.570796</min_angle>
                        <max_angle>1.570796</max_angle>
                    </horizontal>
                </scan>
                <range>
                    <min>0.10</min>
                    <max>30.0</max>
                    <resolution>0.01</resolution>
                </range>
                <noise>
                    <type>gaussian</type>
                    <mean>0.0</mean>
                    <stddev>0.01</stddev>
                </noise>
            </ray>
            <plugin name="gazebo_ros_head_hokuyo_controller" filename="libgazebo_ros_laser.so">
                <topicName>/scan</topicName>
                <frameName>lidar_link</frameName>
            </plugin>
        </sensor>
    </gazebo>
```

##### Explanation

`<gazebo reference="lidar_link">`

This line links the Gazebo-specific elements to the `lidar_link` in the URDF.

`<samples>720</samples>`

 The number of samples (laser beams) per scan.

`<min_angle>-1.570796</min_angle>`:

The minimum angle of the scan (in radians), which is -90 degrees.

`<max_angle>1.570796</max_angle>`

The maximum angle of the scan (in radians), which is 90 degrees.

`<min>0.10</min>`

 The minimum range of the sensor (0.10 meters).

`<max>30.0</max>`

The maximum range of the sensor (30 meters).

`<plugin name="gazebo_ros_head_hokuyo_controller" filename="libgazebo_ros_laser.so">`

This defines a Gazebo plugin to interface the LIDAR sensor with ROS. The plugin is named "gazebo_ros_head_hokuyo_controller" and the corresponding library is "libgazebo_ros_laser.so".

`<topicName>/scan</topicName>`

This specifies the ROS topic name on which the sensor data will be published (`/scan`).

`<frameName>lidar_link</frameName>`

This specifies the reference frame for the sensor data, which is `lidar_link`. should match the link name in urdf.


#### Gazebo Plugin of 3D Lidar

```xml
<gazebo reference="lidar_link">
        <sensor type="ray" name="lidar_3d_sensor">
            <pose>0 0 0 0 0 0</pose>
            <visualize>true</visualize>
            <update_rate>40</update_rate>
            <ray>
              <scan>
        <!-- Horizontal Scan Configuration -->
        <horizontal>
          <samples>1024</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
        <!-- Vertical Scan Configuration -->
        <vertical>
          <samples>64</samples>
          <resolution>1</resolution>
          <min_angle>-0.785398</min_angle>
          <max_angle>0.785398</max_angle>
        </vertical>
      </scan>
      <range>
        <min>0.1</min>
        <max>100.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin name="gazebo_ros_laser" filename="libgazebo_ros_laser.so">
      <topicName>/lidar_3d/scan</topicName>
      <frameName>lidar_link</frameName>
    </plugin>
  </sensor>
</gazebo>
```


* **Horizontal Scan Configuration** :
* `samples`: The number of samples taken in the horizontal plane (1024 in this case).
* `resolution`: The resolution of the horizontal scan (1 degree per sample).
* `min_angle` and `max_angle`: The range of angles scanned horizontally (from -180 degrees to 180 degrees).
* **Vertical Scan Configuration** :
* `samples`: The number of samples taken in the vertical plane (64 in this case).
* `resolution`: The resolution of the vertical scan (1 degree per sample).
* `min_angle` and `max_angle`: The range of angles scanned vertically (from -45 degrees to 45 degrees).

---

### Adding Depth Camera

**Description:** simulates a sensor like a Kinect, which is duplicated in the Kinect plugin.

```xml
 <link name="camera_link">
    <inertial>
        <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
        <mass value="0.1"/>
        <inertia ixx="0.0" ixy="0.0" ixz="0.0" iyy="0.0" iyz="0.0" izz="0.0"/>
    </inertial>
    <visual name="camera_link_visual">
        <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
        <geometry>
         <mesh filename="package://robot_describtion/meshes/kinect.dae"/>
          </geometry>
        <material name="">
            <color rgba="0.0 1.0 0.0 1.0"/>
  
        </material>
    </visual>
    <collision>
        <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
        <geometry>
            <box size="0.1 0.1 0.1"/>
        </geometry>
    </collision>
</link>
<joint name="camera_joint" type="fixed">
 <origin xyz="0 0.0 0.28" rpy="0 0 0"/>
  <parent link="base_link"/>
    <child link="camera_link"/>
    <axis xyz="0.0 0.0 0.0"/>
    <limit lower="0.0" upper="0.0" effort="0.0" velocity="0.0"/>
</joint>
```

### Gazebo Plugin of Depth Camera

```xml
<gazebo reference="${link_name}">
  <sensor name="${link_name}_camera" type="depth">
    <update_rate>20</update_rate>
    <camera>
      <horizontal_fov>1.047198</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.05</near>
        <far>3</far>
      </clip>
    </camera>
    <plugin name="${link_name}_controller" filename="libgazebo_ros_openni_kinect.so">
      <baseline>0.2</baseline>
      <alwaysOn>true</alwaysOn>
      <updateRate>1.0</updateRate>
      <cameraName>${camera_name}_ir</cameraName>
      <imageTopicName>/${camera_name}/color/image_raw</imageTopicName>
      <cameraInfoTopicName>/${camera_name}/color/camera_info</cameraInfoTopicName>
      <depthImageTopicName>/${camera_name}/depth/image_raw</depthImageTopicName>
      <depthImageInfoTopicName>/${camera_name}/depth/camera_info</depthImageInfoTopicName>
      <pointCloudTopicName>/${camera_name}/depth/points</pointCloudTopicName>
      <frameName>${frame_name}</frameName>
      <pointCloudCutoff>0.5</pointCloudCutoff>
      <pointCloudCutoffMax>3.0</pointCloudCutoffMax>
      <distortionK1>0.00000001</distortionK1>
      <distortionK2>0.00000001</distortionK2>
      <distortionK3>0.00000001</distortionK3>
      <distortionT1>0.00000001</distortionT1>
      <distortionT2>0.00000001</distortionT2>
      <CxPrime>0</CxPrime>
      <Cx>0</Cx>
      <Cy>0</Cy>
      <focalLength>0</focalLength>
      <hackBaseline>0</hackBaseline>
    </plugin>
  </sensor>
</gazebo>

```

### Explanation

* **`<gazebo reference="${link_name}">`** : This line links the Gazebo sensor definition to a specific link in the URDF model. The `${link_name}` is a placeholder and should be replaced with the actual name of the link to which the sensor is attached.
* **`<sensor name="${link_name}_camera" type="depth">`** : This defines a new sensor of type `depth`, with a name based on the link name.
* **`<camera>`** : This block contains settings specific to the depth camera.
* **`<horizontal_fov>1.047198</horizontal_fov>`** : Sets the horizontal field of view (in radians).
* **`<image>`** : Defines the properties of the image produced by the camera.

  * **`<width>640</width>`** : Sets the image width to 640 pixels.
  * **`<height>480</height>`** : Sets the image height to 480 pixels.
  * **`<format>R8G8B8</format>`** : Sets the image format to RGB.
* **`<plugin name="${link_name}_controller" filename="libgazebo_ros_openni_kinect.so">`** : This block defines the Gazebo plugin that will simulate the depth camera. The plugin name and filename attributes should be set accordingly.
* **`<baseline>0.2</baseline>`** : Sets the baseline distance between the infrared projector and the infrared sensor (typically for stereo vision).
* **`<alwaysOn>true</alwaysOn>`** : Ensures the camera is always active.
* **`<updateRate>1.0</updateRate>`** : Sets the update rate of the plugin to 1 Hz.
* **`<cameraName>${camera_name}_ir</cameraName>`** : Sets the name of the camera in the plugin.
* **`<imageTopicName>/${camera_name}/color/image_raw</imageTopicName>`** : Sets the ROS topic for the color image.
* **`<cameraInfoTopicName>/${camera_name}/color/camera_info</cameraInfoTopicName>`** : Sets the ROS topic for the camera information.
* **`<depthImageTopicName>/${camera_name}/depth/image_raw</depthImageTopicName>`** : Sets the ROS topic for the depth image.
* **`<depthImageInfoTopicName>/${camera_name}/depth/camera_info</depthImageInfoTopicName>`** : Sets the ROS topic for the depth camera information.
* **`<pointCloudTopicName>/${camera_name}/depth/points</pointCloudTopicName>`** : Sets the ROS topic for the point cloud data.
* **`<frameName>${frame_name}</frameName>`** : Sets the reference frame for the sensor data.
* **`<pointCloudCutoff>0.5</pointCloudCutoff>`** : Sets the minimum distance for point cloud data.
* **`<pointCloudCutoffMax>3.0</pointCloudCutoffMax>`** : Sets the maximum distance for point cloud data.

  ---

  ### Adding Mono Camera

```xml

 <link name="camera_link">
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
    <box size="${camera_link} ${camera_link} ${camera_link}"/>
      </geometry>
    </collision>

    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
    <box size="${camera_link} ${camera_link} ${camera_link}"/>
      </geometry>
      <material name="red"/>
    </visual>

    <inertial>
      <mass value="1e-5" />
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6" />
    </inertial>
  </link>
  <joint name="camera_joint" type="fixed">
    <axis xyz="0 1 0" />
    <origin xyz="${camera_link} 0 0" rpy="0 0 0"/>
    <parent link="link3"/>
    <child link="camera_link"/>
  </joint>
```

### Gazebo Plugin for Mono Camera

```xml
<gazebo reference="camera_link">
  <sensor type="camera" name="camera1">
    <update_rate>30.0</update_rate>
    <camera name="head">
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>800</width>
        <height>800</height>
        <format>L8</format> 
      </image>
      <clip>
        <near>0.02</near>
        <far>300</far>
      </clip>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.007</stddev>
      </noise>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <alwaysOn>true</alwaysOn>
      <updateRate>0.0</updateRate>
      <cameraName>rrbot/camera1</cameraName>
      <imageTopicName>image_raw</imageTopicName>
      <cameraInfoTopicName>camera_info</cameraInfoTopicName>
      <frameName>camera_link</frameName>
      <hackBaseline>0.07</hackBaseline>
      <distortionK1>0.0</distortionK1>
      <distortionK2>0.0</distortionK2>
      <distortionK3>0.0</distortionK3>
      <distortionT1>0.0</distortionT1>
      <distortionT2>0.0</distortionT2>
    </plugin>
  </sensor>
</gazebo>
```

### Explanation

* **`<gazebo reference="camera_link">`** : Associates this sensor configuration with a specific link in the URDF, named `camera_link`.
* **`<sensor type="camera" name="camera1">`** : Defines a camera sensor named `camera1`.
* **`<horizontal_fov>1.3962634</horizontal_fov>`** : Sets the horizontal field of view to approximately 80 degrees (1.3962634 radians).
* **`<image>`** :
* **`<width>800</width>`** : Sets the image width to 800 pixels.
* **`<height>800</height>`** : Sets the image height to 800 pixels.
* **`<format>R8G8B8</format>`** : Sets the image format to mono format  `L8` or `MONO8` for grayscale images.

---

### Adding an Ultrasonic sensor

```xml
<link name="ultrasonic_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.1"/> 
      </geometry>
      <material name="grey"/>
    </visual>
  </link>

  <joint name="ultrasonic_joint" type="fixed">
    <parent link="base_link"/> 
    <child link="ultrasonic_link"/>
    <origin xyz="0.25 0 0"/> 
  </joint>
```

#### Gazebo Plugin for Ultrasonic sensor

```xml
<gazebo reference="ultrasonic_link">
    <sensor type="ray" name="sonar">
      <pose>0 0 0 0 0 0</pose>
      <visualize>true</visualize>
      <update_rate>5</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>5</samples>
            <resolution>1.0</resolution>
            <min_angle>-0.25</min_angle>
            <max_angle>0.25</max_angle>
          </horizontal>
          <vertical>
            <samples>5</samples>
            <resolution>1</resolution>
            <min_angle>-0.25</min_angle>
            <max_angle>0.25</max_angle>
          </vertical>
        </scan>
        <range>
          <min>0.01</min>
          <max>0.75</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin filename="libgazebo_ros_range.so" name="gazebo_ros_range">
        <gaussianNoise>0.005</gaussianNoise>
        <alwaysOn>true</alwaysOn>
        <updateRate>5</updateRate>
        <topicName>/genius/sonar</topicName>
        <frameName>ultrasonic</frameName>
        <fov>0.5</fov>
        <radiation>ultrasound</radiation>
      </plugin>
    </sensor>
  </gazebo>
```

#### Explaination

* **`<sensor type="ray" name="sonar">`** : Defines a ray-based sensor, which simulates an ultrasonic sensor.
* **`<scan>`** : Specifies the scanning parameters.

  * **`<horizontal>`** : Defines the horizontal scanning parameters.
  * `<samples>5</samples>`: Number of horizontal samples.
  * `<resolution>1.0</resolution>`: Resolution of the horizontal scan.
  * `<min_angle>-0.25</min_angle>`: Minimum horizontal angle.
  * `<max_angle>0.25</max_angle>`: Maximum horizontal angle.
    * **`<vertical>`** : Defines the vertical scanning parameters.
  * `<samples>5</samples>`: Number of vertical samples.
  * `<resolution>1</resolution>`: Resolution of the vertical scan.
  * `<min_angle>-0.25</min_angle>`: Minimum vertical angle.
  * `<max_angle>0.25</max_angle>`: Maximum vertical angle.
* **`<range>`** : Specifies the range of the ultrasonic sensor.

  * `<min>0.01</min>`: Minimum range distance (0.01 meters).
  * `<max>0.75</max>`: Maximum range distance (0.75 meters).
  * `<resolution>0.01</resolution>`: Range resolution.
* `<topicName>/genius/sonar</topicName>`: Topic name where the sensor data is published.
* `<frameName>ultrasonic</frameName>`: Frame of reference for the sensor data.
* `<fov>0.5</fov>`: Field of view of the sensor.
* `<radiation>ultrasound</radiation>`: Specifies that the sensor uses ultrasound radiation

  ---

### Adding an IMU Sensor

```xml
 <link name="imu_link">
        <inertial>
            <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
            <mass value="0.0"/>
            <inertia ixx="0.0" ixy="0.0" ixz="0.0" iyy="0.0" iyz="0.0" izz="0.0"/>
        </inertial>
        <visual name="">
            <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
            <geometry>
                <box size="0.01 0.01 0.01"/>
      
            </geometry>
            <material name="">
                <color rgba="1.0 0.0 0.0 1.0"/>
                <texture filename=""/>
            </material>
        </visual>
        <collision>
            <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
          <geometry>
            <box size="0.01 0.01 0.01"/>
  
            </geometry>
        </collision>
    </link>
<joint name="imu_joint" type="fixed">
        <origin xyz="0.0 0.0 0.04" rpy="0.0 0.0 0.0"/>
        <parent link="base_link"/>
        <child link="imu_link"/>
    </joint>
```

### Gazebo Plugin

```xml
  <gazebo reference="imu_link">  <!--the name of the link-->
    <gravity>true</gravity>
    <sensor name="imu_sensor" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <visualize>true</visualize>
      <topic>__default_topic__</topic>
      <plugin filename="libgazebo_ros_imu_sensor.so" name="imu_plugin">
        <topicName>imu</topicName>
        <bodyName>imu_link</bodyName>
        <updateRateHZ>100.0</updateRateHZ>
        <gaussianNoise>0.0</gaussianNoise>
        <xyzOffset>0 0 0</xyzOffset>
        <rpyOffset>0 0 0</rpyOffset>
        <frameName>imu_link</frameName>
      </plugin>
      <pose>0 0 0 0 0 0</pose>
    </sensor>
  </gazebo>
```

---

## **robot drive systems**

#### **Differential Drive**

Uses two independently driven wheels on either side of the robot.

##### Gazebo plugin

```xml
<gazebo>
  <plugin name="differential_drive_controller" filename="libgazebo_ros_diff_drive.so">

    <!-- Plugin update rate in Hz -->
    <updateRate>${update_rate}</updateRate>

    <!-- Name of left joint, defaults to `left_joint` -->
    <leftJoint>base_link_left_wheel_joint</leftJoint>

    <!-- Name of right joint, defaults to `right_joint` -->
    <rightJoint>base_link_right_wheel_joint</rightJoint>

    <!-- The distance from the center of one wheel to the other, in meters, defaults to 0.34 m -->
    <wheelSeparation>0.5380</wheelSeparation>

    <!-- Diameter of the wheels, in meters, defaults to 0.15 m -->
    <wheelDiameter>0.2410</wheelDiameter>

    <!-- Wheel acceleration, in rad/s^2, defaults to 0.0 rad/s^2 -->
    <wheelAcceleration>1.0</wheelAcceleration>

    <!-- Maximum torque which the wheels can produce, in Nm, defaults to 5 Nm -->
    <wheelTorque>20</wheelTorque>

    <!-- Topic to receive geometry_msgs/Twist message commands, defaults to `cmd_vel` -->
    <commandTopic>cmd_vel</commandTopic>

    <!-- Topic to publish nav_msgs/Odometry messages, defaults to `odom` -->
    <odometryTopic>odom</odometryTopic>

    <!-- Odometry frame, defaults to `odom` -->
    <odometryFrame>odom</odometryFrame>

    <!-- Robot frame to calculate odometry from, defaults to `base_footprint` -->
    <robotBaseFrame>base_footprint</robotBaseFrame>

    <!-- Odometry source, 0 for ENCODER, 1 for WORLD, defaults to WORLD -->
    <odometrySource>1</odometrySource>

    <!-- Set to true to publish transforms for the wheel links, defaults to false -->
    <publishWheelTF>true</publishWheelTF>

    <!-- Set to true to publish transforms for the odometry, defaults to true -->
    <publishOdom>true</publishOdom>

    <!-- Set to true to publish sensor_msgs/JointState on /joint_states for the wheel joints, defaults to false -->
    <publishWheelJointState>true</publishWheelJointState>

    <!-- Set to true to swap right and left wheels, defaults to true -->
    <legacyMode>false</legacyMode>
  </plugin>
</gazebo>
```

## Skid Steering Drive

```xml
<gazebo>
    <plugin name="skid_steer_controller" filename="libgazebo_ros_skid_steer_drive.so">
      <alwaysOn>true</alwaysOn>
      <updateRate>100.0</updateRate>
      <leftFrontJoint>front_left_wheel_joint</leftFrontJoint>
      <leftRearJoint>rear_left_wheel_joint</leftRearJoint>
      <rightFrontJoint>front_right_wheel_joint</rightFrontJoint>
      <rightRearJoint>rear_right_wheel_joint</rightRearJoint>
      <cmdVelTopic>cmd_vel</cmdVelTopic>
      <odometryTopic>odom</odometryTopic>
      <robotBaseFrame>base_link</robotBaseFrame>
    </plugin>
  </gazebo>
```
