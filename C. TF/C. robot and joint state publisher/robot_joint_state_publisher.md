
author: Mo3taz

# Robot State Publisher


ROS gives you a Node called robot state publisher. In essence, it takes a file describing URDF file as input, and it automatically publishes the TF for you. Sounds pretty nice, right?

![robot state publisher](<images/robot_state_publisher_node.png>)

### Purpose and Functionality:
* The robot_state_publisher allows you to broadcast the state of a robot to the tf transform library.
* It internally maintains a kinematic model of the robot, which includes information about its links, joints, and transformations.
* Given the joint positions of the robot (obtained from the joint_states topic), the robot_state_publisher computes and broadcasts the 3D pose of each link in the robot.


### How It Works:

- To understad what i mean:

let's load urdf in **robot_description** and run **robot state publisher**
```xml
<launch>

    <arg name="model" default="$(find robot_description_pkg)/urdf/arabian_robot.urdf"/>
    <param name="robot_description" textfile="$(arg model)" />
    <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" respawn="false" output="screen"/>
</launch>
```
To run this

```bash
roslaunch robot_description_pkg arabian_gazebo.launch 

```

```bash
roslaunch robot_description_pkg arabian_robot_state.launch 

```
Open `rqt_tf_tree`

![robot state publisher](<images/only_robot_state.png>)
```bash
roslaunch robot_description_pkg arabian_gazebo.launch 

```


As you can see in the tf_tree, there are some links not here `front_right_wheel`, `front_left_wheel`! Why does this happen?

#### If you noticed:

**these links connected with non fixed joint**

This is because non-fixed joints, like this ***front_left_wheel_joint***, can be in any configuration, The TF  depends on the value of the joint, joints like encoder or other actuators.

**to solve this problem we need to publish encoder reading to change joint state for example if encoder increase by 5 ticks this map with 0.01 rad we need to change transformation by this degree**

#### And here is where the `joint_state_publisher` comes in.



# Joint_State_Publisher

The joint_state_publishera valuable tool for managing joint states For Non fixed joint in the robot.

### Overview:
* The joint_state_publisher is a ROS package that publishes `sensor_msgs/JointState`messages for a robot described on topic `/joint_states`

![joint state publisher](<images/joint state publisher.png>)


### to understeand joint state publisher :

frist we need to understand `joint_State` massage
Massage consist of:

* `Header`: Contains the timestamp of the message.
* `name`: List of joint names.
* `position`: List of joint positions.
* `velocity`: List of joint velocities.
* `effort`: List of efforts applied to the joints.

1. let's run this code :
```xml
<launch>

    <arg name="model" default="$(find robot_description_pkg)/urdf/arabian_robot.urdf"/>
    <param name="robot_description" textfile="$(arg model)" />
    <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" respawn="false" output="screen"/>
    
    <node name="joint_state_publisher_gui" pkg="joint_state_publisher_gui" type="joint_state_publisher_gui"/>

    <node name="rviz" pkg="rviz" type="rviz" args="-d $(find robot_description_pkg)/rviz/rviz_fake.rviz"/>

</launch>
```

TO RUN:

```bash
roslaunch robot_description_pkg arabian_gazebo.launch 

```

```bash
roslaunch robot_description_pkg arabiadn_robot_joint.launch 

```
Here we publish fake joint state with `joint_state_publisher_gui` to make non fixed joint movable 

**robot state publisher will accept this joint state and as we see here the transformation with `front_right_wheel`, `front_left_wheel` appear**


![tf tree](<images/fake_tree.png>)

**and you can make non fixed joint move like here**

![tf tree](<images/fake_send_joint_State.gif>)

![tf tree rahal](<images/rahal_fake_send_joint_State.gif>)
### As you can see we make change in joint state by our hand, the question is How this done in Real world or in simulation?


#### in `Gazebo simulation`
 we use the **controller_manager** ROS package. It allows you to simulate different types of PID control, like Position Control or Effort Control.



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