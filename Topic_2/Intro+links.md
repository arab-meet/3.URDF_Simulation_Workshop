# URDF

Author: **Yousef Asal**

---

**URDF** (unified robotics description format) is an `XML` file format that includes the robot description in it. it provides details on the physical characteristics of the robot in addition to all the information about the shape, geometry, and colors of the robot.

**URDF** files are the format that ros uses to identify any robot and its interactions with the environment.

**URDF** files does not only contain the characteristics of the body of the robot but it can also contain informations about the sensors and motors of the robot. also it can contain some plugins to make it compatable with different programs like gazebo.

## Describing the robot

URDF describes the robot as a set of links that is connected with each other through some joints that defines the relative motion between these links. so when we have a robot that we want to describe in a URDF file we need to split up our robot to a set of links and joints.

URDF can represent the kinematic and dynamic description of the robot, the `visual`
representation of the robot, and the `collision` model of the robot.

Since the URDF file is based on XML format so there is different `XML tags` that we need to know to help create the robot.

in a typical URDF file we will find that it contains three main tags which are:

* robot
* links
* joints

### Robot tag

first thing we find in an XML file type is the declaration tag which defines the XML version that we are using. This line usually found in the first line in a URDF file

```
?xml version="1.0"?
```

after this we difine the main tag that will contain links and joints tags in it and this is the robot tag. we find that the robot tag has one attribute which is the name and here we can specity the name of our robot

```
<robot name="robot_name">
   all the tags of the robot goes here
    <link> ..... </link>
    <link> ...... </link>
    <joint> ....... </joint>
    <joint> ........</joint>
</robot>
```

### Link

The link of the robot represents the physical component of the robot.The link tag represents a single link of a robot. Using this tag, we can model a robot link and its properties. These properties include:

* visual
* collision
* inertial

we can have multible instances of the collision and visual tags in the same link if we want but only one inertial tag per link.

all three tags are optional meaning that for example you can have a link with just `visual` but no `collision` nor `inertial` tags.

```
<link name="<link_name>">
<inertial>...........</inertial>
<visual> ............</visual>
<collision>..........</collision>
</link>
```



---



#### Visual

the visual tag has the information of how we see the robot in the simulation here we can specify the size, shape, and color of the robot.

the visual tag has three main tags which are:

##### Geometry

defines the shape of the visual.

we have a set of different shapes in the URDF format which are:

* box: has three attributes the length, width, and height.
* cylinder: has two attributes which are the raduis, and length.
* sphere: has one attribute the raduis.
* mesh: here we can use an existing mesh by adding the file path. also it's recommended that the mesh's format would be .dae file as this format provide the best texture and color support

  ![1722971129026](image/URDFcopy/1722971129026.png)

  ```
  #for the box
  <geometry>
      <box size="1.0 2.0 1.0"/>
  </geometry>
  #for the cylinder
  <geometry>
      <cylinder radius="1" length="0.5"/>
  </geometry>
  #for the sphere
  <geometry>
      <sphere radius="1"/>
  </geometry>
  ```

##### Origin

defines the reference frame of the visual relative to the reference frame of the link.

has six attributes which are `xyz` and `rpy` and the default of these values are zeros.

```
<origin rpy="0 0 0" xyz="0 0 0"/>
```

##### Material

here we can specify two tags for the visual of our link which are:

* color: has for attributes `rgba` each one has a range from 0 to 1
* texture: is specified by a file name
  note that the color disappears when we run our robot on gazebo since it should have a certain plugin installed to show the link's color in gazebo.

  ```
  <material name="white"/>
  ```

we also have an extra tag which is the `name` that gives a name to a part of the link which can be useful when you have complex links and you want to access certain bits of a link.

note that all these properties of the visual tags are optional except for the geometry tag.

```
<visual>  
  <origin xyz="0 0 0" rpy="0 0 0" />
  <geometry>
    <box size="1 1 1" />
  </geometry>
  <material name="Cyan">
    <color rgba="0 1.0 1.0 1.0"/>
  </material>
</visual>
```



---

#### Collision

here we define the collision properties of the link.

the collision properties in most cases will ba the same as the visual properties but in a more complex links we tend to make a simpler collision models to reduce the computational power and make the simulation smoother.

as the visual tag the collision tag has `Geometry` and `Origin` but it does not have a material tag since it is not visible in the simulation.

```
<collision>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <cylinder radius="1" length="0.5"/>
  </geometry>
</collision>
```



---

#### Inertial

here we can specify informations about the link's mass, center of mass, and inertia.

has three main tags which are:

##### origin

this describes the center of mass relative to the link frame.

it has six attributes `xyz` for position and `rpy` for orientation.

##### mass

defines the mass of the link and it has only one attribute the link's mass

##### inertia

this takes a matrix that contains information about the moment of inertia for the link around the xyz axes. it also contains the products of inertia about the center of mass.

note that URDF assumes a negative product of inertia convention.

```
<inertial>
  <mass value="10"/>
  <inertia ixx="0.4" ixy="0.0" ixz="0.0" iyy="0.4" iyz="0.0" izz="0.2"/>
</inertial>
```

In the end we find that the final result of a link looks something like this:

![1722968398405](image/URDFcopy/1722968398405.png)

```
<link name="<link_name>">
  <visual>  
    <origin xyz="0 0 0" rpy="0 0 0" />
    <geometry>
      <box size="1 1 1" />
    </geometry>
    <material name="Cyan">
      <color rgba="0 1.0 1.0 1.0"/>
    </material>
  </visual>
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <cylinder radius="1" length="0.5"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="10"/>
    <inertia ixx="0.4" ixy="0.0" ixz="0.0" iyy="0.4" iyz="0.0" izz="0.2"/>
  </inertial>
</link>
```
