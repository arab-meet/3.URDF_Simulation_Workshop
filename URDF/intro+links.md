# URDF

**URDF** (unified robot description format) is an `XML` file format that includes the robot description in it. it provides details on the physical characteristics of the robot in addition to all the information about the shape, geometry, and colors of the robot.

**Extensible Markup Language (XML)** uses markup symbols, called tags to define data. Each instance of an XML tag is called an element. In an XML file, elements
are arranged in a hierarchy, which means that elements can contain other elements. The topmost element is called the “root” element and contains all other elements, which are called “child” elements.

The main purpose of **XML** language is to store data in an easy way that human can read and modefy and also the computers can understand. **XML** itself can't interact with the data it stores in any way so we will need another software to utilize these information such as rviz and gazebo in our case

**URDF** files are the format that ros uses to identify any robot and its interactions with the environment.

**URDF** files does not only contain the characteristics of the body of the robot but it can also contain informations about the sensors and motors of the robot. also it can contain some plugins to make it compatable with different programs like gazebo.

**URDF** is used to build the transformation tree (**TF**) for the robot so that ROS and other systems can interact with it.

## Describing the robot

**URDF** describes the robot as a set of links that is connected with each other through some joints that defines the relative motion between these links. so when we have a robot that we want to describe in a URDF file we need to split up our robot to a set of links and joints.

**URDF** can represent the kinematic and dynamic description of the robot, the `visual` representation of the robot, and the `collision` model of the robot.

Since the URDF file is based on XML format so there is different `XML tags` that we need to know to help create the robot.

in a typical URDF file we will find that it contains three main tags which are:

* robot
* links
* joints

![Robot](https://github.com/user-attachments/assets/b787527b-57ae-4ca6-bac9-4a52f33be1f4)

before we start writing our robot description we will need some extintions that will help us write our descrioption and visualize it along the way so we have a better idea of what we are making

there are three main extentions that we will need which are

* ROS extintion by microsoft  (this is the extintion that contain the urdf preview that lets us visualize our robot)
* ROS snippets by pijar          (this extintions has some snippets that helps us write our description more effeciently)
* URDF by smilerobotics         (this extintion also provides some premade structures that helps us write our code faster)

![Extentions](https://github.com/user-attachments/assets/f14191f6-c231-4d4f-bd19-4c46092eeb38)

**Note** that when you install the ROS extention you will have to chage the version to prerelease version in order for the urdf preview to work and after we change it we'll have to restart vscode for in order for our changes to take place.

![prerelease](https://github.com/user-attachments/assets/7e37cbc9-a2a5-4c9c-81bc-5414dce869ec)

after we have installed everything now we are ready to start descriping our robot.

### Robot tag

first thing we find in an XML file type is the declaration tag which defines the XML version that we are using. This line usually found in the first line in a URDF file

```xml
?xml version="1.0"?
```

after this we difine the main tag that will contain links and joints tags in it and this is the robot tag. we find that the robot tag has one attribute which is the name and here we can specity the name of our robot

```xml
<robot name="robot_name">
   all the tags of the robot goes here
    <link> ..... </link>
    <link> ...... </link>
    <joint> ....... </joint>
    <joint> ........</joint>
</robot>
```

Here we can see how to initialize our urdf file with the main tag that will contaion all the robot tags

![robot_tag](https://github.com/user-attachments/assets/a5b2a17c-d402-4656-b567-297dab6095a9)

now we can start the ros preview to see how our robot looks like in every step as we go. and we can do that by clicking  `Shift+Ctrl+p` and type `ROS: URDF Preview` that will open a new tab that show your robot

![ROS_preview](https://github.com/user-attachments/assets/93e38b8f-5d30-4f98-860e-7d38a6191ed5)

As we can see the preview screen is empty because we didn't add any links yet so now we will start to build our robot

### Link

The link of the robot represents the physical component of the robot.The link tag represents a single link of a robot. Using this tag, we can model a robot link and its properties. These properties include:

* visual
* collision
* inertial

we can have multible instances of the collision and visual tags in the same link if we want but only one inertial tag per link.

all three tags are optional meaning that for example you can have a link with just `visual` but no `collision` nor `inertial` tags.

```xml
<link name="<link_name>">
<inertial>...........</inertial>
<visual> ............</visual>
<collision>..........</collision>
</link>
```

![link](https://github.com/user-attachments/assets/b1764f87-0e65-427d-a39a-30c691532c56)

#### Visual

the visual tag has the information of how we see the robot in the simulation here we can specify the size, shape, and color of the robot.

the visual tag has three main tags which are:

##### Geometry

defines the shape of the visual.

we have a set of different shapes in the URDF format which are:

* box: has three attributes the length, width, and height.
* cylinder: has two attributes which are the raduis, and length.
* sphere: has one attribute the raduis.
* mesh: here we can use an existing mesh by adding the file path. also it's recommended that the mesh's format would be .dae file as this format provide the best texture and color support.

  ```xml
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

```xml
<origin rpy="0 0 0" xyz="0 0 0"/>
```

##### Material

here we can specify two tags for the visual of our link which are:

* color: has for attributes `rgba` each one has a range from 0 to 1
* texture: is specified by a file name
  note that the color disappears when we run our robot on gazebo since it should have a certain plugin installed to show the link's color in gazebo.

```xml
<material name="white"/>
```

we also have an extra tag which is the `name` that gives a name to a part of the link which can be useful when you have complex links and you want to access certain bits of a link.

note that all these properties of the visual tags are optional except for the geometry tag.

```xml
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


![link](https://github.com/user-attachments/assets/54bb6764-9fe7-4eb6-9d5d-8bdec9e98103)
After we learned how to use each tag to make a link now we will utilize these information to make our first link in our robot which is the base link.

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
