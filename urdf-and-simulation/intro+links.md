# URDF

---

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
<?xml version="1.0"?>
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
  <material name="white">
    <color rgba="1.0 1.0 1.0 1.0"/>
  </material>
</visual>
```

![link](https://github.com/user-attachments/assets/54bb6764-9fe7-4eb6-9d5d-8bdec9e98103)

#### Collision

here we define the collision properties of the link.

Collision is the physical ascpect of the link which the simulation sees and react to. for example if we have a link that has a box geometry in the visual tag whereas it has a cylinder geometry in the collision tag we as a users will see the link as a box but the simulation will react to it as a cylinder. so to simplify, the collision is how the simulation handles the link.

```xml
<collision>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <cylinder radius="1" length="0.5"/>
  </geometry>
</collision>
```

let's say we have a link that has a visual and collision with a box geometry but with different values.

```xml
<?xml version="1.0"?>
<robot name="my_robot">
    <link name="link_name">
        <visual name="">
            <geometry>
                <box size="1.0 1.0 1.0"/>
            </geometry>
            <material name="white">
                <color rgba="1.0 1.0 1.0 1.0"/>
                <texture filename=""/>
            </material>
        </visual>
        <collision>
            <geometry>
                <box size="2.0 2.0 2.0"/>
            </geometry>
        </collision>
    </link>
</robot>
```

after we add the collision tag we won't see anychanges in urdf preview as you can't preserve the collisions because it's just for the simulator to deal with but to help you understand what this looks like see this picture.

![collision](https://github.com/user-attachments/assets/348c4696-96cb-4fcc-a4ca-7c5800b2441b)

we can see here the difference between the visual and collision tag they are very similar in a lot of attributes that they share such as geometry and origin but we notice that the collision tag doesn't have material in it and that's because the importance of the material tag is to make the robot aesthetically appealing and the simulator doesn't need that.

Also in most cases you don't want your robot to take actions that doesn't match what you see in the simulations so you'll probably need the collision tag to be the same as the visual tag.

Let's say we used a mesh to define the geometry of the visual tag because it's complicated like this robot arm. it's not recommended in most cases to use the same mesh for the collision as this affect the performance of the simulation making it more slow so if using a more simplified geometry that's not going to change the physical properties of the link is possible that will be better in most situations.

![Robot arm](https://github.com/user-attachments/assets/5d6e3a22-e0d1-408e-8354-e8dc4d0afd82)

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

the inertia matrix is a 3*3 matrix that has these values.

![Inertia matrix](https://github.com/user-attachments/assets/a96324fc-2784-4786-aa5b-1e16caa15a41)

But as we see in the code it only has 6 attributes and thats because it's a symetrical matrix so we don't have to define the nondiagonal elements twice.

```xml
<inertial>
  <mass value="10"/>
  <inertia ixx="0.4" ixy="0.0" ixz="0.0" iyy="0.4" iyz="0.0" izz="0.2"/>
</inertial>
```

As for the values of the inertia tag we can calculate them based on the shape, dimensions and mass of the link and we can find the formulas to calculate these values in wikipedia  in this link: [https://en.wikipedia.org/wiki/List_of_moments_of_inertia]()

![inertia](https://github.com/user-attachments/assets/d5e7735b-6115-408e-a8b2-0f8dfa806508)

the inertial tag is a very important tag as if we ignore it this could cause proplems in gazebo during simulation causing the robot model to collapse without warning, and all links
will appear with their origins coinciding with the world origin.

Xacro
-----

After we learned everything we need now we are ready to start building the links of our robot that by the end will look something like this.

![robot body](https://github.com/user-attachments/assets/76944e0d-e6e6-4063-b355-c57350fee884)

the robot consisits of five links the base link which is a box shaped and four wheels which are cylindercal shaped.

```xml
<?xml version="1.0"?>
<robot name="my_robot">
    <link name="base_link">
        <visual name="">
            <origin xyz="0.0 0.0 0.1" rpy="0.0 0.0 0.0"/>
            <geometry>
                <box size=".4 .8 .1"/>
            </geometry>
            <material name="green">
                <color rgba="0.0 1.0 0.0 1.0"/>
            </material>
        </visual>
        <collision>
            <geometry>
                <box size=".4 .8 .1"/>
            </geometry>
        </collision>
    </link>
    <link name="front_right_wheel">
        <visual name="">
            <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
            <geometry>
                <cylinder radius=".12" length=".05"/>
            </geometry>
            <material name="red">
                <color rgba="1.0 0.0 0.0 1.0"/>
            </material>
        </visual>
        <collision>
            <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
            <geometry>
                <cylinder radius=".12" length=".05"/>
            </geometry>
        </collision>
    </link>
</robot>
```

![my_robot](https://github.com/user-attachments/assets/3a547566-e619-4793-93bf-a297a8932bb8)

After writing the descriotion of the base link and wheel link we get this output which are not we desire.

the first problem is the wheel placement and that is the importance of joints and these are the elements that holds the links together and define where the links should connect and how they move relative to each other.

Also we only defined one wheel which is the front right wheel so we will have to write the wheel description three more times for the other wheels and that's not really practical since it will take more time, is harder to track errors if there's one and if we want to change any thing regarding the wheel we'll have to do this change in all the wheels manually.

For all these reasons comes the xacro files to help us organize our code, make it simpler to read and modify and avoid repeation like in the wheel link for our case.

**Xacro** is an Xml macro that introduce the use of macros in an urdf file.

**Macros** are used to make a sequence of computing instructions available to the programmer as a single statement. to put it in simple words we can think of macros as functions in most programming languages that  we can difine it once and call it multiple times.

**Xacro** is just another way of defining a URDF, not an alternative to it.

**Xacro** files help us do three things that are very helpful.

* Constants
* Simple Math
* Macros

Before getting to the details of Xacro first we need to know how to use it in our urdf files. we will have to change the robot tag and add this line to it. 

```xml
we will add this line to the robot tag: xmlns:xacro="http://www.ros.org/wiki/xacro"
so it will look like this
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="my_robot">
```

now we are ready to use Xacro in our file

### Constants

First thing Xacro files allows us to do is to use constants in our code so we can change values in the whole file from one place.

Let's take a look at our wheel link.

```xml
<link name="wheel">
    <visual name="">
        <geometry>
            <cylinder radius=".12" length=".05"/>
        </geometry>
        <material name="red">
            <color rgba="1.0 0.0 0.0 1.0"/>
        </material>
    </visual>
    <collision>
        <geometry>
            <cylinder radius=".12" length=".05"/>
        </geometry>
    </collision>
</link>
```

we will notice here that the raduis and length of the cylinder are specified twice once in the visual tag and another time for the collision tag and also take into account that this only for one wheel so if we have four wheels we will have to specify each value eight times and that is not convenient so we will use constants to replace those values.

to define constants we usually do this at the top of the file in the robot tag before any links or joints.

we will xacro property tag and this tag has two attributes the name of the constant and the value. and in code it looks like this

```xml
<xacro:property name="raduis" value=".12" />
<xacro:property name="length" value=".05" />
```

after we defined the two constants it's time to use them in our wheel link.

to use constants in your description we simply replace the value we want to specify with the dollor sign and curly brackets that has the constant name in them so it will look like this  `${constant name} .`

so after we define the constants and use them our link will look like this.

```xml
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="my_robot">
    <xacro:property name="raduis" value="0.12" />
    <xacro:property name="length" value="0.05" />
    <link name="wheel">
        <visual name="">
            <geometry>
                <cylinder radius="${raduis}" length="${length}"/>
            </geometry>
            <material name="red">
                <color rgba="1.0 0.0 0.0 1.0"/>
            </material>
        </visual>
        <collision>
            <geometry>
                <cylinder radius="${raduis}" length="${length}"/>
            </geometry>
        </collision>
    </link>
</robot>
```

the value of the constant specified are used to replace the ${constant}. this means that we can combine texts using constants. 

```xml
<xacro:property name=”Robot_name” value=”My_robot” />
<link name=”${Robot_name}_wheel” />
the output of this will be as same as this
<link name=”My_robot_wheel” />

```

### Math

We can use Xacro files to make math expressions in the urdf file using basic operations (+,-,*,/) and parenthesis.

for example if there is a relation between the raduis and length of my wheel (raduis = 2 * length) we can only define one of these valuse and replace the other with this relation.

```xml
<xacro:property name="length" value="0.05" />

<cylinder radius="${2*length}" length="${length}"/>
```

this is most used in the inertia tag since it has more complex expressions so it's not convenient to calculate each value each time you change one dimension.

### Macros

As we said before macros are more like functions that we can write a chunk of code in them and call this macro multiple time with different parameters or with no parameters at all.

to define a macro we will use the macro tag which has two attributes the name of the macro and the parameters we want to define. and it looks like this.

```xml
<xacro:macro name="name_of_macro" params="param_1 param_2 param_3">
    Add your content here
</xacro:macro>
```

ok now we will utilize everything we have learned so far to make a wheel macro and call it four times to generate the four wheels of the robot.

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="my_robot">

    <xacro:property name="length" value="0.05" />

    <link name="base_link">
        <visual name="">
            <origin xyz="0.0 0.0 0.1" rpy="0.0 0.0 0.0"/>
            <geometry>
                <box size=".4 .8 .1"/>
            </geometry>
            <material name="green">
                <color rgba="0.0 1.0 0.0 1.0"/>
            </material>
        </visual>
        <collision>
            <geometry>
                <box size=".4 .8 .1"/>
            </geometry>
        </collision>
    </link>

    <xacro:macro name="leg" params="prefix">
        <link name="${prefix}_wheel">
            <visual name="">
                <geometry>
                    <cylinder radius="${length*2}" length="${length}"/>
                </geometry>
                <material name="red">
                    <color rgba="1.0 0.0 0.0 1.0"/>
                </material>
            </visual>
            <collision>
                <geometry>
                    <cylinder radius="${length*2}" length="${length}"/>
                </geometry>
            </collision>
        </link>
    </xacro:macro>
    <xacro:leg prefix="front_right" />
    <xacro:leg prefix="front_left" />
    <xacro:leg prefix="rear_right" />
    <xacro:leg prefix="rear_left" />
</robot>
```
And the output of this code will look like this:

![Final_links](https://github.com/user-attachments/assets/0d62a1c2-4088-45da-bb65-20d2f7e523d0)

Now it's time to put these links in there desired place and we will use joints to achieve this.
