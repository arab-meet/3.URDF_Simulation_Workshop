###### Author: Yousef Asal

###### Review: KG

# URDF

**URDF** (unified robot description format) is an `XML` file format that includes the robot description in it.

* it's the format that ros uses to identify any robot and its interactions with the environment.
* provides details on the physical characteristics of the robot in addition to all the information about the shape, geometry, and colors of the robot.
* does not only contain the characteristics of the body of the robot but it can also contain informations about the sensors and motors of the robot.
* can contain some plugins to make it compatable with different programs like gazebo.
* is used to build the transformation tree (**TF**) for the robot so that `ROS` and other systems can interact with it.

**Extensible Markup Language (XML)** uses markup symbols, called tags to define data.

* Each instance of an `XML tag` is called an element.
* In an `XML` file, elements are arranged in a hierarchy, which means that elements can contain other elements.
* The topmost element is called the “root” element and contains all other elements, which are called “child” elements.
* **XML** itself can't interact with the data it stores in any way so we will need another software to utilize these information such as rviz and gazebo in our case

The main purpose of **XML** language is to store data in an easy way that human can read and modefy and also the computers can understand.

## Describing the robot

* **URDF** describes a robot as a set of links connected by joints, which define the relative motion between the links. To describe a robot in a URDF file, it must be divided into links and joints.
* **URDF** can represent:
  * The kinematic and dynamic description of the robot.
  * The `visual` representation of the robot.
  * The `collision` model of the robot.
* As **URDF** files are based on `XML` format, different **XML tags** are used to create the robot description.
* A typical URDF file contains three main tags:
  * **robot**
  * **links**
  * **joints**

![Robot](https://github.com/user-attachments/assets/b787527b-57ae-4ca6-bac9-4a52f33be1f4)

before we start writing our robot description we will need some extintions that will help us write our descrioption and visualize it along the way so we have a better idea of what we are making

there are three main extentions that we will need which are

* **ROS** extintion by microsoft  (this is the extintion that contain the urdf preview that lets us visualize our robot)
* **ROS snippets** by pijar          (this extintions has some snippets that helps us write our description more effeciently)
* **URDF** by smilerobotics         (this extintion also provides some premade structures that helps us write our code faster)

![Extentions](https://github.com/user-attachments/assets/f14191f6-c231-4d4f-bd19-4c46092eeb38)

> **Note** that when you install the `ROS` extention you will have to chage the version to prerelease version in order for the `urdf preview` to work and after we change it we'll have to restart `vscode` for in order for our changes to take place.

![prerelease](https://github.com/user-attachments/assets/7e37cbc9-a2a5-4c9c-81bc-5414dce869ec)

after we have installed everything now we are ready to start descriping our robot.

### `Robot tag`

first thing we find in an `XML` file type is the declaration tag which defines the version that we are using. This line usually found in the first line in a `URDF` file

```xml
<?xml version="1.0"?>
```

after this we difine the main tag that will contain links and joints tags in it and this is the robot tag. we find that the robot tag has one attribute which is the name and here we can specity the name of our robot

```xml
<robot name="robot_name">
    <!-- all the tags of the robot goes here -->
    <link> ..... </link>
    <link> ...... </link>
    <joint> ....... </joint>
    <joint> ........</joint>
</robot>
```

Here we can see how to initialize our urdf file with the main tag that will contaion all the robot tags

![robot_tag](https://github.com/user-attachments/assets/a5b2a17c-d402-4656-b567-297dab6095a9)

now we can start the **ros preview** to see how our robot looks like in every step as we go. and we can do that by clicking  `Shift+Ctrl+p` and type `ROS: URDF Preview` that will open a new tab that show your robot

![ROS_preview](https://github.com/user-attachments/assets/93e38b8f-5d30-4f98-860e-7d38a6191ed5)

As we can see the preview screen is empty because we didn't add any links yet so now we will start to build our robot

### `Link`

The link of the robot represents the physical component of the robot.The link tag represents a single link of a robot. Using this tag, we can model a robot link and its properties. These properties include:

* visual
* collision
* inertial

we can have multible instances of the `collision` and `visual` tags in the same link if we want but only one `inertial` tag per link.

all three tags are optional meaning that for example you can have a link with just `visual` but no `collision` nor `inertial` tags.

```xml
<link name="<link_name>">
<inertial>...........</inertial>
<visual> ............</visual>
<collision>..........</collision>
</link>
```

![link](https://github.com/user-attachments/assets/b1764f87-0e65-427d-a39a-30c691532c56)

### `Visual`

the visual tag describes the actual shape. here we can specify the size, shape, and color of the robot.

It has three main tags which are:

#### `Geometry`

describes the `shape` and `size` of the display range centered on the origin coordinates

we have a set of different shapes in the **URDF format** which are:

* **box**: has three attributes the `length`, `width`, and `height`.
* **cylinder**: has two attributes which are the `raduis`, and `length`.
* **sphere**: has one attribute the `raduis`.
* **mesh**: here we can use an existing mesh by adding the `file path`. also it's recommended that the mesh's format would be `.dae` file as this format provide the best texture and color support.

  ```xml
  <!-- for the box -->
  <geometry>
      <box size="1.0 2.0 1.0"/>
  </geometry>

  <!-- for the cylinder -->
  <geometry>
      <cylinder radius="1" length="0.5"/>
  </geometry>

  <!-- #for the sphere -->
  <geometry>
      <sphere radius="1"/>
  </geometry>
  ```

> **Note** When it is difficult to express it in a simple shape, **CAD** files such as `STL` and `DAE` can also be input here.

#### `Origin`

defines the reference frame of the `visual` relative to the reference frame of the link.

has six attributes which are `xyz` and `rpy` and the default of these values are zeros.

```xml
<origin rpy="0 0 0" xyz="0 0 0"/>
```

#### `Material`

here we can specify two tags for the visual of our link which are:

* **color**: The color tag is used to set the color by entering a number between `0.0` and `1.0` corresponding to `red`, `green`, and `blue` in the rgba option. The last number is the `transparency` (alpha), which has a value between `0.0` and `1.0`. If it is `1.0`, it means that the original color is displayed as is without using the transparent option.
* **texture**: is specified by a file name

> **Note** that the color disappears when we run our robot on `gazebo` since it should have a certain plugin installed to show the link's color in gazebo.

```xml
<material name="white"/>
```

we also have an extra tag which is the `name` that gives a name to a part of the link which can be useful when you have complex links and you want to access certain bits of a link.

> **Note** that all these properties of the `visual` tags are optional except for the **geometry** tag.

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

### `Collision`

* **Collision** describes the physical properties of the link that the simulation uses to detect and respond to interactions.
* The **collision** aspect is crucial for the simulation's behavior, as it dictates how the link is treated in terms of physics and interactions.
* For example:
  * If a link has a box geometry in the **visual** tag, but a cylinder geometry in the **collision** tag:
    * **Users** will see the link as a box.
    * **The simulation** will treat and react to the link as a cylinder.
* In summary, **collision** defines how the simulation handles the link, separate from its visual representation.

```xml
<collision>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <cylinder radius="1" length="0.5"/>
  </geometry>
</collision>
```

let's say we have a link that has a **visual** and **collision** with a `box` geometry but with different values.

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

after we add the collision tag we won't see any changes in **urdf preview** as you can't preserve the `collisions` because it's just for the simulator to deal with but to help you understand what this looks like see this picture.

![collision](https://github.com/user-attachments/assets/348c4696-96cb-4fcc-a4ca-7c5800b2441b)

* The **visual** and **collision** tags in `URDF` files are similar, sharing attributes like geometry and origin.
* The **collision** tag does not include a material attribute, as the `material` is primarily for aesthetic purposes, which are not needed by the simulator.
* To ensure the robot's actions in simulations match its appearance, the **collision** tag should generally match the **visual** tag.
* If a complex mesh is used for the `geometry` of the **visual** tag (e.g., a robot arm), it is not recommended to use the same mesh for the **collision** tag.
* Using a complex mesh for **collision** can degrade simulation performance, making it slower.
* In most cases, it is better to use a simplified geometry for the **collision** tag, provided it does not alter the physical properties of the link.

![Robot arm](https://github.com/user-attachments/assets/5d6e3a22-e0d1-408e-8354-e8dc4d0afd82)

### `Inertial`

here we can specify informations about the link's `mass`, `center of mass`, and `inertia`.

has three main tags which are:

#### `origin`

this describes the center of mass relative to the link frame.

it has six attributes `xyz` for position and `rpy` for orientation.

#### `mass`

defines the weight of the link(mass, unit: kg), and it has only one attribute the link's mass

#### `inertia`

* this takes a matrix that contains information about the moment of inertia for the link around the `xyz` axes. it also contains the products of inertia about the center of mass.
* note that **URDF** assumes a negative product of inertia convention.
* the inertia matrix is a `3*3 matrix` that has these values.

![Inertia matrix](https://github.com/user-attachments/assets/a96324fc-2784-4786-aa5b-1e16caa15a41)

But as we see in the code it only has **6 attributes** and thats because it's a symetrical matrix so we don't have to define the nondiagonal elements twice.

```xml
<inertial>
  <mass value="10"/>
  <inertia ixx="0.4" ixy="0.0" ixz="0.0" iyy="0.4" iyz="0.0" izz="0.2"/>
</inertial>
```

As for the values of the inertia tag we can calculate them based on the **shape**, **dimensions** and **mass** of the link and we can find the formulas to calculate these values in wikipedia  in this link: [https://en.wikipedia.org/wiki/List_of_moments_of_inertia]()

![inertia](https://github.com/user-attachments/assets/d5e7735b-6115-408e-a8b2-0f8dfa806508)

the `inertial` tag is a very important tag as if we ignore it this could cause proplems in `gazebo` during simulation causing the robot model to collapse without warning, and all links will appear with their origins coinciding with the world origin.

> **Note** This inertial information can be obtained through design software or actual measurements and calculations, and is mainly used in simulations.

`Xacro`
---

**URDF** files can be hard to manage because they often have repetitive code, especially when describing many links and joints. This can make changes difficult. `Xacro` is a tool that helps by reducing this repetitive work.

The **xacro** file is an abbreviation for `XML` Macro.

**Benefits of Using Xacro:**

* **Less Repetition:** `Xacro` lets you create macros to avoid writing the same code over and over. This makes your `URDF` file easier to read and manage.
* **Variables:** You can use variables, like `pi`, to simplify your code and make it more flexible.
* **Separate Files:** With `Xacro`, you can keep material information and `Gazebo` settings in separate files and include them in your main `URDF` file. This keeps everything organized and easier to update.

**Example Usage:**

The robot consisits of five links the base link which is a box shaped and four wheels which are cylindercal shaped.

![robot body](https://github.com/user-attachments/assets/76944e0d-e6e6-4063-b355-c57350fee884)

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

* After writing the description for the **base link** and  **wheel link** , the output is not as desired.
* The first problem is  **wheel placement** , highlighting the importance of  **joints** . Joints connect links and define their attachment points and relative movement.

Before getting to the details of `Xacro` first we need to know how to use it in our `urdf` files. we will have to change the robot tag and add this line to it.

```xml
<!-- we will add this line to the robot tag: xmlns:xacro="http://www.ros.org/wiki/xacro" -->
<!-- so it will look like this -->

<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="my_robot">
```

now we are ready to use `Xacro` in our file

### `Constants`

First thing `Xacro` files allows us to do is to use **constants** in our code so we can change values in the whole file from one place.

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

* In the current setup, the **radius** and **length** of the cylinder are specified twice: once in the `visual` tag and once in the `collision` tag.
* This redundancy means that for one wheel, these values are repeated eight times (visual and collision tags for each wheel). This approach is not convenient.
* To simplify, we will use **constants** to replace these repeated values.
* **Constants** are typically defined at the top of the file within the **robot** tag, before any links or joints.
* We will use the `Xacro`tag to define constants. This tag has two attributes:
  * **Name** : The name of the constant.
  * **Value** : The value assigned to the constant.
* In code, defining constants with the **Xacro property** tag looks like this:

```xml
<xacro:property name="raduis" value=".12" />
<xacro:property name="length" value=".05" />
```

* After defining the two constants, we need to use them in our **wheel link** description.
* To use constants in the description, replace the values with `${constant_name}`, where `constant_name` is the name of the constant you defined.
* The syntax to include a constant in your description is: `${constant_name}`.
* With the constants defined and used, your updated **wheel link** description will look like this:

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

the value of the constant specified are used to replace the `${constant}`. this means that we can combine texts using constants.

```xml
<xacro:property name=”Robot_name” value=”My_robot” />
<link name=”${Robot_name}_wheel” />

<!-- the output of the previous code will be as same as this -->

<link name=”My_robot_wheel” />
```

### `Math`

We can use `Xacro` files to make math expressions in the `urdf` file using basic operations **(+,-,*,/)** and **parenthesis**.

for example if there is a relation between the raduis and length of my wheel `(raduis = 2 * length)` we can only define one of these valuse and replace the other with this relation.

```xml
<xacro:property name="length" value="0.05" />

<cylinder radius="${2*length}" length="${length}"/>
```

this is most used in the `inertia` tag since it has more complex expressions so it's not convenient to calculate each value each time you change one dimension.

### `Macros`

As we said before **macros** are more like functions that we can write a chunk of code in them and call this macro multiple time with different parameters or with no parameters at all.

to define a **macro** we will use the `macro tag` which has two attributes the `name` of the macro and the `parameters` we want to define. and it looks like this.

```xml
<xacro:macro name="name_of_macro" params="param_1 param_2 param_3">
    <!-- Add your content here -->
</xacro:macro>
```

ok now we will utilize everything we have learned so far to make a **wheel macro** and call it four times to generate the four wheels of the robot.

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

Now it's time to put these links in there desired place and we will use **joints** to achieve this.
<<<<<<<< HEAD:An Introduction to URDF, Links, and Xacro/An Introduction to URDF, Links, and Xacro.md
========

## Joints

Joints serve to connect two links in a robot and describes the kinematics and dynamics of them. The primary link is designated as the `<parent>`, and the second link becomes the `<child>`. There are four types of joints:  **Fixed** ,  **Revolute** ,  **Continuous** , and  **Prismatic** . Each type defines how the `<parent>` link is related to the `<child>` link.

<p align="center">
<img src="image/joint_frame.png" width="600">

To create a joint in a URDF file:

* First create a frame and name the joint and select the joint type.

  ```xml
  <joint name="joint_name" type="joint_type">

  </joint>
  ```
* Then select the child and parent link of this joint.

  ```xml
  <joint name="joint_name" type="joint_type">
     <parent link="parent_link"/>
     <child link="child_link"/>
  </joint>

  ```
* After that, add the values of the origin point in which the origin point is the transform from the parent link to the child link and the joint origin is located at the origin of child link.

  ```xml
  <joint name="joint_name" type="joint_type">
     <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
     <parent link="parent_link"/>
     <child link="child_link"/>
  </joint>
  ```
* Then, add the axis as it spacify the axis of rotation for **revolute joints**, the axis of **translation for prismatic** joints. *Fixed joint do not use the axis field*

  ```xml
  <joint name="joint_name" type="joint_type">
     <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
     <parent link="parent_link"/>
     <child link="child_link"/>
     <axis xyz="0.0 0.0 0.0"/>
  </joint>
  ```
* If the joint type is prismatic or revolute then the a limit should be add tho the joint in which the limit has four parameters.

  * 1- `lower` : This parameter value is for the lower limit in *meters* for **parasmitic joint** and *radian* for **revolute joint**.
  * 2- `upper` : This parameter value is for the upper limit in *meters* for **parasmitic joint** and *radian* for **revolute joint**.
  * 3- `effort` : This parameter value is for the maximum effort in *N* for the **prismatic joint** and *N-m* for the **revolute joint**.
  * 4- `velocity`: This parameter value is for the maximum velocity in *m/sec* for **prismatic joint** and *rad/sec* for **revolute joint**.

  ```xml
  <joint name="joint_name" type="joint_type">
     <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
     <parent link="parent_link"/>
     <child link="child_link"/>
     <axis xyz="0.0 0.0 0.0"/>
     <limit lower="0.0" upper="0.0" effort="0.0" velocity="0.0"/>
  </joint>
  ```

## Types of joints

### Fixed

This is not really a joint because it cannot move. All degrees of freedom are locked.So, it is used to fix links that does not move.

```xml
<joint name="joint_name" type="fixed">
    <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
    <parent link="parent_link"/>
    <child link="child_link"/>
    <axis xyz="0.0 0.0 0.0"/>
</joint>
```

<p align="center">
<img src="image/fixed_joint2.png" width="600">

**Example:**

If a robot base link is a parent link and a child link for a box which can represent a camera a fixed joint is needed to fix the box on the robot base and lock its movement. So, we can take the base link code and the camera link from the links example above and add a joint as shown in the below code.

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Base link (box) -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.2 0.06"/>
      </geometry>
      <material name="">
        <color rgba="1.0 0.70 0.0 1.0"/>
        <texture filename=""/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.2 0.06"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.02 0.05 0.02"/>
      </geometry>
      <material name="">
        <color rgba="1.0 0.0 0.0 1.0"/>
        <texture filename=""/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.02 0.05 0.02"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.02"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="1.67e-6" ixy="0" ixz="0" iyy="1.67e-6" iyz="0" izz="1.67e-6"/>
    </inertial>
  </link>

  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.11 0.0 0.0" rpy="0.0 0.0 0.0"/>
  </joint>


</robot>

```

<p align="center">
<img src="image/fixed_joint.png" width="600">

### Revolute

It is a hinge joint that rotates along the axis and has a limited range specified by the upper and lower limits.

```xml
<joint name="joint_name" type="revolute">
    <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
    <parent link="parent_link"/>
    <child link="child_link"/>
    <axis xyz="0.0 0.0 0.0"/>
    <limit lower="0.0" upper="0.0" effort="0.0" velocity="0.0"/>
</joint>

```

**Example:**

If a robot base link is the parent link and a robot arm is the child link in which the robot arm wanted to revolute around an axis with limits. So, as shown in the below figure the joint between the base link and the revoluting link is revolute joint in which the link revoluting link is needed to revolute around an axis with limits in angles and velocity.

<p align="center">
<img src="image/revolute_joint2.png">

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Base link (box) -->
  <link name="base_link">
    <visual>
      <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
      <geometry>
        <box size="0.05 0.05 0.3"/>
      </geometry>
      <material name="">
        <color rgba="1.0 0.70 0.0 1.0"/>
        <texture filename=""/>
      </material>
    </visual>
    <collision>
      <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
      <geometry>
        <box size="0.05 0.05 0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.0079167" ixy="0" ixz="0" iyy="0.0079167" iyz="0" izz="0.0004167"/>
    </inertial>
  </link>

  <link name="child_link">
    <visual>
      <origin xyz="0.0 0.0 0.08" rpy="0.0 0.0 0.0"/>
      <geometry>
        <box size="0.02 0.05 0.2"/>
      </geometry>
      <material name="">
        <color rgba="1.0 0.0 0.0 1.0"/>
        <texture filename=""/>
      </material>
    </visual>
    <collision>
      <origin xyz="0.0 0.0 0.08" rpy="0.0 0.0 0.0"/>
      <geometry>
        <box size="0.02 0.05 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.02"/>
      <origin xyz="0.0 0.0 0.08" rpy="0 0 0"/>
      <inertia ixx="5.6333e-5" ixy="0" ixz="0" iyy="6.8000e-5" iyz="0" izz="3.3333e-6"/>
    </inertial>
  </link>

  <joint name="revolute_joint" type="revolute">
    <parent link="base_link"/>
    <child link="child_link"/>
    <origin xyz="0.035 0.0 0.15" rpy="0.0 0.0 0.0"/>
    <axis xyz="1 0.0 0.0"/>
    <limit lower="-0.523599" upper="0.523599" effort="0.0" velocity="0.0"/>  
  </joint>

  
</robot>
```

<p align="center">
<img src="image/revolute_joint.png">

<p align="center">
<img src="image/revolute_joint.gif" width="600">

### Continuous

This is a continuous hinge joint that rotates around the axis and has no upper and lower limits.

```xml
<joint name="joint_name" type="continous">
    <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
    <parent link="parent_link"/>
    <child link="child_link"/>
    <axis xyz="0.0 0.0 0.0"/>
</joint>
```

**Example:**

If a robot base link is the parent link and a wheel is the child link in which the wheel wanted to revolute around an axis continuous without any limits. So, as shown in the below figure the joint between the base link and the continuous revoluting link is continuous joint.

<p align="center">
<img src="image/continuous_joint2.png">

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Base link (box) -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.2 0.06"/>
      </geometry>
      <material name="">
        <color rgba="1.0 0.70 0.0 1.0"/>
        <texture filename=""/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.2 0.06"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Wheel link (cylinder) -->

  <link name="wheel_link_right">
    <visual>
      <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
      <geometry>
        <cylinder radius="0.06" length="0.04"/>
      </geometry>
      <material name="">
        <color rgba="1.0 0.0 0.0 1.0"/>
        <texture filename=""/>
      </material>
    </visual>
    <collision>
      <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
      <geometry>   
        <cylinder radius="0.06" length="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.02"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="3.7333e-5" ixy="0" ixz="0" iyy="3.7333e-5" iyz="0" izz="3.6000e-5"/>
    </inertial>
  </link>

  <link name="wheel_link_left">
    <visual>
      <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
      <geometry>
        <cylinder radius="0.06" length="0.04"/>
      </geometry>
      <material name="">
        <color rgba="1.0 0.0 0.0 1.0"/>
        <texture filename=""/>
      </material>
    </visual>
    <collision>
      <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
      <geometry>   
        <cylinder radius="0.06" length="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.02"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="3.7333e-5" ixy="0" ixz="0" iyy="3.7333e-5" iyz="0" izz="3.6000e-5"/>
    </inertial>
  </link>

  <joint name="wheel_joint_right" type="continuous">
    <origin xyz="0.0 0.12 0.0" rpy="-1.5708 0.0 0.0"/> 
    <parent link="base_link"/>
    <child link="wheel_link_right"/>
    <axis xyz="0.0 0.0 1"/>
  </joint>

  <joint name="wheel_joint_left" type="continuous">
    <origin xyz="0.0 -0.12 0.0" rpy="-1.5708 0.0 0.0"/> 
    <parent link="base_link"/>
    <child link="wheel_link_left"/>
    <axis xyz="0.0 0.0 1"/>
  </joint> 
</robot>
```

<p align="center">
<img src="image/continuous_joint.png">

<p align="center">
<img src="image/continuous_joint.gif">

### Prismatic

This is a sliding joint that slides along the axis, and has a limited range specified by the upper and lower limits.

```xml
<joint name="joint_name" type="prismatic">
    <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
    <parent link="parent_link"/>
    <child link="child_link"/>
    <axis xyz="0.0 0.0 0.0"/>
    <limit lower="0.0" upper="0.0" effort="0.0" velocity="0.0"/>
</joint>
```

**Example:**

If a robot base link is the parent link and a linear actuator or sliding mechanism is the child link, a prismatic joint can be used to model their connection. This joint allows for linear movement of the child link along a specified axis relative to the parent link without any rotational constraints.

<p align="center">
<img src="image/prismatic_joint2.png">

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Base link (box) -->
  <link name="base_link">
    <visual>
      <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
      <geometry>
        <box size="0.2 0.2 0.1"/>
      </geometry>
      <material name="">
        <color rgba="1.0 0.70 0.0 1.0"/>
        <texture filename=""/>
      </material>
    </visual>
    <collision>
      <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
      <geometry>
        <box size="0.2 0.2 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.005833" ixy="0" ixz="0" iyy="0.005833" iyz="0" izz="0.006667"/>
    </inertial>
  </link>

  <link name="moving_link">
    <visual>
      <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
      <geometry>
        <box size="0.1 0.1 0.2"/>
      </geometry>
      <material name="">
        <color rgba="1.0 0.0 0.0 1.0"/>
        <texture filename=""/>
      </material>
    </visual>
    <collision>
      <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
      <geometry>
        <box size="0.1 0.1 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.02"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="7.333e-5" ixy="0" ixz="0" iyy="7.333e-5" iyz="0" izz="3.333e-5"/>
    </inertial>
  </link>

  <joint name="prismatic_joint" type="prismatic">
    <parent link="base_link"/>
    <child link="moving_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.2" upper="0.1" effort="0.0" velocity="0.0"/>
  </joint>
</robot>
```

<p align="center">
<img src="image/prismatic_joint.png">

<p align="center">
<img src="image/prismatic_joint.gif">
>>>>>>>> parent of fcba3c0 ( first reorganize the by adding every part in a folder and the examples too, then add a urdf.md file to be the main repo readme file to easily navigate throw the repo and its sub topics.then add a description for the topics in the main workshop md flie (README.md). add links to all partitions (introduction to urdf..... , joints , plugins and examples). At last adding the final gif for example under its hyperlinks in plugin.md file.):urdf-and-simulation/An Introduction to URDF, Links, and Xacro/An Introduction to URDF, Links, and Xacro.md