Author: Yousef Asal

Review: KG

# URDF

**URDF** (unified robot description format) is an `XML` file format that includes the robot description in it.

- it's the format that ros uses to identify any robot and its interactions with the environment.
- provides details on the physical characteristics of the robot in addition to all the information about the shape, geometry, and colors of the robot.
- does not only contain the characteristics of the body of the robot but it can also contain informations about the sensors and motors of the robot.
- can contain some plugins to make it compatable with different programs like gazebo.
- is used to build the transformation tree (**TF**) for the robot so that `ROS` and other systems can interact with it.

**Extensible Markup Language (XML)** uses markup symbols, called tags to define data.

- Each instance of an `XML tag` is called an element.
- In an `XML` file, elements are arranged in a hierarchy, which means that elements can contain other elements.
- The topmost element is called the “root” element and contains all other elements, which are called “child” elements.
- **XML** itself can't interact with the data it stores in any way so we will need another software to utilize these information such as rviz and gazebo in our case

The main purpose of **XML** language is to store data in an easy way that human can read and modefy and also the computers can understand.

## Describing the robot

- **URDF** describes a robot as a set of links connected by joints, which define the relative motion between the links. To describe a robot in a URDF file, it must be divided into links and joints.
- **URDF** can represent:
  - The kinematic and dynamic description of the robot.
  - The `visual` representation of the robot.
  - The `collision` model of the robot.
- As **URDF** files are based on `XML` format, different **XML tags** are used to create the robot description.
- A typical URDF file contains three main tags:
  - **robot**
  - **links**
  - **joints**

<p align="center">
<img src="images/link_joint.png">

before we start writing our robot description we will need some extintions that will help us write our descrioption and visualize it along the way so we have a better idea of what we are making

there are three main extentions that we will need which are

- **ROS** extintion by microsoft (this is the extintion that contain the urdf preview that lets us visualize our robot)
- **ROS snippets** by pijar (this extintions has some snippets that helps us write our description more effeciently)
- **URDF** by smilerobotics (this extintion also provides some premade structures that helps us write our code faster)

    <p align="center">
    <img src="images/Extentions.gif">

  > **Note** that when you install the `ROS` extention you will have to chage the version to prerelease version in order for the `urdf preview` to work and after we change it we'll have to restart `vscode` for in order for our changes to take place.

    <p align="center">
    <img src="images/prerelease.gif" width="1000">

after we have installed everything now we are ready to start descriping our robot.

### Robot tag

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

<p align="center">
<img src="images/robot_tag.gif">

now we can start the **ros preview** to see how our robot looks like in every step as we go. and we can do that by clicking `Shift+Ctrl+p` and type `ROS: URDF Preview` that will open a new tab that show your robot

<p align="center">
<img src="images/ROS_preview.gif" >

As we can see the preview screen is empty because we didn't add any links yet so now we will start to build our robot

## Link

The link of the robot represents the physical component of the robot.The link tag represents a single link of a robot. Using this tag, we can model a robot link and its properties. These properties include:

- A. visual

- B. collision
- c. inertial

  we can have multible instances of the `collision` and `visual` tags in the same link if we want but only one `inertial` tag per link.

  all three tags are optional meaning that for example you can have a link with just `visual` but no `collision` nor `inertial` tags.

  ```xml
  <link name="<link_name>">
  <inertial>...........</inertial>
  <visual> ............</visual>
  <collision>..........</collision>
  </link>
  ```

    <p align="center">
    <img src="images/link.png">

### A. Visual

the visual tag describes the actual shape. here we can specify the size, shape, and color of the robot.

It has three main tags which are:

- ### Geometry

  describes the `shape` and `size` of the display range centered on the origin coordinates

  we have a set of different shapes in the **URDF format** which are:

  - **box**: has three attributes the `length`, `width`, and `height`.
  - **cylinder**: has two attributes which are the `raduis`, and `length`.
  - **sphere**: has one attribute the `raduis`.
  - **mesh**: here we can use an existing mesh by adding the `file path`. also it's recommended that the mesh's format would be `.dae` file as this format provide the best texture and color support.

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

- ### Origin

  defines the reference frame of the `visual` relative to the reference frame of the link.

  has six attributes which are `xyz` and `rpy` and the default of these values are zeros.

  ```xml
  <origin rpy="0 0 0" xyz="0 0 0"/>
  ```

- ### Material

  here we can specify two tags for the visual of our link which are:

  - **color**: The color tag is used to set the color by entering a number between `0.0` and `1.0` corresponding to `red`, `green`, and `blue` in the rgba option. The last number is the `transparency` (alpha), which has a value between `0.0` and `1.0`. If it is `1.0`, it means that the original color is displayed as is without using the transparent option.
  - **texture**: is specified by a file name

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

    <p align="center">
    <img src="images/link.gif">

## B. Collision

- **Collision** describes the physical properties of the link that the simulation uses to detect and respond to interactions.
- The **collision** aspect is crucial for the simulation's behavior, as it dictates how the link is treated in terms of physics and interactions.
- For example:
  - If a link has a box geometry in the **visual** tag, but a cylinder geometry in the **collision** tag:
    - **Users** will see the link as a box.
    - **The simulation** will treat and react to the link as a cylinder.
- In summary, **collision** defines how the simulation handles the link, separate from its visual representation.

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

<p align="center">
<img src="images/collision.png" width="600">

- The **visual** and **collision** tags in `URDF` files are similar, sharing attributes like geometry and origin.
- The **collision** tag does not include a material attribute, as the `material` is primarily for aesthetic purposes, which are not needed by the simulator.
- To ensure the robot's actions in simulations match its appearance, the **collision** tag should generally match the **visual** tag.
- If a complex mesh is used for the `geometry` of the **visual** tag (e.g., a robot arm), it is not recommended to use the same mesh for the **collision** tag.
- Using a complex mesh for **collision** can degrade simulation performance, making it slower.
- In most cases, it is better to use a simplified geometry for the **collision** tag, provided it does not alter the physical properties of the link.

<p align="center">
<img src="images/Robot arm.png"  width="600">

- ### Inertial :

  here we can specify informations about the link's `mass`, `center of mass`, and `inertia`.

  has three main tags which are:

- ### origin :

  this describes the center of mass relative to the link frame.

  it has six attributes `xyz` for position and `rpy` for orientation.

- ### mass :

  defines the weight of the link(mass, unit: kg), and it has only one attribute the link's mass

- ### inertia :

  - this takes a matrix that contains information about the moment of inertia for the link around the `xyz` axes. it also contains the products of inertia about the center of mass.
  - note that **URDF** assumes a negative product of inertia convention.
  - the inertia matrix is a `3*3 matrix` that has these values.

<p align="center">
<img src="images/Inertia matrix.png" width="200">

But as we see in the code it only has **6 attributes** and thats because it's a symetrical matrix so we don't have to define the nondiagonal elements twice.

```xml
<inertial>
  <mass value="10"/>
  <inertia ixx="0.4" ixy="0.0" ixz="0.0" iyy="0.4" iyz="0.0" izz="0.2"/>
</inertial>
```

As for the values of the inertia tag we can calculate them based on the **shape**, **dimensions** and **mass** of the link and we can find the formulas to calculate these values in wikipedia in this link: [https://en.wikipedia.org/wiki/List_of_moments_of_inertia]()

<p align="center">
<img src="images/inertia.png">

the `inertial` tag is a very important tag as if we ignore it this could cause proplems in `gazebo` during simulation causing the robot model to collapse without warning, and all links will appear with their origins coinciding with the world origin.

> **Note** This inertial information can be obtained through design software or actual measurements and calculations, and is mainly used in simulations.

# Xacro

**URDF** files can be hard to manage because they often have repetitive code, especially when describing many links and joints. This can make changes difficult. `Xacro` is a tool that helps by reducing this repetitive work.

The **xacro** file is an abbreviation for `XML` Macro.

**Benefits of Using Xacro:**

- **Less Repetition:** `Xacro` lets you create macros to avoid writing the same code over and over. This makes your `URDF` file easier to read and manage.
- **Variables:** You can use variables, like `pi`, to simplify your code and make it more flexible.
- **Separate Files:** With `Xacro`, you can keep material information and `Gazebo` settings in separate files and include them in your main `URDF` file. This keeps everything organized and easier to update.

**Example Usage:**

The robot consisits of five links the base link which is a box shaped and four wheels which are cylindercal shaped.

<p align="center">
<img src="images/robot body.png">

```xml
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

<p align="center">
<img src="images/my_robot.png">

- After writing the description for the **base link** and **wheel link** , the output is not as desired.
- The first problem is **wheel placement** , highlighting the importance of **joints** . Joints connect links and define their attachment points and relative movement.

Before getting to the details of `Xacro` first we need to know how to use it in our `urdf` files. we will have to change the robot tag and add this line to it.

```xml
<!-- we will add this line to the robot tag: xmlns:xacro="http://www.ros.org/wiki/xacro" -->
<!-- so it will look like this -->

<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="my_robot">
```

now we are ready to use `Xacro` in our file

- ### Constants

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

  - In the current setup, the **radius** and **length** of the cylinder are specified twice: once in the `visual` tag and once in the `collision` tag.
  - This redundancy means that for one wheel, these values are repeated eight times (visual and collision tags for each wheel). This approach is not convenient.
  - To simplify, we will use **constants** to replace these repeated values.
  - **Constants** are typically defined at the top of the file within the **robot** tag, before any links or joints.
  - We will use the `Xacro`tag to define constants. This tag has two attributes:
  - **Name** : The name of the constant.
  - **Value** : The value assigned to the constant.
  - In code, defining constants with the **Xacro property** tag looks like this:

    ```xml
    <xacro:property name="raduis" value=".12" />
    <xacro:property name="length" value=".05" />
    ```

  - After defining the two constants, we need to use them in our **wheel link** description.
  - To use constants in the description, replace the values with `${constant_name}`, where `constant_name` is the name of the constant you defined.
  - The syntax to include a constant in your description is: `${constant_name}`.
  - With the constants defined and used, your updated **wheel link** description will look like this:

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

- ### Math

  We can use `Xacro` files to make math expressions in the `urdf` file using basic operations **(+,-,\*,/)** and **parenthesis**.

  for example if there is a relation between the raduis and length of my wheel `(raduis = 2 * length)` we can only define one of these valuse and replace the other with this relation.

  ```xml
  <xacro:property name="length" value="0.05" />

  <cylinder radius="${2*length}" length="${length}"/>
  ```

  this is most used in the `inertia` tag since it has more complex expressions so it's not convenient to calculate each value each time you change one dimension.

- ### Macros

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

    <p align="center">
    <img src="images/Final_links.gif">

Now it's time to put these links in there desired place and we will use **joints** to achieve this.

---
<div align="center">
  <a href="https://www.youtube.com/@ArabianROSMeetup/" target="_blank">
    <img src="https://raw.githubusercontent.com/maurodesouza/profile-readme-generator/master/src/assets/icons/social/youtube/default.svg" width="72" height="60" alt="YouTube logo" />
  </a>
  <p>
    <h3>You can watch our session on YouTube by clicking the 
      <a href="https://www.youtube.com/live/MMFMuSxuZ8U?si=sfU1WxxtPeea_1Ig" target="_blank">"Part 1" Link (Starts at 30:00)</a> ,
      <a href="https://www.youtube.com/live/euiqZBUEY1U?si=TkKL5vQawJ7lCYUO" target="_blank">"Part2" Link</a>
    </h3>
  </p>
</div>

## [Next Topic →](<../B.  joints/joints.md>)

## [↩Back to main](../README.md)
