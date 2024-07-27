# Robotic simulation

Robotic simulation involves creating a virtual model that mimics real-world processes. By using simulation, we can design a virtual representation of a robot and test its design and programming in a controlled environment.

Robotic simulators are software applications that create models of robots and render virtual environments that replicate the real-world settings in which the robots will operate. In our scenario, the environment is a typical hotel or restaurant with tables and chairs. We need to replicate this setup in the simulator to test the robot's functionality. One commonly used robotic simulator for such tasks is **`Gazebo`**.

# Gazebo

**Gazebo** is an open-source 3D robotics simulator. Gazebo simulated real-world physics in a high fidelity simulation.

Gazebo is a physics-based, high fidelity 3D simulator for robotics. Gazebo provides the ability to accurately simulate one or more robots in complex indoor and outdoor environments filled with static and dynamic objects.

Gazebo simulated real-world physics in a high fidelity simulation.

Robotics simulation is an ever-growing space. Companies are investing more and more money to improve their workflow through robotic simulation.

Robotic simulation saves a lot of time and money because it allows people to test how robots work without huge investments.

We have the capability to employ Gazebo for `building` both our `robot` model and the `environment/world` model.

### Gazebo Architecture

Gazebo runs two processes:

- **Gazebo Server**:
  The first main component running a
  Gazebo simulation is the Gazebo
  Server or also known by gzserver. It is
  responsible for simulating(robots,
  sensors, objects, ….)
- **Gazebo Client**:
  The second main component running an
  of a Gazebo simulation is the Gazebo
  Client or also known by gzclient. gzclient
  provides the very essential Graphical
  Client that connects to the gzserver and
  renders the simulation scene.

<p align="center">
<img src="images/1.png">

To run gazebo server type:

```bash
gzserver
```

This will start the physics engine with an empty world.

To run gazebo GUI client type in anther terminal:

```bash
gzclient
```

This will connect t the server and give you a graphical display of the simulation.

<p align="center">
<img src="images/2.png">

Actually you can launch server and clinet with single command:

```bash
gazebo
```

### Running Gazebo from Ros

```sh
roscore
```

```bash
rosrun gazebo_ros gazebo
```

After starting **Gazebo**, we will see the following **topics** generated. Using the rostopic command, we will find the following list of topics:

```sh
rostopic list
```

> The preceding command line prints the following information

```sh
/gazebo/link_states
/gazebo/model_states
/gazebo/parameter_descriptions
/gazebo/parameter_updates
/gazebo/performance_metrics
/gazebo/set_link_state
/gazebo/set_model_state
```

#### Gazebo User Interface

- **`World` :**
  This tab displays the lights and models currently in the scene.
  you can view or edit its basic parameters like position and
  orientation. In addition, you can also change the physics of the
  scene like gravity and magnetic field via the Physics option.
- **`Insert` :**
  you will find objects (models) to add to the simulation scene.
- **`Select mode` :**
  Select mode is the most commonly used cursor mode. It allows you to navigate the scene
- **`Translate mode`:**
  change an object's position is to select the object in the world tab on the side panel and
  then change its pose via properties.
- **`Rotate mode` :**
  Similar to translation mode, this mode allows you to change the orientation of any given model.
- **`Scale mode` :**
  Scale mode allows you to change the scale, and hence, the overall size of any model.
- **`Undo/Redo `:**
  the undo tool helps us revert our mistakes. the redo tool used if you didn’t make something
  that you need.
- **`Simple shapes` :**
  You can insert basic 3D models like Boxs, spheres, or cylinders into the scene.
- **`Lights` :**
  Add different light sources like a spotlight, point light, or directional light to the scene.
- **`Copy/Paste` :**
  These tools let you copy/paste models in the scene. Or, you can simply press
  Ctrl+C to copy and Ctrl+V to paste any model.
- **`Align` :**
  This tool allows you to align one model with another along with one of the three principal axes.
- **`Change view` :**
The change view tool lets you view the scene from different
perspectives like top view, side view, front view, bottom view.
<p align="center">
<img src="images/3.png">

### Building robot models in Gazebo

click on Edit and select model editor

- Create the robot chassis:

  - Select from simple shapes “Box”.
  - Double click on a Box shape.
  - Click on visual and go geometry.
  - Change x = 1m, y = 0.8m , z = 0.2m.
  - Click on collision and go geometry.
  - Change x = 1m, y = 0.8m , z = 0.2m.
  <p align="center">
  <img src="images/4.png">

- Create robot wheels:

  - Select from simple shapes “cylinder”.
  - Double click on the cylinder shape.
  - Click on visual and go geometry.
  - Change radius = 0.2m , length = 0.1m.
  - go pose and change roll Rotate the wheel to 90 degree = 1.570700 rad.
  - Click on collision and go geometry.
  - Change radius = 0.2m , length = 0.1m.
  - go pose and change roll Rotate the wheel to 90 degree = 1.570700 rad.
  <p align="center">
  <img src="images/5.png">

- Connect wheels to the chassis via joints:

  - Select Joint from the toolbar.
  - Chage joint type to “Revolute”.
  - Choose parent as robot chassis and child as a wheel.
  - Align links x, y, z, And choose a suitable rotate axis.
  <p align="center">
  <img src="images/6.png">

- Adding Camera:

  - Select from simple shapes “Box”.
  - Double click on a Box shape.
  - Click on visual and go geometry.
  - Change x = 0.1, y = 0.1 , z = 0.1.
  - Click on collision and go geometry.
  - Change x = 0.1, y = 0.1 , z = 0.1.
  <p align="center">
  <img src="images/7.png">

- Connect Camera to the chassis via joints:

  - Select Joint from the toolbar.
  - Chage joint type to “fixed”.
  - Choose parent as robot chassis and child as a Camera.
  - Align links x, y, z, And choose a suitable rotate axis.
  <p align="center">
  <img src="images/8.png">

- Save the model file

  - Model: Save it as a robot in <your_pkg_ws/model>
  - Exit the Model Editor

    <p align="center">
    <img src="images/9.png">

To initiate movement of the robot, simply double-click on the robot and select "Apply Force and Torque."

<p align="center">
<img src="images/10.gif">

### Building world models in Gazebo

The Building Editor allows you to create modeBls of multi-level builings without write any code.

Builings can contin doorways, windows and stairs.

To access the Building Editor, go to the Edit menu and select Building Editor.

<p align="center">
<img src="images/11.gif">

> Final custom model

<p align="center">
<img src="images/12.png">

### include custom models for the world

Click on the toolbar, select `Insert` and then choose `Robot` and `World` to create them as per your requirements.
then save this world such as <`my_world.world`>

you can open this file using this line

```
gazebo <your_pkg_ws/my_world.world>
```

<p align="center">
<img src="images/13.png">
