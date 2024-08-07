## Joints

Joints serve to connect two links in a robot and describes the kinematics and dynamics of them. The first link is designated as the `<parent>`, and the second link becomes the `<child>`. There are four types of joints:  **Fixed** ,  **Revolute** ,  **Continuous** , and  **Prismatic** . Each type defines how the `<parent>` link is related to the `<child>` link. in URDF file the joint can be writen as shown:

```bash

<joint name="joint_name" type="joint_type">
    <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
    <parent link="parent_link"/>
    <child link="child_link"/>
    <axis xyz="0.0 0.0 0.0"/>
    <limit lower="0.0" upper="0.0" effort="0.0" velocity="0.0"/>
</joint>
```

### Fixed

This is not really a joint because it cannot move. All degrees of freedom are locked.

```bash
<joint name="joint_name" type="fixed">
    <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
    <parent link="parent_link"/>
    <child link="child_link"/>
    <axis xyz="0.0 0.0 0.0"/>
    <limit lower="0.0" upper="0.0" effort="0.0" velocity="0.0"/>
</joint>
```

### Revolute

It is a hinge joint that rotates along the axis and has a limited range specified by the upper and lower limits.

```bash
<joint name="joint_name" type="revolute">
    <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
    <parent link="parent_link"/>
    <child link="child_link"/>
    <axis xyz="0.0 0.0 0.0"/>
    <limit lower="0.0" upper="0.0" effort="0.0" velocity="0.0"/>
</joint>
```

### Continous

This is a continuous hinge joint that rotates around the axis and has no upper and lower limits.

```bash
<joint name="joint_name" type="continous">
    <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
    <parent link="parent_link"/>
    <child link="child_link"/>
    <axis xyz="0.0 0.0 0.0"/>
    <limit lower="0.0" upper="0.0" effort="0.0" velocity="0.0"/>
</joint>
```

### Prismatic

This is a sliding joint that slides along the axis, and has a limited range specified by the upper and lower limits.

```bash
<joint name="joint_name" type="prismatic">
    <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
    <parent link="parent_link"/>
    <child link="child_link"/>
    <axis xyz="0.0 0.0 0.0"/>
    <limit lower="0.0" upper="0.0" effort="0.0" velocity="0.0"/>
</joint>
```

### Floating

This is a joint  that allows motion for all 6 degrees of freedom.

```bash
<joint name="joint_name" type="floating">
    <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
    <parent link="parent_link"/>
    <child link="child_link"/>
    <axis xyz="0.0 0.0 0.0"/>
    <limit lower="0.0" upper="0.0" effort="0.0" velocity="0.0"/>
</joint>
```

### Planar

This is a joint that allows motion in a plane perpendicular to the axis.

```bash
<joint name="joint_name" type="planar">
    <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
    <parent link="parent_link"/>
    <child link="child_link"/>
    <axis xyz="0.0 0.0 0.0"/>
    <limit lower="0.0" upper="0.0" effort="0.0" velocity="0.0"/>
</joint>
```
