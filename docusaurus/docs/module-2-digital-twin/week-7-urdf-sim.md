---
sidebar_label: 'Week 7: URDF & Simulation'
sidebar_position: 2
estimated_time: '5 hours'
week: 7
module: 2
learning_objectives:
  - Create robot models with URDF
  - Define joints and links
  - Simulate robots in Gazebo
---

# Week 7: URDF Modeling & Simulation

## URDF (Unified Robots Description Format)

URDF is an XML format for describing robot geometry, kinematics, and dynamics.

## Basic URDF Structure

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.6 0.4 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.6 0.4 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="0.4" ixy="0" ixz="0" iyy="0.4" iyz="0" izz="0.2"/>
    </inertial>
  </link>
  
  <!-- Wheel link -->
  <link name="wheel_left">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
  </link>
  
  <!-- Joint connecting wheel to base -->
  <joint name="wheel_left_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_left"/>
    <origin xyz="0 0.3 0" rpy="0 1.57 0"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>
```

## Joint Types

- **fixed**: No motion
- **revolute**: Rotation with limits
- **continuous**: Unlimited rotation (wheels)
- **prismatic**: Linear motion (sliders)

## Xacro: Programmable URDF

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="robot">
  <!-- Macro for wheels -->
  <xacro:macro name="wheel" params="prefix reflect">
    <link name="${prefix}_wheel">
      <visual>
        <geometry>
          <cylinder radius="0.1" length="0.05"/>
        </geometry>
      </visual>
    </link>
    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="base_link"/>
      <child link="${prefix}_wheel"/>
      <origin xyz="0 ${reflect*0.3} 0"/>
    </joint>
  </xacro:macro>
  
  <xacro:wheel prefix="left" reflect="1"/>
  <xacro:wheel prefix="right" reflect="-1"/>
</robot>
```

## Gazebo Integration

Add Gazebo-specific tags:

```xml
<gazebo reference="base_link">
  <material>Gazebo/Blue</material>
  <mu1>0.5</mu1>  <!-- Friction -->
  <mu2>0.5</mu2>
</gazebo>

<gazebo reference="camera_link">
  <sensor type="camera" name="camera1">
    <update_rate>30.0</update_rate>
    <camera>
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>800</width>
        <height>800</height>
      </image>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <frame_name>camera_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

## Assessment

Complete [Gazebo Simulation Assessment](/docs/assessments/gazebo-simulation).

## Next: [Module 3: NVIDIA Isaac](/docs/module-3-nvidia-isaac/week-8-intro)
