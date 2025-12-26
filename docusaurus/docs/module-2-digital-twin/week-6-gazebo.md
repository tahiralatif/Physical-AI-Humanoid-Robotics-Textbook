---
sidebar_label: 'Week 6: Gazebo Simulation'
sidebar_position: 1
estimated_time: '4 hours'
week: 6
module: 2
prerequisites:
  - Module 1 completed
learning_objectives:
  - Launch and navigate Gazebo simulator
  - Create world files and environments
  - Spawn robots and objects
---

# Week 6: Gazebo Simulation Basics

## Introduction to Gazebo

**Gazebo** is a robotics simulator that provides:
- **Physics simulation**: Realistic dynamics, collisions
- **Sensor simulation**: Cameras, LIDAR, IMU
- **ROS 2 integration**: Seamless connection

## Why Simulation?

1. **Safe testing**: No risk to hardware
2. **Rapid iteration**: Test changes instantly
3. **Scenario testing**: Reproduce edge cases
4. **Cost effective**: No physical hardware needed

## Launching Gazebo

```bash
# Install Gazebo
sudo apt install ros-humble-gazebo-ros-pkgs

# Launch empty world
gazebo --verbose

# Launch with ROS 2
ros2 launch gazebo_ros gazebo.launch.py
```

## World Files

Create `my_world.world`:

```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <world name="default">
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <!-- Add a box -->
    <model name="box">
      <pose>0 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>1 1 1</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>1 1 1</size></box>
          </geometry>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

## Spawning Robots

```python
from gazebo_msgs.srv import SpawnEntity

class RobotSpawner(Node):
    def__init__(self):
        super().__init__('robot_spawner')
        self.client = self.create_client(SpawnEntity, '/spawn_entity')
        
    def spawn_robot(self):
        request = SpawnEntity.Request()
        request.name = 'my_robot'
        request.xml = robot_urdf_string
        request.initial_pose.position.z = 1.0
        
        self.client.call_async(request)
```

## Key Takeaways

Gazebo enables safe,cost-effective robot testing in realistic environments.

## Next: [Week 7: URDF & Sim](./week-7-urdf-sim.md)
