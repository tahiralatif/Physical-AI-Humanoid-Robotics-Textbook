---
sidebar_label: 'Week 8: Isaac Sim Introduction'
sidebar_position: 1
estimated_time: '4 hours'
week: 8
module: 3
prerequisites:
  - Modules 1-2 completed
  - NVIDIA GPU recommended
learning_objectives:
  - Set up NVIDIA Isaac Sim
  - Understand photorealistic simulation
  - Use Omniverse platform
---

# Week 8: NVIDIA Isaac Sim Introduction

## What is Isaac Sim?

**Isaac Sim** is NVIDIA's robotics simulation platform built on Omniverse, offering:

- **Photorealistic rendering**: Ray-traced graphics
- **Physics-accurate simulation**: PhysX 5 engine
- **ROS 2 integration**: Native bridge
- **Synthetic data generation**: For ML training

## Installation

```bash
# Download from NVIDIA website
# Requires: Ubuntu 20.04/22.04, RTX GPU

# Launch Isaac Sim
~/.local/share/ov/pkg/isaac_sim-*/isaac-sim.sh
```

## Creating Your First Scene

```python
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid

# Create world
world = World()

# Add cube
cube = world.scene.add(
    DynamicCuboid(
        prim_path="/World/Cube",
        position=[0, 0, 1.0],
        size=0.5
    )
)

# Run simulation
world.reset()
while simulation_app.is_running():
    world.step(render=True)
```

## ROS 2 Bridge

Enable bi-directional communication:

```python
from omni.isaac.core.utils.extensions import enable_extension
enable_extension("omni.isaac.ros2_bridge")

# Publish/subscribe to ROS 2 topics
# Camera, LIDAR data automatically published
```

## Key Takeaways

Isaac Sim provides photorealistic simulation with GPU acceleration.

## Next: [Week 9: Perception](./week-9-perception.md)
