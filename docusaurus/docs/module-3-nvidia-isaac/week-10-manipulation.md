---
sidebar_label: 'Week 10: Manipulation'
sidebar_position: 3
estimated_time: '5 hours'
week: 10
module: 3
learning_objectives:
  - Implement motion planning
  - Control robot manipulators
  - Perform grasping tasks
---

# Week 10: Robot Manipulation

## Motion Planning with MoveIt 2

```python
from omni.isaac.motion_generation import ArticulationMotionPolicy

# Create motion planner
policy = ArticulationMotionPolicy(robot)

# Plan to target
target_pose = [0.5, 0.3, 0.2]  # x, y, z
trajectory = policy.compute_path_to_target(target_pose)

# Execute
robot.follow_trajectory(trajectory)
```

## Grasping

```python
# Detect object
obj_pose = perception.detect_object()

# Plan grasp
grasp_pose = planner.compute_grasp(obj_pose)

# Execute pick
robot.move_to(grasp_pose + [0, 0, 0.1])  # Pre-grasp
robot.move_to(grasp_pose)  # Grasp
gripper.close()
robot.move_to(grasp_pose + [0, 0, 0.2])  # Lift
```

## Next: [Module 4: VLA & Humanoids](/docs/module-4-vla-humanoids/week-11-vla)
