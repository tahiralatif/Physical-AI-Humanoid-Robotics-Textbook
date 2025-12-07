---
sidebar_label: 'Troubleshooting Guide'
---

# Troubleshooting Guide

## ROS 2 Issues

### "Package not found"

```bash
source install/setup.bash
colcon build --packages-select my_package
```

### "No executable found"

Check `setup.py` entry points.

## Gazebo Issues

### Black screen

Update GPU drivers or run:
```bash
export SVGA_VGPU10=0
```

### Slow simulation

Reduce physics update rate in world file.

## Isaac Sim Issues

### Out of memory

Reduce scene complexity or use lower resolution.

##More Help

- [ROS Answers](https://answers.ros.org)
- [Gazebo Community](https://community.gazebosim.org)
- [Isaac Sim Forums](https://forums.developer.nvidia.com/c/isaac-sim)
