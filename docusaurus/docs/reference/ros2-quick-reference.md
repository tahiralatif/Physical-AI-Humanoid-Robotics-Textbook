---
sidebar_label: 'ROS 2 Quick Reference'
---

# ROS 2 Quick Reference

## Common Commands

```bash
# Nodes
ros2 node list
ros2 node info /node_name

# Topics
ros2 topic list
ros2 topic echo /topic
ros2 topic hz /topic

# Services
ros2 service list
ros2 service call /service

# Parameters
ros2 param list
ros2 param get /node param_name
ros2 param set /node param_name value

# Packages
ros2 pkg create my_package
ros2 pkg list

# Launch
ros2 launch package_name launch_file.py

# Build
colcon build
colcon build --packages-select my_package
```

## Message Types Cheat Sheet

```python
# std_msgs
from std_msgs.msg import String, Int32, Float64

# geometry_msgs
from geometry_msgs.msg import Pose, Twist, Point

# sensor_msgs
from sensor_msgs.msg import Image, LaserScan, Joy
```

Print and keep handy!
