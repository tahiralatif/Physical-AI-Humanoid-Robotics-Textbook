---
sidebar_label: 'Cloud-Native Setup'
sidebar_position: 3
---

# Hardware Setup: Cloud-Native (AWS/Azure)

## When to Use Cloud

- No local GPU available
- Need more compute power
- Team collaboration

## AWS Setup

### EC2 Instance

- **Type**: g5.xlarge (NVIDIA A10G)
- **OS**: Ubuntu 22.04 Deep Learning AMI
- **Cost**: ~$1.00/hour

### Launch Instance

```bash
# SSH into instance
ssh -i key.pem ubuntu@<instance-ip>

# ROS 2 already installed in Deep Learning AMI
source /opt/ros/humble/setup.bash

# Install Gazebo
sudo apt install ros-humble-gazebo-ros-pkgs
```

## Cost Optimization

- Use spot instances (70% cheaper)
- Stop instances when not in use
- Use smaller instances for non-GPU tasks

## Remote Desktop

```bash
# Install NoMachine or x2go for GUI access
sudo apt install x2goserver
```

Budget: ~$50-100 for full course
