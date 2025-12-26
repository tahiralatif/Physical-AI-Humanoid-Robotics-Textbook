---
sidebar_label: 'Digital Twin Workstation'
sidebar_position: 1
---

# Hardware Setup: Digital Twin Workstation

## Recommended Specifications

### Minimum
- **CPU**: Intel i5 / AMD Ryzen 5
- **RAM**: 16GB
- **GPU**: NVIDIA RTX 3060 (6GB VRAM)
- **Storage**: 256GB SSD
- **OS**: Ubuntu 22.04 LTS

### Recommended
- **CPU**: Intel i7 / AMD Ryzen 7
- **RAM**: 32GB
- **GPU**: NVIDIA RTX 4070 (12GB VRAM)
- **Storage**: 512GB NVMe SSD

## Installation Guide

### 1. Install Ubuntu 22.04

Download from [ubuntu.com](https://ubuntu.com/download/desktop)

### 2. Install NVIDIA Drivers

```bash
sudo apt update
sudo apt install nvidia-driver-535
sudo reboot
```

### 3. Install ROS 2 Humble

```bash
sudo apt install ros-humble-desktop
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### 4. Install Gazebo

```bash
sudo apt install ros-humble-gazebo-ros-pkgs
```

### 5. Install Isaac Sim (Optional)

Download from [NVIDIA Developer Portal](https://developer.nvidia.com/isaac-sim)

## Verification

```bash
# Test ROS 2
ros2 run demo_nodes_cpp talker

# Test Gazebo
gazebo

# Test GPU
nvidia-smi
```

You're ready to start the course!
