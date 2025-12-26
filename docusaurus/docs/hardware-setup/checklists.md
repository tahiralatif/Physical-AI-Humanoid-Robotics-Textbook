# Weekly Hardware & Software Checklist

To succeed in this course, ensure your workstation and lab environment meet these standards.

## 🖥️ "Digital Twin" Workstation (Simulation)

This machine runs **NVIDIA Isaac Sim**, **Gazebo**, and **Unity**. It handles the heavy lifting of physics and visual rendering.

| Component | Minimum Requirement | Recommended (High Performance) |
| :--- | :--- | :--- |
| **GPU** | NVIDIA RTX 3070 (8GB VRAM) | **NVIDIA RTX 4080 (16GB VRAM)** |
| **CPU** | Intel i7-12700K | **Intel i9-13900K / Ryzen 9 7900X** |
| **RAM** | 32 GB DDR4 | **64 GB DDR5** |
| **Storage** | 500 GB NVMe SSD | **1 TB NVMe Gen4 SSD** |
| **OS** | Ubuntu 22.04 LTS | **Ubuntu 22.04 LTS (Optimized kernels)** |

### Mandatory Software
- [ ] **NVIDIA Drivers**: Version 525+ (Production Branch)
- [ ] **Docker & NVIDIA Container Toolkit**: For containerized Isaac Sim
- [ ] **VS Code**: With ROS, Python, and C++ extensions
- [ ] **Git**: Configured with GitHub SSH keys

---

## 🧠 "Physical AI" Edge Kit (Deployment)

This is the "Brain" placed inside the robot for real-time inference and hardware control.

| Component | Choice A: High Performance | Choice B: Budget/Efficient |
| :--- | :--- | :--- |
| **Computer** | **NVIDIA Jetson Orin Nano (8GB)** | Raspberry Pi 5 (8GB) + Coral TPU |
| **Vision** | **Intel RealSense D435i** | Luxonis OAK-D Lite |
| **Storage** | 128 GB NVMe | 64 GB MicroSD (High Speed) |
| **Power** | 19V DC Supply / LiPo Battery | 5V/5A USB-C |

---

## 🤖 Robot Platforms (Actuators)

Depending on your budget and facility, choose one of the following paths:

### Path 1: Simulation Only (Free)
- **Model**: Custom URDF (provided) or Unitree G1 Sim.
- **Tools**: Gazebo + ROS 2 Humble.

### Path 2: Miniature Humanoid ($$$)
- **Model**: **Unitree G1 Humanoid**.
- **Focus**: Sim-to-Real transfer, bipedal balance, and manipulation.

### Path 3: Quadruped Proxy ($$)
- **Model**: **Unitree Go2 Edu**.
- **Focus**: Terrain navigation and VLA-to-Legged-Motion.

---

## 🛠️ Essential Tools & Sensors
- **IMU**: BNO055 (for orientation)
- **Lidar**: LD19 or Slamtec RPLidar A1 (for 2D SLAM)
- **Audio**: ReSpeaker USB Mic Array (for Voice Commands)
- **Wireless**: 5GHz WiFi Router (Dedicated for Robot communication)
