---
sidebar_label: 'Physical AI Edge Kit'
sidebar_position: 2
---

# Hardware Setup: Physical AI Edge Kit (Jetson)

## Overview

NVIDIA Jetson Orin Nano is ideal for edge robotics applications.

## Specifications

- **GPU**: 1024-core NVIDIA Ampere
- **CPU**: 6-core Arm Cortex-A78AE
- **RAM**: 8GB LPDDR5
- **Storage**: MicroSD / NVMe SSD
- **Power**: 7-15W

## Setup

1. Flash JetPack 6.0
2. Install ROS 2 Humble (ARM64)
3. Configure power mode for performance

```bash
sudo nvpmodel -m 0  # Max performance
sudo jetson_clocks   # Lock clocks
```

## Limitations   

- Limited compute compared to desktop GPU
- Focus on optimized models (TensorRT)

See [NVIDIA Jetson Docs](https://developer.nvidia.com/embedded/jetson-orin-nano-developer-kit)
