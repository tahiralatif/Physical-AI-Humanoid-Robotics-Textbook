---
sidebar_label: 'Week 9: Perception Systems'  
sidebar_position: 2
estimated_time: '5 hours'
week: 9
module: 3
learning_objectives:
  - Implement object detection pipelines
  - Use Isaac Sim synthetic data
  - Train perception models
---

# Week 9: Perception with Isaac

## Computer Vision Pipeline

```python
import cv2
import numpy as np
from omni.isaac.sensor import Camera

# Create camera
camera = Camera("/World/Camera")

# Capture image
rgb = camera.get_rgba()[:, :, :3]

# Object detection (example with YOLO)
from ultralytics import YOLO
model = YOLO('yolov8n.pt')
results = model(rgb)
```

## Synthetic Data Generation

Generate training data in Isaac Sim:

- **Domain randomization**: Vary lighting, textures
- **Automatic labeling**: Bounding boxes, segmentation
- **Scale**: Generate 1000s of images

## Assessment

[Isaac Perception Assessment](/docs/assessments/isaac-perception)

## Next: [Week 10: Manipulation](./week-10-manipulation.md)
