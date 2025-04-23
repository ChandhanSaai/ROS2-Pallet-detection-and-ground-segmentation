# ROS2-Pallet-detection-and-ground-segmentation

A ROS2 package for real-time pallet detection and ground segmentation using YOLOv11 models.

## Overview
This package provides two ROS2 nodes:

- Pallet Detection: Identifies and draws bounding boxes around pallets in camera feed using YOLOv11m
- Ground Segmentation: Segments and highlights the ground surface in camera feed using YOLOv11 segment

Both nodes use custom trained YOLOv11 models to perform computer vision tasks on ZED2i camera images, making them suitable for mobile robot navigation and manipulation applications.

## Features

- Real-time pallet detection using a custom YOLOv11m model
- Ground surface segmentation with color overlay using YOLOv11 segment
- Integration with ZED2i stereoscopic camera
- Optimized QoS profiles for reliable performance
- Visualization outputs for debugging and monitoring

## Prerequisites

- ROS2 (tested on Humble)
- Python 3.8+
- CUDA-capable GPU (recommended)
- ZED2i camera

## Dependencies
ROS2 packages
```
ros-humble-cv-bridge
ros-humble-sensor-msgs
ros-humble-vision-msgs
```
Python packages
```
pip install ultralytics
pip install opencv-python
pip install numpy
```

## Installation
Clone this repository:
```
git clone https://github.com/ChandhanSaai/ROS2-Pallet-detection-and-ground-segmentation.git
```
Open the node file to modify:
```
# For pallet detection
/pallet_ws/src/pallet_segmentation/pallet_segmentation/inference.py

# For ground segmentation
/pallet_ws/src/pallet_segmentation/pallet_segmentation/inference_segment.py
```

Modify the subscriber topic name for both the pallet detection and ground segmentation:
```
# Change input topic
self.image_sub = self.create_subscription(
    Image,
    '/your/custom/input/topic',  # Modify this line
    self.image_callback,
    qos_profile)
```

The trained YOLOv11 models are in the link below. You'll just need to update the model paths in the code to match your system
```
https://drive.google.com/drive/folders/1A7ZMifJnFLoJqVkPu4_7kpU6qIkRFfK3?usp=sharing
```
```
# In inference.py
self.model = YOLO('......./pallet_ws/src/pallet_segmentation/models/pallet_detection/best.pt')

# In inference_segment.py
self.model = YOLO('......./pallet_ws/src/pallet_segmentation/models/ground_segment/best.pt')

```
Build your workspace:
```
cd ~/pallet_ws
colcon build --symlink-install
source install/setup.bash
```
RUN
```
ros2 launch pallet_segmentation inference_launch.py 
```

## OUTPUT

- Topic for pallet detection
```
/inference/pallet_detection'
```
- Topic for ground Segmentation
```
'/inference/ground_segmentation'
```

## Model Performance

### Pallet Detection (YOLOv11m)
The pallet detection model has been trained on a custom dataset with the following metrics:

- Architecture: YOLOv11m (125 layers, ~20M parameters)
- Training Hardware: NVIDIA GeForce RTX 4070 (Mobile)
- Model Size: 40.5MB
Performance Metrics:

- mAP50: 0.911 (91.1%)
- mAP50-95: 0.733 (73.3%)
- Precision: 0.818
- Recall: 0.844

- Inference Speed: ~7.4ms per image (6.8ms inference + pre/post processing)

This model achieves high accuracy in detecting pallets in various environments.

### Ground Segmentation (YOLOv11-seg)
The ground segmentation model has been trained on a custom dataset with the following metrics:

- Architecture: YOLOv11-seg (113 layers, ~2.8M parameters)
- Training Hardware: NVIDIA GeForce RTX 4070 (Mobile)
Performance Metrics:

- Box mAP: 0.976 (97.6%)
- Segmentation mAP: 0.966 (96.6%)
- mAP50: 0.99 (99.0%)
- mAP50-95: 0.966 (96.6%)

- Inference Speed: ~5.0ms per image (1.0ms preprocess, 3.3ms inference, 0.7ms postprocess)

This model provides highly accurate ground surface segmentation for mobile robot navigation.


